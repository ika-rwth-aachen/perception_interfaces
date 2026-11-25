// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include "perception_msgs/displays/image_observation/image_observation_display.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

#include <image_transport/image_transport.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "perception_msgs_utils/object_access.hpp"

namespace perception_msgs {
namespace displays {

namespace {

using rviz_common::properties::StatusProperty;

constexpr const char* kAutoSensor = "auto";
constexpr double kMaxSyncWaitSeconds = 1.0;

bool validStamp(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000u;
}

std::optional<uint64_t> sensorId(const std::string& text) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) return std::nullopt;
  try {
    return std::stoull(text);
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

}  // namespace

ImageObservationDisplay::ImageObservationDisplay() {
  using namespace rviz_common::properties;

  qos_profile = rclcpp::SensorDataQoS();
  topic_property_->subProp("Reliability Policy")->setValue("Best Effort");

  // image source properties
  image_source_property_ = new EnumProperty(
      "Image Source", "Image topic",
      "Canvas behind the annotations. 'Crop mosaic' pastes the observation crops onto a blank canvas and "
      "needs no image stream.",
      this, SLOT(updateSourceVisibility()));
  image_source_property_->addOption("Image topic", static_cast<int>(ImageSource::ImageTopic));
  image_source_property_->addOption("Crop mosaic", static_cast<int>(ImageSource::CropMosaic));
  image_source_property_->addOption("Blank", static_cast<int>(ImageSource::Blank));

  image_topic_property_ = new RosTopicProperty(
      "Image Topic", "", "sensor_msgs/msg/Image",
      "Camera image the observation refers to (same resolution or a uniformly resized stream).",
      image_source_property_, SLOT(updateImageSubscription()), this);

  image_transport_property_ = new EditableEnumProperty(
      "Transport", "raw", "image_transport transport used to subscribe to the image topic.", image_source_property_,
      SLOT(updateImageSubscription()), this);
  for (const char* transport : {"raw", "compressed", "zstd", "theora"}) image_transport_property_->addOption(transport);

  sync_mode_property_ = new EnumProperty(
      "Synchronization", "Synchronized",
      "Synchronized: show the image whose stamp matches the observation (lags by detector latency). "
      "Latest image: show the newest image with the latest observation.",
      image_source_property_, SLOT(queueRender()), this);
  sync_mode_property_->addOption("Synchronized", static_cast<int>(SyncMode::Synchronized));
  sync_mode_property_->addOption("Latest image", static_cast<int>(SyncMode::LatestImage));

  sync_tolerance_property_ = new FloatProperty(
      "Tolerance [ms]", 20.0,
      "Maximum stamp difference between image and observation. Older annotations are drawn as stale.",
      image_source_property_, SLOT(queueRender()), this);
  sync_tolerance_property_->setMin(0.0);

  image_buffer_size_property_ = new IntProperty(
      "Image Buffer", 30, "Number of images kept for matching delayed evidence.", image_source_property_,
      SLOT(queueRender()), this);
  image_buffer_size_property_->setMin(1);
  image_buffer_size_property_->setMax(300);

  // filter properties
  filter_property_ = new Property("Filter", QVariant(), "Select which evidence is drawn.", this);

  sensor_id_property_ = new EditableEnumProperty(
      "Sensor ID", kAutoSensor,
      "Camera whose evidence is drawn. 'auto' matches the image frame_id, or picks the lowest sensor ID "
      "without an image.",
      filter_property_, SLOT(queueRender()), this);
  sensor_id_property_->addOption(kAutoSensor);

  min_existence_property_ = new FloatProperty(
      "Min Existence Probability", 0.0, "Hide objects below this existence probability.", filter_property_,
      SLOT(queueRender()), this);
  min_existence_property_->setMin(0.0);
  min_existence_property_->setMax(1.0);

  show_stale_property_ = new BoolProperty(
      "Show Stale", true, "Draw evidence that does not match the image stamp dashed instead of hiding them.",
      filter_property_, SLOT(queueRender()), this);

  // object appearance properties
  appearance_properties_ = new Property(
      "Appearance Properties", " ", "Different properties to modify the appearance of objects", this);
  box_property_ = new BoolProperty(
      "Bounding box", true, "Draw the 2D bounding box.", appearance_properties_, SLOT(queueRender()), this);
  box_line_width_property_ =
      new FloatProperty("Line Width", 2.0, "Box line width [px].", box_property_, SLOT(queueRender()), this);
  box_line_width_property_->setMin(0.5);
  box_fill_alpha_property_ =
      new FloatProperty("Fill Alpha", 0.0, "Box fill opacity.", box_property_, SLOT(queueRender()), this);
  box_fill_alpha_property_->setMin(0.0);
  box_fill_alpha_property_->setMax(1.0);

  mask_property_ = new BoolProperty(
      "Mask", true, "Draw the instance mask.", appearance_properties_, SLOT(queueRender()), this);
  mask_mode_property_ =
      new EnumProperty("Mode", "Filled + contour", "Mask style.", mask_property_, SLOT(queueRender()), this);
  mask_mode_property_->addOption("Filled", static_cast<int>(rendering::MaskMode::Filled));
  mask_mode_property_->addOption("Contour", static_cast<int>(rendering::MaskMode::Contour));
  mask_mode_property_->addOption("Filled + contour", static_cast<int>(rendering::MaskMode::FilledAndContour));
  mask_alpha_property_ =
      new FloatProperty("Alpha", 0.4, "Mask fill opacity.", mask_property_, SLOT(queueRender()), this);
  mask_alpha_property_->setMin(0.0);
  mask_alpha_property_->setMax(1.0);

  crop_property_ = new BoolProperty(
      "Crop", false, "Paste the observation crop at its ROI.", appearance_properties_, SLOT(queueRender()), this);
  roi_property_ = new BoolProperty(
      "ROI", false, "Draw the crop/mask source rectangle.", appearance_properties_, SLOT(queueRender()), this);

  label_property_ = new BoolProperty(
      "Label", true, "Draw a text label per evidence.", appearance_properties_, SLOT(queueRender()), this);
  label_format_property_ = new StringProperty(
      "Format", "{id} {class} {class_prob}",
      "Placeholders: {id} {class} {class_prob} {exist} {sensor}.", label_property_, SLOT(queueRender()), this);
  font_size_property_ =
      new IntProperty("Font Size", 13, "Label font size [px].", label_property_, SLOT(queueRender()), this);
  font_size_property_->setMin(6);
  font_size_property_->setMax(72);

  color_mode_property_ =
      new EnumProperty("Color Mode", "By class", "Annotation color.", appearance_properties_, SLOT(queueRender()), this);
  color_mode_property_->addOption("By class", static_cast<int>(ColorMode::ByClass));
  color_mode_property_->addOption("By object ID", static_cast<int>(ColorMode::ByObjectId));
  color_mode_property_->addOption("By sensor", static_cast<int>(ColorMode::BySensor));
  color_mode_property_->addOption("Fixed", static_cast<int>(ColorMode::Fixed));
  fixed_color_property_ = new ColorProperty("Color", QColor(0, 255, 25),
      "Color to visualize objects if no specific class color is defined.",
      appearance_properties_, SLOT(queueRender()), this);

  color_property_group_ = new BoolProperty(
      "Classification coloring", true, "Use the object classification to set the color for supported visuals.",
      appearance_properties_, SLOT(queueRender()), this);
  using Class = perception_msgs::msg::ObjectClassification;
  for (const auto type : {Class::PEDESTRIAN, Class::BICYCLE, Class::MOTORCYCLE, Class::CAR, Class::UTILITY,
                          Class::BUS, Class::ANIMAL, Class::VRU, Class::MICRO, Class::UNKNOWN}) {
    const QString name = rendering::classificationName(type);
    classification_color_properties_[type] = new ColorProperty(
        name, rendering::defaultClassificationColor(type),
        "Color to visualize objects with classification " + name + ".",
        color_property_group_, SLOT(queueRender()), this);
  }

  image_alpha_property_ =
      new FloatProperty("Image Opacity", 1.0, "Opacity of the camera image; lower it to make annotations stand out.",
                        appearance_properties_, SLOT(queueRender()), this);
  image_alpha_property_->setMin(0.0);
  image_alpha_property_->setMax(1.0);
}

ImageObservationDisplay::~ImageObservationDisplay() { unsubscribeImage(); }

void ImageObservationDisplay::onInitialize() {
  RTDClass::onInitialize();
  image_topic_property_->initialize(context_->getRosNodeAbstraction());

  // RViz hosts the widget in a dock panel named after the display (dockable, floatable, saved in the config)
  // and shows/hides it together with the display.
  widget_ = std::make_unique<ImageObservationWidget>();
  widget_->setWindowTitle(getName());
  setAssociatedWidget(widget_.get());
  updateSourceVisibility();
}

void ImageObservationDisplay::reset() {
  RTDClass::reset();
  object_list_.reset();
  images_.clear();
  known_sensor_ids_.clear();
  sync_reference_stamp_.reset();
  sync_wait_started_.reset();
  sync_image_difference_ms_ = std::numeric_limits<double>::infinity();
  dirty_ = true;
}

void ImageObservationDisplay::onEnable() {
  RTDClass::onEnable();
  subscribeImage();
  dirty_ = true;
}

void ImageObservationDisplay::onDisable() {
  RTDClass::onDisable();
  unsubscribeImage();
}

void ImageObservationDisplay::processMessage(perception_msgs::msg::ObjectListWithObservation::ConstSharedPtr msg) {
  object_list_ = msg;
  if (!validStamp(msg->header.stamp))
    setStatus(StatusProperty::Warn, "Object timestamp", "Invalid object-list timestamp; using observation timestamps.");
  else
    deleteStatus("Object timestamp");

  std::set<uint64_t> sensor_ids;
  for (const auto& entry : msg->objects)
    if (!entry.observation.image.empty() && entry.object.state.sensor_id.size() == 1)
      sensor_ids.insert(entry.object.state.sensor_id.front());
  updateSensorOptions(sensor_ids);
  dirty_ = true;
}

void ImageObservationDisplay::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (!validStamp(msg->header.stamp)) {
    setStatus(StatusProperty::Warn, "Image timestamp", "Image with invalid timestamp ignored.");
    return;
  }
  deleteStatus("Image timestamp");
  const bool frame_changed = images_.empty() || images_.back()->header.frame_id != msg->header.frame_id;
  images_.push_back(msg);
  const auto capacity = static_cast<size_t>(image_buffer_size_property_->getInt());
  while (images_.size() > capacity) images_.pop_front();

  // Repaint synchronized views only when a late image can improve the current match.
  if (static_cast<SyncMode>(sync_mode_property_->getOptionInt()) == SyncMode::LatestImage || !object_list_ ||
      frame_changed || !sync_reference_stamp_ ||
      std::abs((rclcpp::Time(msg->header.stamp) - *sync_reference_stamp_).seconds()) * 1e3 < sync_image_difference_ms_)
    dirty_ = true;
}

void ImageObservationDisplay::update(float wall_dt, float ros_dt) {
  RTDClass::update(wall_dt, ros_dt);
  if (dirty_ && isEnabled()) dirty_ = !render();
}

void ImageObservationDisplay::queueRender() { dirty_ = true; }

void ImageObservationDisplay::updateSourceVisibility() {
  const bool topic = static_cast<ImageSource>(image_source_property_->getOptionInt()) == ImageSource::ImageTopic;
  image_topic_property_->setHidden(!topic);
  image_transport_property_->setHidden(!topic);
  sync_mode_property_->setHidden(!topic);
  image_buffer_size_property_->setHidden(!topic);
  updateImageSubscription();
}

void ImageObservationDisplay::updateImageSubscription() {
  unsubscribeImage();
  images_.clear();
  sync_wait_started_.reset();
  if (isEnabled()) subscribeImage();
  dirty_ = true;
}

void ImageObservationDisplay::subscribeImage() {
  if (!context_ || static_cast<ImageSource>(image_source_property_->getOptionInt()) != ImageSource::ImageTopic) {
    deleteStatus("Image");
    return;
  }
  if (image_topic_property_->isEmpty()) {
    setStatus(StatusProperty::Warn, "Image", "No image topic set; drawing on a blank canvas.");
    return;
  }
  try {
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    image_subscriber_ = image_transport::create_subscription(
        node.get(), image_topic_property_->getTopicStd(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { imageCallback(msg); },
        image_transport_property_->getStdString(), rmw_qos_profile_sensor_data);
    setStatus(StatusProperty::Ok, "Image", "Subscribed to " + image_topic_property_->getTopic());
  } catch (const std::exception& e) {
    setStatus(StatusProperty::Error, "Image", QString("Error subscribing: ") + e.what());
  }
}

void ImageObservationDisplay::unsubscribeImage() { image_subscriber_.shutdown(); }

void ImageObservationDisplay::updateSensorOptions(const std::set<uint64_t>& sensor_ids) {
  const size_t before = known_sensor_ids_.size();
  known_sensor_ids_.insert(sensor_ids.begin(), sensor_ids.end());
  if (known_sensor_ids_.size() == before) return;
  sensor_id_property_->clearOptions();
  sensor_id_property_->addOption(kAutoSensor);
  for (const auto id : known_sensor_ids_) sensor_id_property_->addOptionStd(std::to_string(id));
}

ImageObservationDisplay::Selection ImageObservationDisplay::selectObservations(
    const std::optional<std::string>& image_frame) const {
  Selection selection;
  if (!object_list_) return selection;

  std::vector<const perception_msgs::msg::ObjectWithObservation*> candidates;
  const double min_existence = min_existence_property_->getFloat();
  for (const auto& entry : object_list_->objects) {
    if (entry.object.existence_probability < min_existence || entry.observation.image.empty()) continue;
    try { object_access::imageView(entry); candidates.push_back(&entry); }
    catch (const std::exception&) { ++selection.invalid; }
  }
  const auto text = sensor_id_property_->getStdString();
  const bool automatic = text == kAutoSensor || text.empty();
  auto selected_sensor = automatic ? std::optional<uint64_t>{} : sensorId(text);
  if (!automatic && !selected_sensor) return selection;
  const bool match_frame = automatic && image_frame.has_value();
  if (automatic && !match_frame && !candidates.empty()) {
    selected_sensor = (*std::min_element(candidates.begin(), candidates.end(), [](const auto* a, const auto* b) {
      return a->object.state.sensor_id.front() < b->object.state.sensor_id.front();
    }))->object.state.sensor_id.front();
  }
  for (const auto* entry : candidates) {
    const auto image = object_access::imageView(*entry);
    if (match_frame ? image.header.frame_id != *image_frame : !selected_sensor || image.sensor_id != *selected_sensor) continue;
    rendering::ImageObservationAnnotation annotation;
    annotation.observation = entry;
    annotation.color = annotationColor(entry->object, *entry);
    annotation.label = formatLabel(entry->object, *entry);
    selection.annotations.push_back(annotation);
    const rclcpp::Time stamp(image.header.stamp);
    if (!selection.reference_stamp || stamp > *selection.reference_stamp) selection.reference_stamp = stamp;
    if (!selection.source_width) {
      selection.source_width = image.image_width;
      selection.source_height = image.image_height;
    }
  }
  return selection;
}

sensor_msgs::msg::Image::ConstSharedPtr ImageObservationDisplay::findImage(const rclcpp::Time& stamp,
                                                                       double& dt_ms) const {
  sensor_msgs::msg::Image::ConstSharedPtr best;
  dt_ms = std::numeric_limits<double>::infinity();
  for (const auto& image : images_) {
    if (!validStamp(image->header.stamp)) continue;
    const double dt = std::abs((rclcpp::Time(image->header.stamp) - stamp).seconds()) * 1e3;
    if (dt < dt_ms) {
      dt_ms = dt;
      best = image;
    }
  }
  return best;
}

QString ImageObservationDisplay::formatLabel(const perception_msgs::msg::Object& object,
                                          const perception_msgs::msg::ObjectWithObservation& observation) const {
  const auto classification = perception_msgs::object_access::getClassWithHighestProbability(object);
  QString text = label_format_property_->getString();
  text.replace("{id}", QString::number(object.id));
  text.replace("{class}", rendering::objectClassName(object.state));
  text.replace("{class_prob}", classification.probability >= 0.0 ? QString::number(classification.probability, 'f', 2) : QString());
  text.replace("{exist}", QString::number(object.existence_probability, 'f', 2));
  text.replace("{sensor}", QString::number(observation.object.state.sensor_id.front()));
  return text.simplified();
}

QColor ImageObservationDisplay::annotationColor(const perception_msgs::msg::Object& object,
                                             const perception_msgs::msg::ObjectWithObservation& observation) const {
  switch (static_cast<ColorMode>(color_mode_property_->getOptionInt())) {
    case ColorMode::ByObjectId: return rendering::colorForId(object.id);
    case ColorMode::BySensor: return rendering::colorForId(observation.object.state.sensor_id.front());
    case ColorMode::Fixed: return fixed_color_property_->getColor();
    case ColorMode::ByClass:
    default:
      if (!color_property_group_->getBool()) return fixed_color_property_->getColor();
      if (object_access::isTrafficSignClass(object_access::bestClass(object.state)) ||
          object_access::isTrafficLightClass(object_access::bestClass(object.state)))
        return rendering::defaultObjectColor(object.state);
      const auto type = perception_msgs::object_access::getClassWithHighestProbability(object).type;
      const auto color = classification_color_properties_.find(
          type == perception_msgs::msg::ObjectClassification::UNCLASSIFIED
              ? perception_msgs::msg::ObjectClassification::UNKNOWN : type);
      return color != classification_color_properties_.end() ? color->second->getColor()
                                                            : rendering::defaultClassificationColor(type);
  }
}

rendering::ImageObservationStyle ImageObservationDisplay::currentStyle() const {
  rendering::ImageObservationStyle style;
  style.draw_box = box_property_->getBool();
  style.box_line_width = box_line_width_property_->getFloat();
  style.box_fill_alpha = box_fill_alpha_property_->getFloat();
  style.draw_mask = mask_property_->getBool();
  style.mask_mode = static_cast<rendering::MaskMode>(mask_mode_property_->getOptionInt());
  style.mask_alpha = mask_alpha_property_->getFloat();
  style.draw_crop = crop_property_->getBool() ||
                    static_cast<ImageSource>(image_source_property_->getOptionInt()) == ImageSource::CropMosaic;
  style.draw_roi = roi_property_->getBool();
  style.draw_label = label_property_->getBool();
  style.font_pixel_size = font_size_property_->getInt();
  return style;
}

bool ImageObservationDisplay::render() {
  if (!widget_) return true;

  const auto source = static_cast<ImageSource>(image_source_property_->getOptionInt());
  const auto sync_mode = static_cast<SyncMode>(sync_mode_property_->getOptionInt());
  const double tolerance_ms = sync_tolerance_property_->getFloat();
  const bool use_images = source == ImageSource::ImageTopic && !images_.empty();

  // sensor selection; 'auto' follows the frame of the image stream
  std::optional<std::string> image_frame;
  if (use_images) image_frame = images_.back()->header.frame_id;
  Selection selection = selectObservations(image_frame);

  const std::string sensor_text = sensor_id_property_->getStdString();
  if (sensor_text != kAutoSensor && !sensor_text.empty() &&
      !sensorId(sensor_text))
    setStatus(StatusProperty::Error, "Filter", "Sensor ID must be 'auto' or an unsigned integer.");
  else
    deleteStatus("Filter");

  if (selection.invalid > 0)
    setStatus(StatusProperty::Warn, "Observation",
              QString("%1 drawn, %2 invalid observations skipped").arg(selection.annotations.size()).arg(selection.invalid));
  else if (object_list_)
    setStatus(StatusProperty::Ok, "Observation", QString("%1 drawn").arg(selection.annotations.size()));

  // reference time: newest selected evidence, otherwise the object list itself
  std::optional<rclcpp::Time> reference = selection.reference_stamp;
  if (!reference && object_list_ && validStamp(object_list_->header.stamp))
    reference = rclcpp::Time(object_list_->header.stamp);
  sync_reference_stamp_ = reference;
  sync_image_difference_ms_ = std::numeric_limits<double>::infinity();

  // choose the canvas image
  if (!use_images || sync_mode == SyncMode::LatestImage || !reference) sync_wait_started_.reset();
  sensor_msgs::msg::Image::ConstSharedPtr canvas_msg;
  if (use_images) {
    if (sync_mode == SyncMode::LatestImage || !reference) {
      canvas_msg = images_.back();
      deleteStatus("Synchronization");
    } else {
      double dt_ms = 0.0;
      canvas_msg = findImage(*reference, dt_ms);
      sync_image_difference_ms_ = dt_ms;
      if (dt_ms <= tolerance_ms) {
        sync_wait_started_.reset();
        setStatus(StatusProperty::Ok, "Synchronization", QString("Image matched (%1 ms)").arg(dt_ms, 0, 'f', 1));
      } else {
        // the matching image may still be in flight: wait a moment before falling back to the nearest one
        const bool newer_image_exists = rclcpp::Time(images_.back()->header.stamp) > *reference;
        const auto now = std::chrono::steady_clock::now();
        if (!sync_wait_started_) sync_wait_started_ = now;
        // New object lists must not restart the wait while the image stream remains behind.
        const double waited = std::chrono::duration<double>(now - *sync_wait_started_).count();
        if (!newer_image_exists && waited < kMaxSyncWaitSeconds) return false;
        setStatus(StatusProperty::Warn, "Synchronization",
                  QString("No image within tolerance, nearest is %1 ms off").arg(dt_ms, 0, 'f', 1));
      }
    }
  }

  QImage canvas_image;
  if (canvas_msg) {
    std::string error;
    canvas_image = rendering::toQImage(*canvas_msg, &error);
    if (canvas_image.isNull())
      setStatus(StatusProperty::Error, "Image", QString::fromStdString("Cannot display image: " + error));
    else if (!image_topic_property_->isEmpty())
      setStatus(StatusProperty::Ok, "Image", "Receiving " + image_topic_property_->getTopic());
  }

  // drawing one camera's evidence onto another camera's image is almost always a misconfiguration
  const auto foreign = canvas_msg ? std::find_if(selection.annotations.begin(), selection.annotations.end(),
                                                 [&](const auto& a) {
                                                   return a.observation->object.state.header.frame_id != canvas_msg->header.frame_id;
                                                 })
                                  : selection.annotations.end();
  if (foreign != selection.annotations.end())
    setStatus(StatusProperty::Warn, "Frame",
              QString::fromStdString("Observation in frame '" + foreign->observation->object.state.header.frame_id +
                                     "' drawn on image in frame '" + canvas_msg->header.frame_id + "'"));
  else
    deleteStatus("Frame");

  // stale = evidence not taken at the time of the shown canvas
  std::optional<rclcpp::Time> canvas_stamp = reference;
  if (canvas_msg) canvas_stamp = rclcpp::Time(canvas_msg->header.stamp);
  if (canvas_stamp) {
    for (auto& annotation : selection.annotations)
      annotation.stale =
          std::abs((rclcpp::Time(annotation.observation->object.state.header.stamp) - *canvas_stamp).seconds()) * 1e3 > tolerance_ms;
  }
  if (!show_stale_property_->getBool()) {
    selection.annotations.erase(std::remove_if(selection.annotations.begin(), selection.annotations.end(),
                                               [](const auto& a) { return a.stale; }),
                                selection.annotations.end());
  }

  // canvas geometry; keeps the last known size so the view does not jump when a frame has no evidence
  if (!canvas_image.isNull()) {
    last_source_size_ = canvas_image.size();
  } else if (selection.source_width > 0) {
    last_source_size_ = QSize(static_cast<int>(selection.source_width), static_cast<int>(selection.source_height));
  }

  ImageObservationFrame frame;
  frame.image = std::move(canvas_image);
  frame.image_opacity = image_alpha_property_->getFloat();
  frame.annotations = std::move(selection.annotations);
  frame.style = currentStyle();
  frame.keep_alive = object_list_;
  if (frame.image.isNull() && !object_list_) {
    frame.message = source == ImageSource::ImageTopic ? QStringLiteral("Waiting for image or object list")
                                                      : QStringLiteral("Waiting for object list");
  } else {
    // without any geometry yet (empty lists, no image), fall back to a common camera aspect ratio
    frame.source_size = last_source_size_.isValid() ? last_source_size_ : QSize(1280, 720);
  }
  widget_->setFrame(std::move(frame));
  return true;
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::ImageObservationDisplay, rviz_common::Display)
