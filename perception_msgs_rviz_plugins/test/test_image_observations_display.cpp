// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

#include "perception_msgs/displays/image_observations/image_observations_display.hpp"

namespace perception_msgs {
namespace displays {

class ImageObservationsDisplayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    display_ = std::make_unique<ImageObservationsDisplay>();
    display_->widget_ = std::make_unique<ImageObservationsWidget>();
    display_->widget_->resize(320, 240);
  }

  static msg::Object makeObject(const std::string& frame = "left", uint64_t sensor = 1) {
    msg::Object object;
    object.id = 42;
    object.existence_probability = 1.;
    object_access::initializeState(object, msg::HEXAMOTION::MODEL_ID);
    msg::ImageObservation image;
    image.header.frame_id = frame;
    image.header.stamp.sec = 10;
    image.sensor_id = sensor;
    image.image_width = 80;
    image.image_height = 60;
    image.bounding_box.size_x = 10.;
    image.bounding_box.size_y = 10.;
    object_access::addImageObservation(object, image);
    return object;
  }

  static sensor_msgs::msg::Image::SharedPtr makeImage(int32_t stamp = 10, const std::string& frame = "left") {
    auto image = std::make_shared<sensor_msgs::msg::Image>();
    image->header.frame_id = frame;
    image->header.stamp.sec = stamp;
    image->width = 80;
    image->height = 60;
    image->step = 240;
    image->encoding = "rgb8";
    image->data.assign(60 * 240, stamp < 0 ? 0 : static_cast<uint8_t>(stamp));
    return image;
  }

  void receiveObjects(const std::vector<msg::Object>& objects, int32_t stamp = 10) {
    auto list = std::make_shared<msg::ObjectList>();
    list->header.frame_id = "map";
    list->header.stamp.sec = stamp;
    list->objects = objects;
    display_->processMessage(list);
  }

  void receiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& image) { display_->imageCallback(image); }
  size_t bufferedImages() const { return display_->images_.size(); }
  bool dirty() const { return display_->dirty_; }
  int shownPixel() const {
    auto* widget = display_->widget_.get();
    return widget->grab().toImage().pixelColor(widget->width() / 2, widget->height() / 2).red();
  }
  void expireSyncWait() {
    display_->sync_wait_started_ = std::chrono::steady_clock::now() - std::chrono::seconds(2);
  }
  void setClassColor(uint8_t type, const QColor& color) {
    display_->classification_color_properties_.at(type)->setColor(color);
  }
  void setGeneralColor(const QColor& color) { display_->fixed_color_property_->setColor(color); }
  void enableClassColors(bool enabled) { display_->color_property_group_->setBool(enabled); }
  bool render() {
    const bool rendered = display_->render();
    display_->dirty_ = !rendered;
    return rendered;
  }

  auto select(const std::optional<std::string>& frame) const {
    return display_->selectObservations(frame).annotations;
  }

  void selectSensor(const std::string& sensor) { display_->sensor_id_property_->setValue(QString::fromStdString(sensor)); }
  QString label(const msg::Object& object) const {
    return display_->formatLabel(object, object.state.image_observations.front());
  }
  QColor color(const msg::Object& object) const {
    return display_->annotationColor(object, object.state.image_observations.front());
  }

  rclcpp::QoS qos() const { return display_->qos_profile; }
  QString reliability() const { return display_->topic_property_->subProp("Reliability Policy")->getValue().toString(); }
  void setReliability(const QString& policy) {
    display_->qos_profile_property_->initialize([this](rclcpp::QoS profile) { display_->qos_profile = profile; });
    display_->topic_property_->subProp("Reliability Policy")->setValue(policy);
  }

  std::unique_ptr<ImageObservationsDisplay> display_;
};

TEST_F(ImageObservationsDisplayTest, SensorDataQosIsDefaultAndRemainsConfigurable) {
  EXPECT_EQ(reliability(), "Best Effort");
  EXPECT_EQ(qos().get_rmw_qos_profile().reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_NE(rclcpp::qos_check_compatible(rclcpp::SensorDataQoS(), qos()).compatibility, rclcpp::QoSCompatibility::Error);
  setReliability("Reliable");
  EXPECT_EQ(qos().get_rmw_qos_profile().reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(rclcpp::qos_check_compatible(rclcpp::SensorDataQoS(), qos()).compatibility, rclcpp::QoSCompatibility::Error);
  setReliability("Best Effort");
  EXPECT_NE(rclcpp::qos_check_compatible(rclcpp::SensorDataQoS(), qos()).compatibility, rclcpp::QoSCompatibility::Error);
}

TEST_F(ImageObservationsDisplayTest, InvalidImageTimestampsAreSkippedAndRecoveryWorks) {
  receiveObjects({makeObject()});
  receiveImage(makeImage(-1));
  EXPECT_EQ(bufferedImages(), 0u);
  auto invalid = makeImage();
  invalid->header.stamp.nanosec = 1000000000u;
  receiveImage(invalid);
  EXPECT_EQ(bufferedImages(), 0u);
  EXPECT_NO_THROW(render());
  receiveImage(makeImage());
  EXPECT_EQ(bufferedImages(), 1u);
  EXPECT_TRUE(render());
  EXPECT_EQ(shownPixel(), 10);
}

TEST_F(ImageObservationsDisplayTest, InvalidObjectListTimestampDoesNotThrow) {
  receiveObjects({}, -1);
  receiveImage(makeImage());
  EXPECT_NO_THROW(render());
  EXPECT_EQ(shownPixel(), 10);
}

TEST_F(ImageObservationsDisplayTest, AutoSelectionNeverFallsBackToAnotherCameraImage) {
  receiveObjects({makeObject("left", 1), makeObject("right", 2)});
  auto matching = select(std::string("right"));
  ASSERT_EQ(matching.size(), 1u);
  EXPECT_EQ(matching.front().observation->sensor_id, 2u);
  EXPECT_TRUE(select(std::string("other")).empty());
  auto without_image = select(std::nullopt);
  ASSERT_EQ(without_image.size(), 1u);
  EXPECT_EQ(without_image.front().observation->sensor_id, 1u);
  selectSensor("2");
  auto explicit_sensor = select(std::string("left"));
  ASSERT_EQ(explicit_sensor.size(), 1u);
  EXPECT_EQ(explicit_sensor.front().observation->sensor_id, 2u);
}

TEST_F(ImageObservationsDisplayTest, InvalidSensorSelectionDoesNotChooseAnotherSensor) {
  receiveObjects({makeObject()});
  for (const auto* sensor : {"-1", "1invalid", "18446744073709551616"}) {
    selectSensor(sensor);
    EXPECT_TRUE(select(std::string("left")).empty());
    EXPECT_NO_THROW(render());
  }
}

TEST_F(ImageObservationsDisplayTest, ImageArrivingAfterObjectsSchedulesRendering) {
  receiveObjects({makeObject()});
  EXPECT_TRUE(render());
  EXPECT_FALSE(dirty());
  EXPECT_EQ(shownPixel(), 30);
  receiveImage(makeImage());
  EXPECT_TRUE(dirty());
  EXPECT_TRUE(render());
  EXPECT_EQ(shownPixel(), 10);
}

TEST_F(ImageObservationsDisplayTest, NewerUnrelatedImageDoesNotRepaintSynchronizedMasks) {
  receiveObjects({makeObject()});
  receiveImage(makeImage(10));
  EXPECT_TRUE(render());
  EXPECT_FALSE(dirty());
  receiveImage(makeImage(11));
  EXPECT_FALSE(dirty());
  EXPECT_EQ(shownPixel(), 10);
}

TEST_F(ImageObservationsDisplayTest, LateMatchingImageReplacesNearestFallback) {
  receiveObjects({makeObject()});
  receiveImage(makeImage(9));
  expireSyncWait();
  EXPECT_TRUE(render());
  EXPECT_EQ(shownPixel(), 9);
  EXPECT_FALSE(dirty());
  receiveImage(makeImage(10));
  EXPECT_TRUE(dirty());
  EXPECT_TRUE(render());
  EXPECT_EQ(shownPixel(), 10);
}

TEST_F(ImageObservationsDisplayTest, ContinuousObjectListsDoNotRestartSyncWait) {
  receiveObjects({makeObject()});
  receiveImage(makeImage(9));
  EXPECT_FALSE(render());
  expireSyncWait();
  for (int stamp = 11; stamp < 15; ++stamp) {
    auto object = makeObject();
    object.state.image_observations.front().header.stamp.sec = stamp;
    receiveObjects({object}, stamp);
    receiveImage(makeImage(stamp - 1));
    EXPECT_TRUE(render());
    EXPECT_EQ(shownPixel(), stamp - 1);
  }
  receiveImage(makeImage(14));
  EXPECT_TRUE(render());
  EXPECT_EQ(shownPixel(), 14);

  auto object = makeObject();
  object.state.image_observations.front().header.stamp.sec = 15;
  receiveObjects({object}, 15);
  EXPECT_FALSE(render());
}

TEST(ImageObservationRendering, Mono16PreservesPixelValuesForBothByteOrders) {
  sensor_msgs::msg::Image image;
  image.width = 2;
  image.height = 2;
  image.step = 6;
  image.encoding = "mono16";
  for (const bool big_endian : {false, true}) {
    image.is_bigendian = big_endian;
    image.data = big_endian ? std::vector<uint8_t>{0x12, 0x34, 0xab, 0xcd, 0, 0,
                                                  0x56, 0x78, 0xef, 0x01, 0, 0}
                           : std::vector<uint8_t>{0x34, 0x12, 0xcd, 0xab, 0, 0,
                                                  0x78, 0x56, 0x01, 0xef, 0, 0};
    const auto converted = rendering::toQImage(image);
    ASSERT_FALSE(converted.isNull());
    EXPECT_EQ(reinterpret_cast<const uint16_t*>(converted.constScanLine(0))[0], 0x1234);
    EXPECT_EQ(reinterpret_cast<const uint16_t*>(converted.constScanLine(0))[1], 0xabcd);
    EXPECT_EQ(reinterpret_cast<const uint16_t*>(converted.constScanLine(1))[0], 0x5678);
    EXPECT_EQ(reinterpret_cast<const uint16_t*>(converted.constScanLine(1))[1], 0xef01);
  }
}

TEST(ImageObservationRendering, ClassLabelsRetainExistingNamesAndDeprecationInformation) {
  using Class = msg::ObjectClassification;
  EXPECT_EQ(rendering::classificationName(Class::CAR), "CAR");
  EXPECT_EQ(rendering::classificationName(Class::VAN), "VAN (DEPRECATED)");
  EXPECT_EQ(rendering::classificationName(Class::CAR_UNION), "CAR_UNION (DEPRECATED)");
  EXPECT_EQ(rendering::classificationName(Class::TRUCK_UNION), "TRUCK_UNION (DEPRECATED)");
  EXPECT_EQ(rendering::classificationName(Class::BIKE_UNION), "BIKE_UNION (DEPRECATED)");
  EXPECT_EQ(rendering::classificationName(255), "TYPE NOT IMPLEMENTED");
}

TEST_F(ImageObservationsDisplayTest, ClassificationColorsAreConfigurableWithExistingDefaults) {
  auto object = makeObject();
  msg::ObjectClassification classification;
  classification.type = msg::ObjectClassification::CAR;
  classification.probability = 1.;
  object.state.classifications = {classification};
  setClassColor(classification.type, QColor(12, 34, 56));
  EXPECT_EQ(color(object), QColor(12, 34, 56));
  setGeneralColor(QColor(78, 90, 12));
  enableClassColors(false);
  EXPECT_EQ(color(object), QColor(78, 90, 12));
  enableClassColors(true);
  object.state.classifications.front().type = msg::ObjectClassification::VAN;
  EXPECT_EQ(color(object), QColor(128, 128, 128));
  object.state.classifications.front().type = msg::ObjectClassification::UNCLASSIFIED;
  setClassColor(msg::ObjectClassification::UNKNOWN, QColor(34, 56, 78));
  EXPECT_EQ(color(object), QColor(34, 56, 78));
}

TEST_F(ImageObservationsDisplayTest, LabelsAndColorsUseSignAndLightModels) {
  auto sign = makeObject();
  object_access::initializeState(sign, msg::TRAFFICSIGN::MODEL_ID);
  object_access::setTrafficSignType(sign, msg::TRAFFICSIGN::TYPE_STOP);
  EXPECT_TRUE(label(sign).contains("traffic sign: STOP"));
  EXPECT_FALSE(label(sign).contains("unknown"));
  auto light = makeObject();
  object_access::initializeState(light, msg::TRAFFICLIGHT::MODEL_ID);
  object_access::setTrafficLightState(light, msg::TRAFFICLIGHT::STATE_RED);
  object_access::setTrafficLightType(light, msg::TRAFFICLIGHT::TYPE_LEFT);
  EXPECT_TRUE(label(light).contains("traffic light red left"));
  EXPECT_NE(color(sign), color(light));
  EXPECT_NE(color(sign), rendering::defaultClassificationColor(msg::ObjectClassification::UNKNOWN));
  EXPECT_NE(color(light), rendering::defaultClassificationColor(msg::ObjectClassification::UNKNOWN));
  sign.state.discrete_state.clear();
  light.state.discrete_state.clear();
  EXPECT_TRUE(label(sign).contains("traffic sign"));
  EXPECT_TRUE(label(light).contains("traffic light"));
}

TEST_F(ImageObservationsDisplayTest, RoadUserClassLabelsAndProbabilitiesArePreserved) {
  auto car = makeObject();
  msg::ObjectClassification classification;
  classification.type = msg::ObjectClassification::CAR;
  classification.probability = .9;
  car.state.classifications = {classification};
  EXPECT_EQ(label(car), "42 CAR 0.90");
  EXPECT_EQ(color(car), rendering::defaultClassificationColor(msg::ObjectClassification::CAR));
}

}  // namespace displays
}  // namespace perception_msgs

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
