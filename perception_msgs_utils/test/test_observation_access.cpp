// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <perception_msgs_utils/object_access.hpp>

using namespace perception_msgs;
using namespace perception_msgs::object_access;

ObjectWithObservation imageObject() {
  ObjectWithObservation entry;
  entry.object.id = (uint64_t{1} << 40) + 7;
  entry.object.existence_probability = 0.75;
  initializeState(entry.object, BOX2D::MODEL_ID);
  auto &state = entry.object.state;
  state.header.frame_id = "camera_optical";
  state.sensor_id = {11};
  state.continuous_state = {320, 200, 60, 180, 0};
  state.discrete_state = {640, 480, 290, 110, 60, 180};
  entry.observation.image.emplace_back();
  auto &crop = entry.observation.image.front().crop;
  crop.header = state.header;
  crop.width = 30;
  crop.height = 90;
  crop.step = 90;
  crop.encoding = "bgr8";
  crop.data.assign(8100, 7);
  return entry;
}

TEST(Observation, ImageViewRetainsSourceGeometryWithoutCopyingPixels) {
  auto input = imageObject();
  const auto view = imageView(input);
  EXPECT_EQ(view.image_width, 640u);
  EXPECT_EQ(view.roi.width, 60u);
  EXPECT_EQ(view.crop.width, 30u);
  EXPECT_EQ(&view.crop, &input.observation.image.front().crop);
  EXPECT_EQ(view.header.frame_id, "camera_optical");
}
TEST(Observation, RejectsDimensionalMismatchAndMalformedStorage) {
  auto input = imageObject();
  input.object.state.model_id = HEXAMOTION::MODEL_ID;
  EXPECT_THROW(sanityCheckObjectObservation(input), std::invalid_argument);
  input = imageObject();
  input.observation.image.front().crop.step = 89;
  EXPECT_THROW(sanityCheckObjectObservation(input), std::invalid_argument);
  input = imageObject();
  input.observation.point_cloud.emplace_back();
  EXPECT_THROW(sanityCheckObjectObservation(input), std::invalid_argument);
  input = imageObject();
  input.object.state.sensor_id.push_back(12);
  EXPECT_THROW(sanityCheckObjectObservation(input), std::invalid_argument);
  input = imageObject();
}
TEST(Observation, CloudRequiresMetricGeometryAndValidFieldLayout) {
  auto input = imageObject();
  input.observation.image.clear();
  input.observation.point_cloud.emplace_back();
  input.observation.point_cloud.front().points.header =
      input.object.state.header;
  EXPECT_THROW(sanityCheckObjectObservation(input), std::invalid_argument);
  initializeState(input.object, HEXAMOTION::MODEL_ID,
                  CONTINUOUS_STATE_UNMEASURED);
  EXPECT_NO_THROW(sanityCheckObjectObservation(input));
  auto &cloud = input.observation.point_cloud.front().points;
  cloud.width = 1;
  cloud.height = 1;
  cloud.point_step = 12;
  cloud.row_step = 12;
  cloud.data.resize(12);
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.datatype = 7;
  field.count = 1;
  field.offset = 10;
  cloud.fields.push_back(field);
  EXPECT_THROW(sanityCheckObjectObservation(input), std::invalid_argument);
}
