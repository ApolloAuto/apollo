/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/localization/lmd/predictor/perception/pc_registrator.h"

#include "gtest/gtest.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/math/math_utils.h"
#include "modules/localization/lmd/predictor/perception/lm_provider.h"
#include "modules/localization/lmd/predictor/perception/lm_sampler.h"
#include "modules/localization/lmd/predictor/perception/pc_map.h"

namespace apollo {
namespace localization {

using apollo::common::Point3D;
using apollo::common::PointENU;
using apollo::perception::LaneMarkers;

class PCRegistratorTest : public ::testing::Test {
 public:
  PCRegistratorTest() : map_(&provider_), registrator_(&map_) {}

 protected:
  LMProvider provider_;
  PCMap map_;
  LMSampler sampler_;
  PCRegistrator registrator_;
};

TEST_F(PCRegistratorTest, ComputeError) {
  LaneMarkers lane_markers;
  lane_markers.mutable_left_lane_marker()->set_c0_position(1.3);
  lane_markers.mutable_left_lane_marker()->set_c1_heading_angle(0.0);
  lane_markers.mutable_left_lane_marker()->set_c2_curvature(0.0);
  lane_markers.mutable_left_lane_marker()->set_c3_curvature_derivative(0.0);
  lane_markers.mutable_right_lane_marker()->set_c0_position(-2.3);
  lane_markers.mutable_right_lane_marker()->set_c1_heading_angle(0.0);
  lane_markers.mutable_right_lane_marker()->set_c2_curvature(0.0);
  lane_markers.mutable_right_lane_marker()->set_c3_curvature_derivative(0.0);
  auto source_points = sampler_.Sampling(lane_markers);

  PointENU position;
  position.set_x(683063.0);
  position.set_y(3110730.0);
  position.set_z(57.0);
  double heading = 2.5;
  map_.UpdateRange(position, 16.0);
  auto error0 = registrator_.ComputeError(source_points, position, heading);

  position.set_x(683063.0);
  position.set_y(3110730.0);
  position.set_z(57.0);
  heading = 2.3;
  map_.UpdateRange(position, 16.0);
  auto error1 = registrator_.ComputeError(source_points, position, heading);

  EXPECT_GT(error0, error1);
}

TEST_F(PCRegistratorTest, Register) {
  LaneMarkers lane_markers;
  lane_markers.mutable_left_lane_marker()->set_c0_position(1.3);
  lane_markers.mutable_left_lane_marker()->set_c1_heading_angle(0.0);
  lane_markers.mutable_left_lane_marker()->set_c2_curvature(0.0);
  lane_markers.mutable_left_lane_marker()->set_c3_curvature_derivative(0.0);
  lane_markers.mutable_right_lane_marker()->set_c0_position(-2.3);
  lane_markers.mutable_right_lane_marker()->set_c1_heading_angle(0.0);
  lane_markers.mutable_right_lane_marker()->set_c2_curvature(0.0);
  lane_markers.mutable_right_lane_marker()->set_c3_curvature_derivative(0.0);
  auto source_points = sampler_.Sampling(lane_markers);

  PointENU position;
  position.set_x(683063.0);
  position.set_y(3110730.5);
  position.set_z(57.0);
  map_.UpdateRange(position, 16.0);
  double heading = 2.0;

  PointENU position_processed;
  double heading_processed;
  registrator_.Register(source_points, position, heading, &position_processed,
                        &heading_processed);

  EXPECT_NEAR(2.5, heading_processed, 0.1);
}

}  // namespace localization
}  // namespace apollo
