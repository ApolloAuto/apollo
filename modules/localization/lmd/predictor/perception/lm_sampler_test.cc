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

#include "modules/localization/lmd/predictor/perception/lm_sampler.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {

using apollo::perception::LaneMarker;
using apollo::perception::LaneMarkers;

class LMSampleTest : public ::testing::Test {};

TEST_F(LMSampleTest, SamplingForOneLaneMarker) {
  LaneMarker lane_marker;
  lane_marker.set_c0_position(1.0);
  lane_marker.set_c1_heading_angle(1.0);
  lane_marker.set_c2_curvature(1.0);
  lane_marker.set_c3_curvature_derivative(1.0);

  LaneMarkers lane_markers;
  lane_markers.mutable_left_lane_marker()->CopyFrom(lane_marker);

  LMSampler lm_sampler;
  std::vector<PCSourcePoint> pc_sourcepoint = lm_sampler.Sampling(lane_markers);

  double d_error_permit = 0.001;
  EXPECT_NEAR(0.0, pc_sourcepoint[0].position.x(), d_error_permit);
  EXPECT_NEAR(1.0, pc_sourcepoint[0].position.y(), d_error_permit);
  EXPECT_NEAR(0.0, pc_sourcepoint[0].position.z(), d_error_permit);

  EXPECT_NEAR(1.0, pc_sourcepoint[0].direction.x(), d_error_permit);
  EXPECT_NEAR(3.0, pc_sourcepoint[0].direction.y(), d_error_permit);
  EXPECT_NEAR(0.0, pc_sourcepoint[0].direction.z(), d_error_permit);

  EXPECT_NEAR(0.0633, pc_sourcepoint[0].curvature, d_error_permit);

  EXPECT_NEAR(0.5, pc_sourcepoint[5].position.x(), d_error_permit);
  EXPECT_NEAR(1.875, pc_sourcepoint[5].position.y(), d_error_permit);
  EXPECT_NEAR(0.0, pc_sourcepoint[5].position.z(), d_error_permit);

  EXPECT_NEAR(1.0, pc_sourcepoint[5].direction.x(), d_error_permit);
  EXPECT_NEAR(3.75, pc_sourcepoint[5].direction.y(), d_error_permit);
  EXPECT_NEAR(0.0, pc_sourcepoint[5].direction.z(), d_error_permit);

  EXPECT_NEAR(0.085531, pc_sourcepoint[5].curvature, d_error_permit);
}

}  // namespace localization
}  // namespace apollo
