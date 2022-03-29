/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file: testing cases for autotuning raw feature generator
 **/
#include "modules/planning/tuning/autotuning_raw_feature_generator.h"

#include "gtest/gtest.h"
#include "modules/planning/common/local_view.h"

namespace apollo {
namespace planning {

class AutotuningRawFeatureGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Reference line info
    common::VehicleState ego_state;
    common::TrajectoryPoint ego_pos;
    ReferenceLine reference_line;
    hdmap::RouteSegments segments;
    ref_line_info_.reset(
        new ReferenceLineInfo(ego_state, ego_pos, reference_line, segments));
    // pseudo empty frame info
    LocalView dummy_local_view;
    frame_.reset(new Frame(0, dummy_local_view, ego_pos, ego_state, nullptr));
    speed_limit_.reset(new SpeedLimit());
    generator_.reset(new AutotuningRawFeatureGenerator(8, 17, *ref_line_info_,
                                                       *frame_, *speed_limit_));
  }

  void TearDown() override {
    generator_.reset(nullptr);
    ref_line_info_.reset(nullptr);
    frame_.reset(nullptr);
    speed_limit_.reset(nullptr);
  }

  std::unique_ptr<AutotuningRawFeatureGenerator> generator_ = nullptr;
  std::unique_ptr<ReferenceLineInfo> ref_line_info_ = nullptr;
  std::unique_ptr<Frame> frame_ = nullptr;
  std::unique_ptr<SpeedLimit> speed_limit_ = nullptr;
};

TEST_F(AutotuningRawFeatureGeneratorTest, generate_input_trajectory) {
  // init trajectory
  std::vector<common::TrajectoryPoint> trajectory;
  ASSERT_TRUE(generator_ != nullptr);
  auto result = generator_->EvaluateTrajectory(trajectory, nullptr);
  EXPECT_TRUE(result.ok());
}

TEST_F(AutotuningRawFeatureGeneratorTest, generate_input_trajectory_pointwise) {
  common::TrajectoryPoint trajectory_point;
  ASSERT_TRUE(generator_ != nullptr);
  auto result = generator_->EvaluateTrajectoryPoint(trajectory_point, nullptr);
  EXPECT_TRUE(result.ok());
}

}  // namespace planning
}  // namespace apollo
