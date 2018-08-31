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
#include <memory>
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

class AutotuningRawFeatureGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Referennce line info
    common::VehicleState ego_state;
    common::TrajectoryPoint ego_pos;
    ReferenceLine reference_line;
    hdmap::RouteSegments segments;
    ref_line_info_.reset(new ReferenceLineInfo(ego_state, ego_pos,
                         reference_line, segments));
    // pseudo empty frame info
    frame_.reset(new Frame(0, ego_pos, 0, ego_state, nullptr));
    std::vector<double> evaluate_time{1., 2. , 3., 4., 5., 6., 7., 8.};
    generator_.reset(new AutotuningRawFeatureGenerator(evaluate_time));
  }

  void TearDown() override {
    generator_.reset(nullptr);
    ref_line_info_.reset(nullptr);
    frame_.reset(nullptr);
  }

  std::unique_ptr<AutotuningRawFeatureGenerator> generator_ = nullptr;
  std::unique_ptr<ReferenceLineInfo> ref_line_info_ = nullptr;
  std::unique_ptr<Frame> frame_ = nullptr;
};

TEST_F(AutotuningRawFeatureGeneratorTest, generate_input_trajectory) {
  // init trajectory
  std::vector<common::TrajectoryPoint> trajectory;
  ASSERT_TRUE(generator_ != nullptr);
  auto result = generator_->evaluate_trajectory(trajectory,
                *ref_line_info_, *frame_, nullptr);
  EXPECT_TRUE(result == common::Status::OK());
}

TEST_F(AutotuningRawFeatureGeneratorTest, generate_input_trajectory_pointwise) {
  common::TrajectoryPoint trajectory_point;
  ASSERT_TRUE(generator_ != nullptr);
  auto result = generator_->evaluate_trajectory_point(trajectory_point,
                *ref_line_info_, *frame_, nullptr);
  EXPECT_TRUE(result == common::Status::OK());
}

}  // namespace planning
}  // namespace apollo
