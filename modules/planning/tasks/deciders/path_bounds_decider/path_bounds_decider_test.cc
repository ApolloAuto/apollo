/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

#include "gtest/gtest.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class PathBoundsDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    config_.set_task_type(TaskConfig::PATH_BOUNDS_DECIDER);
    config_.mutable_path_bounds_decider_config();
    config_.mutable_path_bounds_decider_config()->set_is_lane_borrowing(false);
  }

  virtual void TearDown() {}

 protected:
  TaskConfig config_;
};

TEST_F(PathBoundsDeciderTest, Init) {
  PathBoundsDecider path_bounds_decider(config_);
  EXPECT_EQ(path_bounds_decider.Name(),
            TaskConfig::TaskType_Name(config_.task_type()));
}

TEST_F(PathBoundsDeciderTest, InitPathBoundary) {
  PathBoundsDecider path_bounds_decider(config_);
  path_bounds_decider.adc_frenet_s_ = 10.0;

  // TODO(all): implement this unit test.

  // ReferenceLine reference_line;
  // reference_line.map_path_.length_ = 200.0;

  // std::vector<std::tuple<double, double, double>> path_boundary;
}

TEST_F(PathBoundsDeciderTest, GetBoundaryFromLanesAndADC) {
  PathBoundsDecider path_bounds_decider(config_);
  path_bounds_decider.adc_frenet_s_ = 10.0;
  // TODO(all): implement this unit test.
}

TEST_F(PathBoundsDeciderTest, GetBoundaryFromStaticObstacles) {
  PathBoundsDecider path_bounds_decider(config_);
  std::vector<std::tuple<double, double, double>> path_bound;
  PathDecision path_decision;
}

}  // namespace planning
}  // namespace apollo
