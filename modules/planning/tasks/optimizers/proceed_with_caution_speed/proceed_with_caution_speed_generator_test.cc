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
 * @file
 **/

#include "gtest/gtest.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/file.h"
// #include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_context.h"

#define private public
#include "modules/planning/tasks/optimizers/proceed_with_caution_speed/proceed_with_caution_speed_generator.h"

namespace apollo {
namespace planning {

class ProceedWithCautionSpeedGeneratorTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

  virtual void TearDown() {}

 protected:
  DpStSpeedConfig dp_config_;

  SLBoundary adc_sl_boundary_;
  PathData path_data_;
  common::TrajectoryPoint init_point_;
  PathDecision path_decision_;
};

TEST_F(ProceedWithCautionSpeedGeneratorTest, FixedSpeed) {
  // Set up testing for ProceedWithCautionSpeedGenerator::Init
  ProceedWithCautionSpeedConfig proceed_with_caution_speed_config;
  proceed_with_caution_speed_config.set_max_distance(20.0);
  TaskConfig scenario_task_config;
  scenario_task_config.mutable_proceed_with_caution_speed_config()->CopyFrom(
      proceed_with_caution_speed_config);
  ProceedWithCautionSpeedGenerator test_obj(scenario_task_config);

  PlanningContext::GetScenarioInfo()
      ->proceed_with_caution_speed.is_fixed_distance = false;

  // Set up testing for ProceedWithCautionSpeedGenerator::Process
  std::vector<common::PathPoint> path_points;
  common::PathPoint pointA;
  pointA.set_x(0.0);
  pointA.set_y(0.0);
  pointA.set_z(0.0);
  pointA.set_s(0.0);
  common::PathPoint pointB;
  pointB.set_x(10.0);
  pointB.set_y(0.0);
  pointB.set_z(0.0);
  pointB.set_s(10.0);
  path_points.emplace_back(pointA);
  path_points.emplace_back(pointB);
  DiscretizedPath path(path_points);
  double path_len = path.Length();
  EXPECT_EQ(path_len, 10.0);

  ReferenceLine reference_line_;
  path_data_.SetReferenceLine(&reference_line_);
  path_data_.SetDiscretizedPath(path);
  size_t num_of_points = path_data_.discretized_path().size();
  EXPECT_EQ(num_of_points, 2);

  SpeedData speed_data1;
  SpeedData speed_data2;
  common::Status process_status = test_obj.Process(
      adc_sl_boundary_, path_data_, init_point_, reference_line_, speed_data1,
      &path_decision_, &speed_data2);
  EXPECT_EQ(process_status, common::Status::OK());

  int num_speed_points = static_cast<int>(speed_data2.size());
  EXPECT_GT(num_speed_points, 0);
  AERROR << "Number of speed points: " << num_speed_points;

  // Check if the output speed are the same for all points
  for (int i = 0; i < num_speed_points; i++) {
    double actual_speed = speed_data2[i].v();
    EXPECT_DOUBLE_EQ(actual_speed, 2.23);
  }
}

TEST_F(ProceedWithCautionSpeedGeneratorTest, FixedDistance) {
  // Set up testing for ProceedWithCautionSpeedGenerator::Init
  ProceedWithCautionSpeedConfig proceed_with_caution_speed_config;
  proceed_with_caution_speed_config.set_max_distance(20.0);
  TaskConfig scenario_task_config;
  scenario_task_config.mutable_proceed_with_caution_speed_config()->CopyFrom(
      proceed_with_caution_speed_config);
  ProceedWithCautionSpeedGenerator test_obj(scenario_task_config);

  PlanningContext::GetScenarioInfo()
      ->proceed_with_caution_speed.is_fixed_distance = true;
  PlanningContext::GetScenarioInfo()->proceed_with_caution_speed.distance = 5.0;

  // Set up testing for ProceedWithCautionSpeedGenerator::Process
  std::vector<common::PathPoint> path_points;
  common::PathPoint pointA;
  pointA.set_x(0.0);
  pointA.set_y(0.0);
  pointA.set_z(0.0);
  pointA.set_s(0.0);
  common::PathPoint pointB;
  pointB.set_x(10.0);
  pointB.set_y(0.0);
  pointB.set_z(0.0);
  pointB.set_s(10.0);
  path_points.emplace_back(pointA);
  path_points.emplace_back(pointB);
  DiscretizedPath path(path_points);
  double path_len = path.Length();
  EXPECT_EQ(path_len, 10.0);

  ReferenceLine reference_line_;
  path_data_.SetReferenceLine(&reference_line_);
  path_data_.SetDiscretizedPath(path);
  size_t num_of_points = path_data_.discretized_path().size();
  EXPECT_EQ(num_of_points, 2);

  SpeedData speed_data1;
  SpeedData speed_data2;
  common::Status process_status = test_obj.Process(
      adc_sl_boundary_, path_data_, init_point_, reference_line_, speed_data1,
      &path_decision_, &speed_data2);
  EXPECT_EQ(process_status, common::Status::OK());

  int num_speed_points = static_cast<int>(speed_data2.size());
  EXPECT_GT(num_speed_points, 0);
  AERROR << "Number of speed points: " << num_speed_points;

  // Check if the output speed are the same for all points
  for (int i = 0; i < num_speed_points; i++) {
    double actual_speed = speed_data2[i].v();
    EXPECT_DOUBLE_EQ(actual_speed, 2.23);
  }
}

}  // namespace planning
}  // namespace apollo
