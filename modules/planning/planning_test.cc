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

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/planner_factory.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

using TrajectoryPb = ADCTrajectory;

class PlanningTest: public ::testing::Test {
};

TEST_F(PlanningTest, ComputeTrajectory) {

  FLAGS_rtk_trajectory_filename = "modules/planning/testdata/garage.csv";
  Planning planning;
  common::vehicle_state::VehicleState vehicle_state;
  vehicle_state.set_x(586385.782841);
  vehicle_state.set_y(4140674.76065);

  vehicle_state.set_heading(2.836888814);
  vehicle_state.set_linear_velocity(0.15);
  vehicle_state.set_angular_velocity(0.0);

  std::vector<TrajectoryPoint> trajectory1;
  double time1 = 0.1;
  planning.Plan(vehicle_state, false, time1, &trajectory1);

  EXPECT_EQ(trajectory1.size(),
      (std::size_t)FLAGS_rtk_trajectory_forward);
  auto p1_start = trajectory1.front();
  auto p1_end = trajectory1.back();

  EXPECT_DOUBLE_EQ(p1_start.x, 586385.782841);
  EXPECT_DOUBLE_EQ(p1_end.x, 586355.063786);

  std::vector<TrajectoryPoint> trajectory2;
  double time2 = 0.5;
  planning.Plan(vehicle_state, true, time2, &trajectory2);

  EXPECT_EQ(trajectory2.size(),
      (std::size_t)FLAGS_rtk_trajectory_forward +
      (int)FLAGS_rtk_trajectory_backward);

  auto p2_backward = trajectory2.front();
  auto p2_start = trajectory2[(std::size_t)FLAGS_rtk_trajectory_backward];
  auto p2_end = trajectory2.back();

  EXPECT_DOUBLE_EQ(p2_backward.x, 586385.577255);
  EXPECT_DOUBLE_EQ(p2_start.x, 586385.486723);
  EXPECT_DOUBLE_EQ(p2_end.x, 586353.262913);

  double absolute_time1 = trajectory1[100].relative_time
      + time1;
  double absolute_time2 = trajectory2[
      60 + (std::size_t) FLAGS_rtk_trajectory_backward].relative_time
      + time2;

  EXPECT_NEAR(absolute_time1, absolute_time2, 0.001);
}

TEST_F(PlanningTest, ComputeTrajectoryNoRTKFile) {

  FLAGS_rtk_trajectory_filename = "";
  Planning planning;
  common::vehicle_state::VehicleState vehicle_state;
  vehicle_state.set_x(586385.782841);
  vehicle_state.set_y(4140674.76065);

  vehicle_state.set_heading(2.836888814);
  vehicle_state.set_linear_velocity(0.0);
  vehicle_state.set_angular_velocity(0.0);

  double time = 0.1;
  std::vector<TrajectoryPoint> trajectory;
  bool res_planning = planning.Plan(vehicle_state, false, time, &trajectory);
  EXPECT_FALSE(res_planning);

  // check Reset runs gracefully.
  planning.Reset();
}

TEST_F(PlanningTest, PlannerFactory) {
  auto ptr_planner = PlannerFactory::CreateInstance(PlannerType::OTHER);
  EXPECT_TRUE(ptr_planner == nullptr);
}

}  // namespace control
}  // namespace apollo
