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

#include "modules/planning/planner/rtk/rtk_replay_planner.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/planning/common/planning_gflags.h"

using apollo::common::TrajectoryPoint;

namespace apollo {
namespace planning {

TEST(RTKReplayPlannerTest, ComputeTrajectory) {
  FLAGS_rtk_trajectory_filename = "modules/planning/testdata/garage.csv";
  FLAGS_enable_map_reference_unify = false;
  RTKReplayPlanner planner;

  TrajectoryPoint start_point;
  common::PointENU point;
  point.set_x(586385.782842);
  point.set_y(4140674.76063);
  start_point.mutable_path_point()->set_x(586385.782842);
  start_point.mutable_path_point()->set_y(4140674.76063);

  ReferenceLine ref;
  hdmap::RouteSegments segments;
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  localization.mutable_pose()->mutable_position()->set_x(586385.782842);
  localization.mutable_pose()->mutable_position()->set_y(4140674.76063);
  localization.mutable_pose()->mutable_angular_velocity()->set_x(0.0);
  localization.mutable_pose()->mutable_angular_velocity()->set_y(0.0);
  localization.mutable_pose()->mutable_angular_velocity()->set_z(0.0);
  localization.mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
  localization.mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
  localization.mutable_pose()->mutable_linear_acceleration()->set_z(0.0);
  common::VehicleStateProvider::instance()->Update(localization, chassis);
  common::VehicleState state;
  state.set_x(point.x());
  state.set_y(point.y());
  state.set_z(point.z());
  ReferenceLineInfo info(state, start_point, ref, segments);
  auto status = planner.PlanOnReferenceLine(start_point, nullptr, &info);

  const auto& trajectory = info.trajectory();
  EXPECT_TRUE(status.ok());
  EXPECT_TRUE(!trajectory.trajectory_points().empty());
  EXPECT_EQ(trajectory.trajectory_points().size(),
            FLAGS_rtk_trajectory_forward);

  auto first_point = trajectory.trajectory_points().begin();
  EXPECT_DOUBLE_EQ(first_point->path_point().x(), 586385.782841);
  EXPECT_DOUBLE_EQ(first_point->path_point().y(), 4140674.76065);

  auto last_point = trajectory.trajectory_points().rbegin();
  EXPECT_DOUBLE_EQ(last_point->path_point().x(), 586355.063786);
  EXPECT_DOUBLE_EQ(last_point->path_point().y(), 4140681.98605);
}

TEST(RTKReplayPlannerTest, ErrorTest) {
  FLAGS_rtk_trajectory_filename =
      "modules/planning/testdata/garage_no_file.csv";
  FLAGS_enable_map_reference_unify = false;
  RTKReplayPlanner planner;
  FLAGS_rtk_trajectory_filename = "modules/planning/testdata/garage_error.csv";
  RTKReplayPlanner planner_with_error_csv;
  TrajectoryPoint start_point;
  start_point.mutable_path_point()->set_x(586385.782842);
  start_point.mutable_path_point()->set_y(4140674.76063);
  common::PointENU point;
  point.set_x(586385.782842);
  point.set_y(4140674.76063);
  localization::LocalizationEstimate localization;
  canbus::Chassis chassis;
  localization.mutable_pose()->mutable_position()->set_x(586385.782842);
  localization.mutable_pose()->mutable_position()->set_y(4140674.76063);
  localization.mutable_pose()->mutable_angular_velocity()->set_x(0.0);
  localization.mutable_pose()->mutable_angular_velocity()->set_y(0.0);
  localization.mutable_pose()->mutable_angular_velocity()->set_z(0.0);
  localization.mutable_pose()->mutable_linear_acceleration()->set_x(0.0);
  localization.mutable_pose()->mutable_linear_acceleration()->set_y(0.0);
  localization.mutable_pose()->mutable_linear_acceleration()->set_z(0.0);
  common::VehicleStateProvider::instance()->Update(localization, chassis);
  ReferenceLine ref;
  hdmap::RouteSegments segments;
  common::VehicleState state;
  state.set_x(point.x());
  state.set_y(point.y());
  state.set_z(point.z());
  ReferenceLineInfo info(state, start_point, ref, segments);
  EXPECT_TRUE(
      !(planner_with_error_csv.PlanOnReferenceLine(start_point, nullptr, &info))
           .ok());
}

}  // namespace planning
}  // namespace apollo
