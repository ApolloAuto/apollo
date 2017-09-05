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

#include "modules/planning/planning.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

class PlanningTest : public ::testing::Test {
  virtual void SetUp() {
    FLAGS_planning_config_file =
        "modules/planning/testdata/conf/planning_config.pb.txt";

    FLAGS_adapter_config_path = "modules/planning/testdata/conf/adapter.conf";

    FLAGS_enable_record_debug = false;
  }
};

TEST_F(PlanningTest, ComputeTrajectory) {
  FLAGS_rtk_trajectory_filename = "modules/planning/testdata/garage.csv";
  Planning planning;
  common::VehicleState::instance()->set_x(586385.782841);
  common::VehicleState::instance()->set_y(4140674.76065);

  common::VehicleState::instance()->set_heading(2.836888814);
  common::VehicleState::instance()->set_linear_velocity(0.15);
  common::VehicleState::instance()->set_angular_velocity(0.0);

  ADCTrajectory trajectory1;
  planning.Init();
  common::TrajectoryPoint init_adc_point;
  planning.InitFrame(1, 0.0, init_adc_point);
  /**
  double time1 = 0.1;
  planning.Plan(false, time1, &trajectory1);

  EXPECT_EQ(trajectory1.trajectory_point_size(), FLAGS_rtk_trajectory_forward);
  const auto& p1_start = trajectory1.trajectory_point().begin();
  const auto& p1_end = trajectory1.trajectory_point().rbegin();

  EXPECT_DOUBLE_EQ(p1_start->path_point().x(), 586385.782841);
  EXPECT_DOUBLE_EQ(p1_end->path_point().x(), 586355.063786);
  **/

  /**
  ADCTrajectory trajectory2;
  double time2 = 0.5;
  planning.Plan(true, time2, &trajectory2);

  EXPECT_EQ(trajectory2.trajectory_point_size(),
            FLAGS_rtk_trajectory_forward + FLAGS_backward_trajectory_point_num);

  const auto& p2_backward = trajectory2.trajectory_point().begin();
  const auto& p2_start =
      trajectory2.trajectory_point(FLAGS_backward_trajectory_point_num);
  const auto& p2_end = trajectory2.trajectory_point().rbegin();

  EXPECT_DOUBLE_EQ(p2_backward->path_point().x(), 586385.577255);
  EXPECT_DOUBLE_EQ(p2_start.path_point().x(), 586385.486723);
  EXPECT_DOUBLE_EQ(p2_end->path_point().x(), 586353.262913);

  double absolute_time1 =
      trajectory1.trajectory_point(100).relative_time() + time1;
  double absolute_time2 =
      trajectory2.trajectory_point(60 + FLAGS_backward_trajectory_point_num)
          .relative_time() +
      time2;

  EXPECT_NEAR(absolute_time1, absolute_time2, 0.001);
  **/
}

TEST_F(PlanningTest, ComputeTrajectoryNoRTKFile) {
  FLAGS_rtk_trajectory_filename = "";
  Planning planning;
  planning.Init();

  common::TrajectoryPoint init_adc_point;
  planning.InitFrame(1, 0.0, init_adc_point);

  common::VehicleState::instance()->set_x(586385.782841);
  common::VehicleState::instance()->set_y(4140674.76065);

  common::VehicleState::instance()->set_heading(2.836888814);
  common::VehicleState::instance()->set_linear_velocity(0.0);
  common::VehicleState::instance()->set_angular_velocity(0.0);

  /**
  double time = 0.1;
  ADCTrajectory trajectory;
  bool res_planning = planning.Plan(false, time, &trajectory);
  EXPECT_FALSE(res_planning);
  **/
}

}  // namespace planning
}  // namespace apollo
