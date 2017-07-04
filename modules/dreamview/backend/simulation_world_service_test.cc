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

#include "modules/dreamview/backend/simulation_world_service.h"

#include <iostream>
#include "gtest/gtest.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"

using apollo::common::adapter::MonitorAdapter;
using apollo::common::adapter::ChassisAdapter;
using apollo::common::adapter::LocalizationAdapter;
using apollo::common::adapter::PlanningTrajectoryAdapter;
using apollo::common::monitor::MonitorMessage;
using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::planning::ADCTrajectoryPoint;

namespace apollo {
namespace dreamview {

namespace internal {

class InternalTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    apollo::common::config::VehicleConfigHelper::Init();
  }
};

TEST_F(InternalTest, UpdateMonitorSuccessTest) {
  MonitorMessage monitor;
  monitor.add_item()->set_msg("I am the latest message.");
  monitor.mutable_header()->set_timestamp_sec(2000);

  SimulationWorld world;
  world.mutable_monitor()->mutable_header()->set_timestamp_sec(1990);
  world.mutable_monitor()->add_item()->set_msg("I am the previous message.");

  UpdateSimulationWorld<MonitorAdapter>(monitor, &world);
  EXPECT_EQ(2, world.monitor().item_size());
  EXPECT_EQ("I am the latest message.", world.monitor().item(0).msg());
  EXPECT_EQ("I am the previous message.", world.monitor().item(1).msg());
}

TEST_F(InternalTest, UpdateMonitorRemoveTest) {
  MonitorMessage monitor;
  monitor.add_item()->set_msg("I am message -2");
  monitor.add_item()->set_msg("I am message -1");
  monitor.mutable_header()->set_timestamp_sec(2000);

  SimulationWorld world;
  world.mutable_monitor()->mutable_header()->set_timestamp_sec(1990);
  for (int i = 0; i < SimulationWorldService::kMaxMonitorItems; ++i) {
    world.mutable_monitor()->add_item()->set_msg("I am message " +
                                                 std::to_string(i));
  }
  int last = SimulationWorldService::kMaxMonitorItems - 1;
  EXPECT_EQ("I am message " + std::to_string(last),
            world.monitor().item(last).msg());

  UpdateSimulationWorld<MonitorAdapter>(monitor, &world);
  EXPECT_EQ(SimulationWorldService::kMaxMonitorItems,
            world.monitor().item_size());
  EXPECT_EQ("I am message -2", world.monitor().item(0).msg());
  EXPECT_EQ("I am message -1", world.monitor().item(1).msg());
  EXPECT_EQ("I am message " + std::to_string(last - monitor.item_size()),
            world.monitor().item(last).msg());
}

TEST_F(InternalTest, UpdateMonitorTruncateTest) {
  MonitorMessage monitor;
  int large_size = SimulationWorldService::kMaxMonitorItems + 10;
  for (int i = 0; i < large_size; ++i) {
    monitor.add_item()->set_msg("I am message " + std::to_string(i));
  }
  monitor.mutable_header()->set_timestamp_sec(2000);
  EXPECT_EQ(large_size, monitor.item_size());
  EXPECT_EQ("I am message " + std::to_string(large_size - 1),
            monitor.item(large_size - 1).msg());
  SimulationWorld world;
  world.mutable_monitor()->mutable_header()->set_timestamp_sec(1990);

  UpdateSimulationWorld<MonitorAdapter>(monitor, &world);
  int last = SimulationWorldService::kMaxMonitorItems - 1;
  EXPECT_EQ(SimulationWorldService::kMaxMonitorItems,
            world.monitor().item_size());
  EXPECT_EQ("I am message 0", world.monitor().item(0).msg());
  EXPECT_EQ("I am message " + std::to_string(last),
            world.monitor().item(last).msg());
}

TEST_F(InternalTest, UpdateChassisInfoTest) {
  // Prepare the chassis message that will be used to update the
  // SimulationWorld object.
  ::apollo::canbus::Chassis chassis;
  chassis.set_speed_mps(25);
  chassis.set_throttle_percentage(50);
  chassis.set_brake_percentage(10);
  chassis.set_steering_percentage(25);
  chassis.mutable_signal()->set_turn_signal(apollo::canbus::Signal::TURN_RIGHT);

  // Commit the update.
  SimulationWorld world;
  UpdateSimulationWorld<ChassisAdapter>(chassis, &world);

  // Check the update reuslt.
  const Object &car = world.auto_driving_car();
  EXPECT_DOUBLE_EQ(4.933, car.length());
  EXPECT_DOUBLE_EQ(2.11, car.width());
  EXPECT_DOUBLE_EQ(1.48, car.height());
  EXPECT_DOUBLE_EQ(25.0, car.speed());
  EXPECT_DOUBLE_EQ(50.0, car.throttle_percentage());
  EXPECT_DOUBLE_EQ(10.0, car.brake_percentage());
  EXPECT_DOUBLE_EQ(25.0, car.steering_angle());
  EXPECT_EQ("RIGHT", car.current_signal());
}

TEST_F(InternalTest, UpdateLocalizationTest) {
  // Prepare the localization message that will be used to update the
  // SimulationWorld object.
  ::apollo::localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(1.0);
  localization.mutable_pose()->mutable_position()->set_y(1.5);
  localization.mutable_pose()->mutable_orientation()->set_qx(0.0);
  localization.mutable_pose()->mutable_orientation()->set_qy(0.0);
  localization.mutable_pose()->mutable_orientation()->set_qz(0.0);
  localization.mutable_pose()->mutable_orientation()->set_qw(0.0);

  auto pose = localization.pose();
  auto heading = apollo::common::math::QuaternionToHeading(
      pose.orientation().qw(), pose.orientation().qx(), pose.orientation().qy(),
      pose.orientation().qz());
  localization.mutable_pose()->set_heading(heading);

  // Commit the update.
  SimulationWorld world;
  UpdateSimulationWorld<LocalizationAdapter>(localization, &world);

  // Check the update result.
  const Object &car = world.auto_driving_car();
  EXPECT_DOUBLE_EQ(1.0, car.position_x());
  EXPECT_DOUBLE_EQ(1.5, car.position_y());
  EXPECT_DOUBLE_EQ(
      apollo::common::math::QuaternionToHeading(0.0, 0.0, 0.0, 0.0),
      car.heading());
}

TEST_F(InternalTest, UpdatePlanningTrajectoryTest) {
  // Prepare the trajectory message that will be used to update the
  // SimulationWorld object.
  ADCTrajectory planning_trajectory;
  for (int i = 0; i < 30; ++i) {
    ADCTrajectoryPoint *point = planning_trajectory.add_adc_trajectory_point();
    point->set_x(i * 10);
    point->set_y(i * 10 + 10);
  }

  // Commit the update.
  SimulationWorld world;
  UpdateSimulationWorld<PlanningTrajectoryAdapter>(planning_trajectory, &world);

  // Check the update result.
  EXPECT_EQ(world.planning_trajectory_size(), 4);

  // Check first point.
  {
    const Object point = world.planning_trajectory(0);
    EXPECT_DOUBLE_EQ(0.0, point.position_x());
    EXPECT_DOUBLE_EQ(10.0, point.position_y());
    EXPECT_DOUBLE_EQ(atan2(100.0, 100.0), point.heading());
    EXPECT_EQ(point.polygon_point_size(), 4);
  }

  // Check last point.
  {
    const Object point = world.planning_trajectory(3);
    EXPECT_DOUBLE_EQ(280.0, point.position_x());
    EXPECT_DOUBLE_EQ(290.0, point.position_y());
    EXPECT_DOUBLE_EQ(atan2(100.0, 100.0), point.heading());
    EXPECT_EQ(point.polygon_point_size(), 4);
  }
}
}  // namespace internal

}  // namespace dreamview
}  // namespace apollo
