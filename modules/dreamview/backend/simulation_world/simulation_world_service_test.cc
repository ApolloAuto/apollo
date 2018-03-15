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

#include "modules/dreamview/backend/simulation_world/simulation_world_service.h"

#include <iostream>
#include "gtest/gtest.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"

#include "modules/dreamview/backend/common/dreamview_gflags.h"

using apollo::canbus::Chassis;
using apollo::common::TrajectoryPoint;
using apollo::common::monitor::MonitorMessage;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;
using apollo::planning::DecisionResult;
using apollo::prediction::PredictionObstacle;
using apollo::prediction::PredictionObstacles;

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::common::util::StrCat;

namespace apollo {
namespace dreamview {

const float kEpsilon = 0.01;

class SimulationWorldServiceTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    // Setup AdapterManager.
    AdapterManagerConfig config;
    config.set_is_ros(false);
    {
      auto* sub_config = config.add_config();
      sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
      sub_config->set_type(AdapterConfig::MONITOR);
    }
    {
      auto* sub_config = config.add_config();
      sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
      sub_config->set_type(AdapterConfig::ROUTING_RESPONSE);
    }
    AdapterManager::Reset();
    AdapterManager::Init(config);

    FLAGS_routing_response_file =
        "modules/dreamview/backend/testdata/routing.pb.txt";
    apollo::common::VehicleConfigHelper::Init();
    sim_world_service_.reset(new SimulationWorldService(map_service_.get()));
  }

 protected:
  SimulationWorldServiceTest() {
    FLAGS_map_dir = "modules/dreamview/backend/testdata";
    FLAGS_base_map_filename = "garage.bin";
    FLAGS_sim_world_with_routing_path = true;
    map_service_.reset(new MapService(false));
  }

  std::unique_ptr<SimulationWorldService> sim_world_service_;
  std::unique_ptr<MapService> map_service_;
};

TEST_F(SimulationWorldServiceTest, UpdateMonitorSuccess) {
  MonitorMessage monitor;
  monitor.add_item()->set_msg("I am the latest message.");
  monitor.mutable_header()->set_timestamp_sec(2000);

  auto* world_monitor = sim_world_service_->world_.mutable_monitor();
  world_monitor->mutable_header()->set_timestamp_sec(1990);
  world_monitor->add_item()->set_msg("I am the previous message.");

  sim_world_service_->UpdateSimulationWorld(monitor);

  EXPECT_EQ(2, world_monitor->item_size());
  EXPECT_EQ("I am the latest message.", world_monitor->item(0).msg());
  EXPECT_EQ("I am the previous message.", world_monitor->item(1).msg());
}

TEST_F(SimulationWorldServiceTest, UpdateMonitorRemove) {
  MonitorMessage monitor;
  monitor.add_item()->set_msg("I am message -2");
  monitor.add_item()->set_msg("I am message -1");
  monitor.mutable_header()->set_timestamp_sec(2000);

  auto* world_monitor = sim_world_service_->world_.mutable_monitor();
  world_monitor->mutable_header()->set_timestamp_sec(1990);
  for (int i = 0; i < SimulationWorldService::kMaxMonitorItems; ++i) {
    world_monitor->add_item()->set_msg(StrCat("I am message ", i));
  }
  int last = SimulationWorldService::kMaxMonitorItems - 1;
  EXPECT_EQ(StrCat("I am message ", last), world_monitor->item(last).msg());

  sim_world_service_->UpdateSimulationWorld(monitor);

  EXPECT_EQ(SimulationWorldService::kMaxMonitorItems,
            world_monitor->item_size());
  EXPECT_EQ("I am message -2", world_monitor->item(0).msg());
  EXPECT_EQ("I am message -1", world_monitor->item(1).msg());
  EXPECT_EQ(StrCat("I am message ", last - monitor.item_size()),
            world_monitor->item(last).msg());
}

TEST_F(SimulationWorldServiceTest, UpdateMonitorTruncate) {
  MonitorMessage monitor;
  int large_size = SimulationWorldService::kMaxMonitorItems + 10;
  for (int i = 0; i < large_size; ++i) {
    monitor.add_item()->set_msg(StrCat("I am message ", i));
  }
  monitor.mutable_header()->set_timestamp_sec(2000);
  EXPECT_EQ(large_size, monitor.item_size());
  EXPECT_EQ(StrCat("I am message ", large_size - 1),
            monitor.item(large_size - 1).msg());

  auto* world_monitor = sim_world_service_->world_.mutable_monitor();
  world_monitor->mutable_header()->set_timestamp_sec(1990);

  sim_world_service_->UpdateSimulationWorld(monitor);

  int last = SimulationWorldService::kMaxMonitorItems - 1;
  EXPECT_EQ(SimulationWorldService::kMaxMonitorItems,
            world_monitor->item_size());
  EXPECT_EQ("I am message 0", world_monitor->item(0).msg());
  EXPECT_EQ(StrCat("I am message ", last), world_monitor->item(last).msg());
}

TEST_F(SimulationWorldServiceTest, UpdateChassisInfo) {
  // Prepare the chassis message that will be used to update the
  // SimulationWorld object.
  Chassis chassis;
  chassis.set_speed_mps(25);
  chassis.set_throttle_percentage(50);
  chassis.set_brake_percentage(10);
  chassis.set_steering_percentage(25);
  chassis.mutable_signal()->set_turn_signal(
      apollo::common::VehicleSignal::TURN_RIGHT);

  // Commit the update.
  sim_world_service_->UpdateSimulationWorld(chassis);

  // Check the update reuslt.
  const Object& car = sim_world_service_->world_.auto_driving_car();
  EXPECT_DOUBLE_EQ(4.933, car.length());
  EXPECT_DOUBLE_EQ(2.11, car.width());
  EXPECT_DOUBLE_EQ(1.48, car.height());
  EXPECT_DOUBLE_EQ(25.0, car.speed());
  EXPECT_DOUBLE_EQ(50.0, car.throttle_percentage());
  EXPECT_DOUBLE_EQ(10.0, car.brake_percentage());
  EXPECT_DOUBLE_EQ(25.0, car.steering_percentage());
  EXPECT_EQ("RIGHT", car.current_signal());
}

TEST_F(SimulationWorldServiceTest, UpdateLocalization) {
  // Prepare the localization message that will be used to update the
  // SimulationWorld object.
  LocalizationEstimate localization;
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
  sim_world_service_->UpdateSimulationWorld(localization);

  // Check the update result.
  const Object& car = sim_world_service_->world_.auto_driving_car();
  EXPECT_DOUBLE_EQ(1.0, car.position_x());
  EXPECT_DOUBLE_EQ(1.5, car.position_y());
  EXPECT_DOUBLE_EQ(
      apollo::common::math::QuaternionToHeading(0.0, 0.0, 0.0, 0.0),
      car.heading());
}

TEST_F(SimulationWorldServiceTest, UpdatePerceptionObstacles) {
  PerceptionObstacles obstacles;
  PerceptionObstacle* obstacle1 = obstacles.add_perception_obstacle();
  obstacle1->set_id(1);
  apollo::perception::Point* point1 = obstacle1->add_polygon_point();
  point1->set_x(0.0);
  point1->set_y(0.0);
  apollo::perception::Point* point2 = obstacle1->add_polygon_point();
  point2->set_x(0.0);
  point2->set_y(1.0);
  apollo::perception::Point* point3 = obstacle1->add_polygon_point();
  point3->set_x(-1.0);
  point3->set_y(0.0);
  apollo::perception::Point* point4 = obstacle1->add_polygon_point();
  point4->set_x(-1.0);
  point4->set_y(0.0);
  obstacle1->set_timestamp(1489794020.123);
  obstacle1->set_type(apollo::perception::PerceptionObstacle_Type_UNKNOWN);
  PerceptionObstacle* obstacle2 = obstacles.add_perception_obstacle();
  obstacle2->set_id(2);
  apollo::perception::Point* point = obstacle2->mutable_position();
  point->set_x(1.0);
  point->set_y(2.0);
  obstacle2->set_theta(3.0);
  obstacle2->set_length(4.0);
  obstacle2->set_width(5.0);
  obstacle2->set_height(6.0);
  obstacle2->mutable_velocity()->set_x(3.0);
  obstacle2->mutable_velocity()->set_y(4.0);
  obstacle2->set_type(apollo::perception::PerceptionObstacle_Type_VEHICLE);

  sim_world_service_->UpdateSimulationWorld(obstacles);
  sim_world_service_->world_.clear_object();
  for (auto& kv : sim_world_service_->obj_map_) {
    *sim_world_service_->world_.add_object() = kv.second;
  }
  EXPECT_EQ(2, sim_world_service_->world_.object_size());

  for (const auto& object : sim_world_service_->world_.object()) {
    if (object.id() == "1") {
      EXPECT_DOUBLE_EQ(1489794020.123, object.timestamp_sec());
      EXPECT_EQ(3, object.polygon_point_size());
      EXPECT_EQ(Object_Type_UNKNOWN, object.type());
    } else if (object.id() == "2") {
      EXPECT_DOUBLE_EQ(1.0, object.position_x());
      EXPECT_DOUBLE_EQ(2.0, object.position_y());
      EXPECT_DOUBLE_EQ(3.0, object.heading());
      EXPECT_DOUBLE_EQ(4.0, object.length());
      EXPECT_DOUBLE_EQ(5.0, object.width());
      EXPECT_DOUBLE_EQ(6.0, object.height());
      EXPECT_DOUBLE_EQ(5.0, object.speed());
      EXPECT_NEAR(0.927295, object.speed_heading(), kEpsilon);
      EXPECT_EQ(0, object.polygon_point_size());
      EXPECT_EQ(Object_Type_VEHICLE, object.type());
    } else {
      EXPECT_TRUE(false) << "Unexpected object id " << object.id();
    }
  }
}

TEST_F(SimulationWorldServiceTest, UpdatePlanningTrajectory) {
  // Prepare the trajectory message that will be used to update the
  // SimulationWorld object.
  ADCTrajectory planning_trajectory;
  for (int i = 0; i < 30; ++i) {
    TrajectoryPoint* point = planning_trajectory.add_trajectory_point();
    point->mutable_path_point()->set_x(i * 10);
    point->mutable_path_point()->set_y(i * 10 + 10);
  }

  // Commit the update.
  sim_world_service_->UpdatePlanningTrajectory(planning_trajectory);

  // Check the update result.
  const SimulationWorld& world = sim_world_service_->world();
  EXPECT_EQ(29, world.planning_trajectory_size());

  // Check first point.
  {
    const Object point = world.planning_trajectory(0);
    EXPECT_DOUBLE_EQ(0.0, point.position_x());
    EXPECT_DOUBLE_EQ(10.0, point.position_y());
    EXPECT_DOUBLE_EQ(atan2(100.0, 100.0), point.heading());
  }

  // Check last point.
  {
    const Object point =
        world.planning_trajectory(world.planning_trajectory_size() - 1);
    EXPECT_DOUBLE_EQ(280.0, point.position_x());
    EXPECT_DOUBLE_EQ(290.0, point.position_y());
    EXPECT_DOUBLE_EQ(atan2(100.0, 100.0), point.heading());
  }
}

TEST_F(SimulationWorldServiceTest, UpdateDecision) {
  DecisionResult decision_res;

  decision_res.mutable_vehicle_signal()->set_turn_signal(
      apollo::common::VehicleSignal::TURN_RIGHT);

  apollo::planning::MainDecision* main_decision =
      decision_res.mutable_main_decision();
  main_decision->add_target_lane()->set_speed_limit(35);
  apollo::planning::MainStop* main_stop = main_decision->mutable_stop();
  main_stop->mutable_stop_point()->set_x(45678.9);
  main_stop->mutable_stop_point()->set_y(1234567.8);
  main_stop->set_stop_heading(1.234);
  main_stop->set_reason_code(
      apollo::planning::StopReasonCode::STOP_REASON_CROSSWALK);

  apollo::planning::ObjectDecisions* obj_decisions =
      decision_res.mutable_object_decision();
  // The 1st obstacle is from perception and has 2 decisions: nudge or sidepass.
  apollo::planning::ObjectDecision* obj_decision1 =
      obj_decisions->add_decision();
  obj_decision1->set_perception_id(1);
  Object& perception1 = sim_world_service_->obj_map_["1"];
  perception1.set_type(Object_Type_UNKNOWN_UNMOVABLE);
  perception1.set_id("1");
  perception1.set_position_x(1.0);
  perception1.set_position_y(2.0);
  perception1.set_heading(3.0);
  perception1.set_length(4.0);
  perception1.set_width(5.0);
  perception1.set_height(6.0);
  for (int i = 0; i < 3; ++i) {
    PolygonPoint* perception_pt = perception1.add_polygon_point();
    if (i < 2) {
      perception_pt->set_x(10.0 * (i + 1));
      perception_pt->set_y(20.0 * (i + 1));
    } else {
      perception_pt->set_y(10.0 * (i + 1));
      perception_pt->set_x(20.0 * (i + 1));
    }
  }
  obj_decision1->add_object_decision()->mutable_nudge()->set_distance_l(1.8);
  obj_decision1->add_object_decision()->mutable_sidepass();
  // The 2nd obstacle is virtual and has only 1 decision: yield.
  apollo::planning::ObjectDecision* obj_decision2 =
      obj_decisions->add_decision();
  obj_decision2->set_perception_id(2);
  apollo::planning::ObjectYield* yield =
      obj_decision2->add_object_decision()->mutable_yield();
  apollo::common::PointENU* fence_point = yield->mutable_fence_point();
  fence_point->set_x(-1859.98);
  fence_point->set_y(-3000.03);
  yield->set_fence_heading(1.3);

  sim_world_service_->UpdateDecision(decision_res, 1501095053);

  const SimulationWorld& world = sim_world_service_->world();
  EXPECT_EQ("RIGHT", world.auto_driving_car().current_signal());
  EXPECT_EQ(35, world.speed_limit());

  const Object& world_main_stop = world.main_decision();
  EXPECT_EQ(1, world_main_stop.decision_size());
  EXPECT_EQ(Decision::STOP_REASON_CROSSWALK,
            world_main_stop.decision(0).stopreason());
  EXPECT_DOUBLE_EQ(45678.9, world_main_stop.position_x());
  EXPECT_DOUBLE_EQ(1234567.8, world_main_stop.position_y());
  EXPECT_DOUBLE_EQ(1.234, world_main_stop.heading());

  sim_world_service_->world_.clear_object();
  for (auto& kv : sim_world_service_->obj_map_) {
    *sim_world_service_->world_.add_object() = kv.second;
  }
  EXPECT_EQ(2, world.object_size());

  for (int i = 0; i < 2; ++i) {
    const Object& obj_dec = world.object(i);
    if (obj_dec.id() == "1") {
      EXPECT_EQ(Object_Type_UNKNOWN_UNMOVABLE, obj_dec.type());
      EXPECT_DOUBLE_EQ(1.0, obj_dec.position_x());
      EXPECT_DOUBLE_EQ(2.0, obj_dec.position_y());
      EXPECT_DOUBLE_EQ(3.0, obj_dec.heading());
      EXPECT_DOUBLE_EQ(4.0, obj_dec.length());
      EXPECT_DOUBLE_EQ(5.0, obj_dec.width());
      EXPECT_DOUBLE_EQ(6.0, obj_dec.height());
      EXPECT_EQ(obj_dec.decision_size(), 2);
      for (int j = 0; j < obj_dec.decision_size(); ++j) {
        const Decision& decision = obj_dec.decision(j);
        if (j == 0) {
          EXPECT_EQ(decision.type(), Decision_Type_NUDGE);
          EXPECT_EQ(decision.polygon_point_size(), 67);
        } else {
          EXPECT_EQ(decision.type(), Decision_Type_SIDEPASS);
        }
      }
    } else {
      EXPECT_EQ(Object_Type_VIRTUAL, obj_dec.type());
      EXPECT_EQ(1, obj_dec.decision_size());
      const Decision& decision = obj_dec.decision(0);
      EXPECT_EQ(Decision_Type_YIELD, decision.type());
      EXPECT_DOUBLE_EQ(-1859.98, decision.position_x());
      EXPECT_DOUBLE_EQ(-3000.03, decision.position_y());
      EXPECT_DOUBLE_EQ(1.3, decision.heading());
    }
  }
}

TEST_F(SimulationWorldServiceTest, UpdatePrediction) {
  // Update with prediction obstacles
  PredictionObstacles prediction_obstacles;
  for (int i = 0; i < 3; ++i) {
    auto* obstacle = prediction_obstacles.add_prediction_obstacle();
    auto* perception_obstacle = obstacle->mutable_perception_obstacle();
    perception_obstacle->set_id(i);
    for (int j = 0; j < 5; ++j) {
      auto* traj = obstacle->add_trajectory();
      traj->set_probability(i * 0.1 + j);
      for (int k = 0; k < 8; ++k) {
        auto* traj_pt = traj->add_trajectory_point()->mutable_path_point();
        int pt = j * 10 + k;
        traj_pt->set_x(pt);
        traj_pt->set_y(pt);
        traj_pt->set_z(pt);
      }
    }
    obstacle->set_timestamp(123.456);
  }
  sim_world_service_->UpdateSimulationWorld(prediction_obstacles);
  sim_world_service_->world_.clear_object();
  for (auto& kv : sim_world_service_->obj_map_) {
    *sim_world_service_->world_.add_object() = kv.second;
  }

  // Verify
  const auto& sim_world = sim_world_service_->world();
  EXPECT_EQ(3, sim_world.object_size());
  for (int i = 0; i < sim_world.object_size(); ++i) {
    const Object& obj = sim_world.object(i);
    EXPECT_EQ(5, obj.prediction_size());

    for (int j = 0; j < obj.prediction_size(); ++j) {
      const Prediction& prediction = obj.prediction(j);
      EXPECT_NEAR((sim_world.object_size() - i - 1) * 0.1 + j,
                  prediction.probability(), kEpsilon);
      EXPECT_EQ(prediction.predicted_trajectory_size(), 2);  // Downsampled
    }
    EXPECT_NEAR(123.456, obj.timestamp_sec(), kEpsilon);
  }
}

TEST_F(SimulationWorldServiceTest, UpdateRouting) {
  // Load routing from file
  sim_world_service_.reset(
      new SimulationWorldService(map_service_.get(), true));
  sim_world_service_->UpdateSimulationWorld(
      *AdapterManager::GetRoutingResponse()->GetLatestPublished());

  auto& world = sim_world_service_->world_;
  EXPECT_EQ(world.routing_time(), 1234.5);
  EXPECT_EQ(1, world.route_path_size());

  double points[23][2] = {
      {-1826.41, -3027.52}, {-1839.88, -3023.9},  {-1851.95, -3020.71},
      {-1857.06, -3018.62}, {-1858.04, -3017.94}, {-1859.56, -3016.51},
      {-1860.48, -3014.95}, {-1861.12, -3013.2},  {-1861.62, -3010.06},
      {-1861.29, -3005.88}, {-1859.8, -2999.36},  {-1855.8, -2984.56},
      {-1851.39, -2968.23}, {-1844.32, -2943.14}, {-1842.9, -2939.22},
      {-1841.74, -2937.09}, {-1839.35, -2934.03}, {-1837.76, -2932.88},
      {-1835.53, -2931.86}, {-1833.36, -2931.52}, {-1831.33, -2931.67},
      {-1827.05, -2932.6},  {-1809.64, -2937.85}};

  const auto& path = world.route_path(0);
  EXPECT_EQ(23, path.point_size());
  for (int i = 0; i < path.point_size(); ++i) {
    EXPECT_NEAR(path.point(i).x(), points[i][0], kEpsilon);
    EXPECT_NEAR(path.point(i).y(), points[i][1], kEpsilon);
  }
}

}  // namespace dreamview
}  // namespace apollo
