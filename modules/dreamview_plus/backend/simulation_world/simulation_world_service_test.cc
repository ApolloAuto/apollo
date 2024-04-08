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

#include "modules/dreamview_plus/backend/simulation_world/simulation_world_service.h"

#include <string>

#include "gtest/gtest.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

using apollo::canbus::Chassis;
using apollo::common::TrajectoryPoint;
using apollo::common::monitor::MonitorMessage;
using apollo::cyber::blocker::BlockerManager;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;
using apollo::planning::DecisionResult;
using apollo::prediction::PredictionObstacles;

namespace apollo {
namespace dreamview {

const float kEpsilon = 0.01f;

class SimulationWorldServiceTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    cyber::GlobalData::Instance()->EnableSimulationMode();

    std::unique_ptr<cyber::Node> node =
        cyber::CreateNode("sim_world_service_test");
    control_writer_ = node->CreateWriter<apollo::control::ControlCommand>(
        FLAGS_control_command_topic);
  }

  virtual void SetUp() {
    FLAGS_routing_response_file =
        "modules/dreamview_plus/backend/testdata/routing.pb.txt";
    apollo::common::VehicleConfigHelper::Init();
    apollo::cyber::Init("dreamview");
    sim_world_service_.reset(new SimulationWorldService(map_service_.get()));
  }

 protected:
  SimulationWorldServiceTest() {
    FLAGS_map_dir = "modules/dreamview_plus/backend/testdata";
    // demo
    FLAGS_base_map_filename = "base_map.txt";
    FLAGS_sim_world_with_routing_path = true;
    map_service_.reset(new MapService(false));
  }

  std::unique_ptr<SimulationWorldService> sim_world_service_;
  std::unique_ptr<MapService> map_service_;

  static std::shared_ptr<cyber::Writer<apollo::control::ControlCommand>>
      control_writer_;
};

std::shared_ptr<cyber::Writer<apollo::control::ControlCommand>>
    SimulationWorldServiceTest::control_writer_;

TEST_F(SimulationWorldServiceTest, UpdateMonitorSuccess) {
  MonitorMessage monitor;
  monitor.add_item()->set_msg("I am the latest message.");
  monitor.mutable_header()->set_timestamp_sec(2000);

  auto* notification = sim_world_service_->world_.add_notification();
  notification->set_timestamp_sec(1990);
  notification->mutable_item()->set_msg("I am the previous message.");

  sim_world_service_->UpdateSimulationWorld(monitor);

  EXPECT_EQ(2, sim_world_service_->world_.notification_size());
  EXPECT_EQ("I am the previous message.",
            sim_world_service_->world_.notification(0).item().msg());
  EXPECT_EQ("I am the latest message.",
            sim_world_service_->world_.notification(1).item().msg());
  EXPECT_DOUBLE_EQ(1990,
                   sim_world_service_->world_.notification(0).timestamp_sec());
  EXPECT_DOUBLE_EQ(2000,
                   sim_world_service_->world_.notification(1).timestamp_sec());
}

TEST_F(SimulationWorldServiceTest, UpdateMonitorRemove) {
  MonitorMessage monitor;
  monitor.add_item()->set_msg("I am message -1");
  monitor.add_item()->set_msg("I am message -2");
  monitor.mutable_header()->set_timestamp_sec(2000);

  for (int i = 0; i < SimulationWorldService::kMaxMonitorItems; ++i) {
    auto* notification = sim_world_service_->world_.add_notification();
    notification->mutable_item()->set_msg(absl::StrCat("I am message ", i));
    notification->set_timestamp_sec(1990);
  }
  int last = SimulationWorldService::kMaxMonitorItems - 1;
  EXPECT_EQ(absl::StrCat("I am message ", last),
            sim_world_service_->world_.notification(last).item().msg());

  sim_world_service_->UpdateSimulationWorld(monitor);

  EXPECT_EQ(SimulationWorldService::kMaxMonitorItems,
            sim_world_service_->world_.notification_size());
  EXPECT_EQ("I am message -2",
            sim_world_service_->world_.notification(last).item().msg());
  EXPECT_EQ("I am message -1",
            sim_world_service_->world_.notification(last - 1).item().msg());
  EXPECT_EQ(absl::StrCat("I am message ", last),
            sim_world_service_->world_.notification(last - 2).item().msg());
  EXPECT_DOUBLE_EQ(
      2000, sim_world_service_->world_.notification(last).timestamp_sec());
  EXPECT_DOUBLE_EQ(
      2000, sim_world_service_->world_.notification(last - 1).timestamp_sec());
  EXPECT_DOUBLE_EQ(
      1990, sim_world_service_->world_.notification(last - 2).timestamp_sec());
}

TEST_F(SimulationWorldServiceTest, UpdateMonitorTruncate) {
  MonitorMessage monitor;
  int large_size = SimulationWorldService::kMaxMonitorItems + 10;
  for (int i = 0; i < large_size; ++i) {
    monitor.add_item()->set_msg(absl::StrCat("I am message ", i));
  }
  monitor.mutable_header()->set_timestamp_sec(2000);
  EXPECT_EQ(large_size, monitor.item_size());
  EXPECT_EQ(absl::StrCat("I am message ", large_size - 1),
            monitor.item(large_size - 1).msg());

  sim_world_service_->UpdateSimulationWorld(monitor);

  int last = SimulationWorldService::kMaxMonitorItems - 1;
  EXPECT_EQ(SimulationWorldService::kMaxMonitorItems,
            sim_world_service_->world_.notification_size());
  EXPECT_EQ("I am message 0",
            sim_world_service_->world_.notification(0).item().msg());
  EXPECT_EQ(absl::StrCat("I am message ", last),
            sim_world_service_->world_.notification(last).item().msg());
  EXPECT_DOUBLE_EQ(2000,
                   sim_world_service_->world_.notification(0).timestamp_sec());
  EXPECT_DOUBLE_EQ(
      2000, sim_world_service_->world_.notification(last).timestamp_sec());
}

TEST_F(SimulationWorldServiceTest, UpdateChassisInfo) {
  // Prepare the chassis message that will be used to update the
  // SimulationWorld object.
  Chassis chassis;
  chassis.set_speed_mps(25);
  chassis.set_throttle_percentage(50);
  chassis.set_brake_percentage(10);
  chassis.set_steering_percentage(25);
  chassis.set_battery_soc_percentage(80);
  chassis.set_gear_location(Chassis::GEAR_DRIVE);
  chassis.mutable_signal()->set_turn_signal(
      apollo::common::VehicleSignal::TURN_RIGHT);

  // Commit the update.
  sim_world_service_->UpdateSimulationWorld(chassis);

  // Check the update result.
  const Object& car = sim_world_service_->world_.auto_driving_car();
  EXPECT_DOUBLE_EQ(4.933, car.length());
  EXPECT_DOUBLE_EQ(2.11, car.width());
  EXPECT_DOUBLE_EQ(1.48, car.height());
  EXPECT_DOUBLE_EQ(25.0, car.speed());
  EXPECT_DOUBLE_EQ(50.0, car.throttle_percentage());
  EXPECT_DOUBLE_EQ(10.0, car.brake_percentage());
  EXPECT_DOUBLE_EQ(25.0, car.steering_percentage());
  EXPECT_EQ(80, car.battery_percentage());
  EXPECT_EQ(Chassis::GEAR_DRIVE, car.gear_location());
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
  apollo::common::Point3D* point1 = obstacle1->add_polygon_point();
  point1->set_x(0.0);
  point1->set_y(0.0);
  apollo::common::Point3D* point2 = obstacle1->add_polygon_point();
  point2->set_x(0.0);
  point2->set_y(1.0);
  apollo::common::Point3D* point3 = obstacle1->add_polygon_point();
  point3->set_x(-1.0);
  point3->set_y(0.0);
  apollo::common::Point3D* point4 = obstacle1->add_polygon_point();
  point4->set_x(-1.0);
  point4->set_y(0.0);
  obstacle1->set_timestamp(1489794020.123);
  obstacle1->set_type(apollo::perception::PerceptionObstacle_Type_UNKNOWN);
  PerceptionObstacle* obstacle2 = obstacles.add_perception_obstacle();
  obstacle2->set_id(2);
  apollo::common::Point3D* point = obstacle2->mutable_position();
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
    point->mutable_path_point()->set_theta(i * 0.1);
  }

  // Commit the update.
  sim_world_service_->UpdatePlanningTrajectory(planning_trajectory);

  // Check the update result.
  const SimulationWorld& world = sim_world_service_->world();
  EXPECT_EQ(30, world.planning_trajectory_size());

  // Check first point.
  {
    const Object point = world.planning_trajectory(0);
    EXPECT_DOUBLE_EQ(0.0, point.position_x());
    EXPECT_DOUBLE_EQ(10.0, point.position_y());
    EXPECT_DOUBLE_EQ(0, point.heading());
  }

  // Check last point.
  {
    const Object point =
        world.planning_trajectory(world.planning_trajectory_size() - 1);
    EXPECT_DOUBLE_EQ(290.0, point.position_x());
    EXPECT_DOUBLE_EQ(300.0, point.position_y());
    EXPECT_DOUBLE_EQ(2.9, point.heading());
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
  // The 1st obstacle is from perception and has 1 decisions: nudge.
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
  const Decision& decision = world_main_stop.decision(0);
  EXPECT_EQ(Decision::STOP_REASON_CROSSWALK, decision.stopreason());
  EXPECT_DOUBLE_EQ(45678.9, decision.position_x());
  EXPECT_DOUBLE_EQ(1234567.8, decision.position_y());
  EXPECT_DOUBLE_EQ(1.234, decision.heading());

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
      EXPECT_EQ(obj_dec.decision_size(), 1);
      for (int j = 0; j < obj_dec.decision_size(); ++j) {
        const Decision& decision = obj_dec.decision(j);
        if (j == 0) {
          EXPECT_EQ(decision.type(), Decision_Type_NUDGE);
          EXPECT_EQ(decision.polygon_point_size(), 67);
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
  std::set<std::pair<int, double>> original_probabilities;
  for (int i = 0; i < 3; ++i) {
    auto* obstacle = prediction_obstacles.add_prediction_obstacle();
    auto* perception_obstacle = obstacle->mutable_perception_obstacle();
    perception_obstacle->set_id(i);
    for (int j = 0; j < 5; ++j) {
      auto* traj = obstacle->add_trajectory();
      traj->set_probability(i * 0.1 + j);
      original_probabilities.emplace(perception_obstacle->id(),
                                     traj->probability());
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
      const std::pair<int, double> item_to_find(std::stoi(obj.id()),
                                                prediction.probability());
      const auto id_prob_it = original_probabilities.find(item_to_find);
      EXPECT_NE(id_prob_it, original_probabilities.end());
      if (id_prob_it != original_probabilities.end()) {
        original_probabilities.erase(id_prob_it);
      }
      EXPECT_EQ(prediction.predicted_trajectory_size(), 2);  // Downsampled
    }
    EXPECT_NEAR(123.456, obj.timestamp_sec(), kEpsilon);
  }
  EXPECT_TRUE(original_probabilities.empty());
}

TEST_F(SimulationWorldServiceTest, UpdateRouting) {
  // Load routing from file
  sim_world_service_.reset(nullptr);
  sim_world_service_.reset(
      new SimulationWorldService(map_service_.get(), true));

  BlockerManager::Instance()->Observe();
  sim_world_service_->UpdateWithLatestObserved(
      sim_world_service_->planning_command_reader_.get());

  auto& world = sim_world_service_->world_;
  EXPECT_EQ(world.routing_time(), 1234.5);
  EXPECT_EQ(1, world.route_path_size());

  double points[2][2] = {{586392.84, 4140673.01},
                         {586376.61, 4140677.16}};

  const auto& path = world.route_path(0);
  EXPECT_EQ(2, path.point_size());
  for (int i = 0; i < path.point_size(); ++i) {
    EXPECT_NEAR(path.point(i).x(), points[i][0], kEpsilon);
    EXPECT_NEAR(path.point(i).y(), points[i][1], kEpsilon);
  }
}

TEST_F(SimulationWorldServiceTest, UpdateGps) {
  // Prepare the gps message that will be used to update the
  // SimulationWorld object.
  apollo::localization::Gps gps;
  auto* pose = gps.mutable_localization();
  auto* position = pose->mutable_position();
  position->set_x(586922.10396);
  position->set_y(4141065.07993);
  position->set_z(-30.60015);
  auto* orientation = pose->mutable_orientation();
  orientation->set_qx(-0.01742);
  orientation->set_qy(0.00928);
  orientation->set_qz(-0.12262);
  orientation->set_qw(0.99226);

  sim_world_service_->UpdateSimulationWorld(gps);

  // Verify
  const auto& gps_position = sim_world_service_->world().gps();
  EXPECT_DOUBLE_EQ(586922.10396, gps_position.position_x());
  EXPECT_DOUBLE_EQ(4141065.07993, gps_position.position_y());
  EXPECT_NEAR(1.32515, gps_position.heading(), 0.00001);
}

TEST_F(SimulationWorldServiceTest, UpdateControlCommandWithSimpleLonLat) {
  // Prepare the ControlCommand message that will be used to update the
  // SimulationWorld object.
  apollo::control::ControlCommand control_command;
  const double station_error = 0.90225;
  const double heading_error = -0.00097;
  const double lateral_error = 0.004176;
  control_command.mutable_header()->set_timestamp_sec(2000);
  auto* debug = control_command.mutable_debug();
  auto* simple_lon = debug->mutable_simple_lon_debug();
  simple_lon->set_station_error(station_error);

  auto* simple_lat = debug->mutable_simple_lat_debug();
  simple_lat->set_heading_error(heading_error);
  simple_lat->set_lateral_error(lateral_error);
  sim_world_service_->UpdateSimulationWorld(control_command);

  // Verify
  const auto& control_data = sim_world_service_->world().control_data();
  EXPECT_DOUBLE_EQ(station_error, control_data.station_error());
  EXPECT_DOUBLE_EQ(heading_error, control_data.heading_error());
  EXPECT_DOUBLE_EQ(lateral_error, control_data.lateral_error());
}

TEST_F(SimulationWorldServiceTest, UpdateControlCommandWithSimpleMpc) {
  // Prepare the ControlCommand message that will be used to update the
  // SimulationWorld object.
  apollo::control::ControlCommand control_command;
  const double station_error = 0.91225;
  const double heading_error = -0.01097;
  const double lateral_error = 0.014176;
  control_command.mutable_header()->set_timestamp_sec(3000);
  auto* debug = control_command.mutable_debug();
  auto* simple_mpc = debug->mutable_simple_mpc_debug();
  simple_mpc->set_station_error(station_error);
  simple_mpc->set_heading_error(heading_error);
  simple_mpc->set_lateral_error(lateral_error);
  sim_world_service_->UpdateSimulationWorld(control_command);

  // Verify
  const auto& control_data = sim_world_service_->world().control_data();
  EXPECT_DOUBLE_EQ(station_error, control_data.station_error());
  EXPECT_DOUBLE_EQ(heading_error, control_data.heading_error());
  EXPECT_DOUBLE_EQ(lateral_error, control_data.lateral_error());
}

TEST_F(SimulationWorldServiceTest, DownsampleSpeedPointsByInterval) {
  apollo::planning_internal::STGraphDebug graph;
  auto* speed_points = graph.mutable_speed_profile();
  for (int i = 0; i < 10; ++i) {
    auto* point = speed_points->Add();
    point->set_s(i * 1.1);
    point->set_t(i * 1.2);
    point->set_v(i * 1.3);
  }

  size_t interval = 5;
  auto* downsampled_points = sim_world_service_->world_.mutable_planning_data()
                                 ->add_st_graph()
                                 ->mutable_speed_profile();
  sim_world_service_->DownsampleSpeedPointsByInterval(*speed_points, interval,
                                                      downsampled_points);

  // Verify
  std::vector<int> kept_index = {0, 5, 9};
  EXPECT_EQ(kept_index.size(), downsampled_points->size());
  for (int i = 0; i < 3; ++i) {
    auto& point = downsampled_points->Get(i);
    EXPECT_DOUBLE_EQ(kept_index[i] * 1.1, point.s());
    EXPECT_DOUBLE_EQ(kept_index[i] * 1.2, point.t());
    EXPECT_DOUBLE_EQ(kept_index[i] * 1.3, point.v());
  }
}

TEST_F(SimulationWorldServiceTest, UpdateLatency) {
  std::shared_ptr<apollo::control::ControlCommand> control_command =
      std::make_shared<apollo::control::ControlCommand>();
  auto* header = control_command->mutable_header();
  header->set_timestamp_sec(2000.9);
  header->set_radar_timestamp(2000 * 1e9);
  header->set_lidar_timestamp(2000.1 * 1e9);
  header->set_camera_timestamp(2000.2 * 1e9);
  control_writer_->Write(control_command);

  BlockerManager::Instance()->Observe();
  sim_world_service_->UpdateLatency(
      "control", sim_world_service_->control_command_reader_.get());

  EXPECT_EQ(1, sim_world_service_->world_.latency_size());
  const Latency latency = sim_world_service_->world_.latency().at("control");
  EXPECT_DOUBLE_EQ(2000.9, latency.timestamp_sec());
  EXPECT_NEAR(0.7 * 1e3, latency.total_time_ms(), kEpsilon);
}

}  // namespace dreamview
}  // namespace apollo
