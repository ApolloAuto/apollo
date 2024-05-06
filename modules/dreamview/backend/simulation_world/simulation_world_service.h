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
 */

#pragma once

#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "gtest/gtest_prod.h"

#include "nlohmann/json.hpp"

#include "modules/common_msgs/audio_msgs/audio.pb.h"
#include "modules/common_msgs/audio_msgs/audio_event.pb.h"
#include "modules/common_msgs/basic_msgs/drive_event.pb.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/external_command_msgs/valet_parking_command.pb.h"
#include "modules/common_msgs/external_command_msgs/action_command.pb.h"
#include "modules/common_msgs/localization_msgs/gps.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"
#include "modules/common_msgs/task_manager_msgs/task_manager.pb.h"
#include "modules/common_msgs/dreamview_msgs/simulation_world.pb.h"

#include "cyber/common/log.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/dreamview/backend/common/map_service/map_service.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimulationWorldService
 * @brief This is a major component of the Simulation backend, which
 * maintains a SimulationWorld object and keeps updating it. The SimulationWorld
 * represents the most up-to-date information about all the objects
 * in the emulated world, including the car, the planning trajectory, etc.
 * NOTE: This class is not thread-safe.
 */
class SimulationWorldService {
 public:
  // The maximum number of monitor message items to be kept in
  // SimulationWorld.
  static constexpr int kMaxMonitorItems = 30;

  /**
   * @brief Constructor of SimulationWorldService.
   * @param map_service the pointer of MapService.
   * @param routing_from_file whether to read initial routing from file.
   */
  SimulationWorldService(const MapService *map_service,
                         bool routing_from_file = false);

  /**
   * @brief Get a read-only view of the SimulationWorld.
   * @return Constant reference to the SimulationWorld object.
   */
  inline const SimulationWorld &world() const { return world_; }

  /**
   * @brief Returns the json representation of the SimulationWorld object.
   *        This is a public API used by offline dreamview.
   * @param radius the search distance from the current car location
   * @return Json object equivalence of the SimulationWorld object.
   */
  nlohmann::json GetUpdateAsJson(double radius) const;

  /**
   * @brief Returns the binary representation of the SimulationWorld object.
   * @param radius the search distance from the current car location.
   * @param sim_world output of binary format sim_world string.
   * @param sim_world_with_planning_data output of binary format sim_world
   * string with planning_data.
   */
  void GetWireFormatString(double radius, std::string *sim_world,
                           std::string *sim_world_with_planning_data);

  /**
   * @brief Returns the json representation of the map element Ids and hash
   * within the given radius from the car.
   * @param radius the search distance from the current car location
   * @return Json object that contains mapElementIds and mapHash.
   */
  nlohmann::json GetMapElements(double radius) const;

  /**
   * @brief The function Update() is periodically called to check for updates
   * from the external messages. All the updates will be written to the
   * SimulationWorld object to reflect the latest status.
   */
  void Update();

  /**
   * @brief Sets the flag to clear the owned simulation world object.
   */
  void SetToClear() { to_clear_ = true; }

  /**
   * @brief Check whether the SimulationWorld object has enough information.
   * The backend won't push the SimulationWorld to frontend if it is not ready.
   * @return True if the object is ready to push.
   */
  bool ReadyToPush() const { return ready_to_push_.load(); }

  /**
   * @brief Publish message to the monitor
   * @param msg the string to send to monitor
   * @param log_level defined in
   *        modules/common/monitor_log/proto/monitor_log.proto
   */
  void PublishMonitorMessage(
      apollo::common::monitor::MonitorMessageItem::LogLevel log_level,
      const std::string &msg);

  void PublishNavigationInfo(
      const std::shared_ptr<apollo::relative_map::NavigationInfo> &);

  void PublishLaneFollowCommand(
      const std::shared_ptr<apollo::external_command::LaneFollowCommand> &);

  void PublishValetParkingCommand(
      const std::shared_ptr<apollo::external_command::ValetParkingCommand> &);

  void PublishActionCommand(
      const std::shared_ptr<apollo::external_command::ActionCommand> &);

  void PublishTask(const std::shared_ptr<apollo::task_manager::Task> &);

  void GetMapElementIds(double radius, MapElementIds *ids) const;

  const apollo::hdmap::Map &GetRelativeMap() const;

  nlohmann::json GetRoutePathAsJson() const;

  void DumpMessages();

 private:
  void InitReaders();
  void InitWriters();

  /**
   * @brief Update simulation world with incoming data, e.g., chassis,
   * localization, planning, perception, etc.
   */
  template <typename DataType>
  void UpdateSimulationWorld(const DataType &data);

  void UpdateMonitorMessages();

  Object &CreateWorldObjectIfAbsent(
      const apollo::perception::PerceptionObstacle &obstacle);
  void CreateWorldObjectFromSensorMeasurement(
      const apollo::perception::SensorMeasurement &sensor,
      Object *world_object);
  void SetObstacleInfo(const apollo::perception::PerceptionObstacle &obstacle,
                       Object *world_object);
  void SetObstaclePolygon(
      const apollo::perception::PerceptionObstacle &obstacle,
      Object *world_object);
  void SetObstacleSensorMeasurements(
      const apollo::perception::PerceptionObstacle &obstacle,
      Object *world_object);
  void SetObstacleSource(const apollo::perception::PerceptionObstacle &obstacle,
                         Object *world_object);
  void UpdatePlanningTrajectory(
      const apollo::planning::ADCTrajectory &trajectory);
  void UpdateRSSInfo(const apollo::planning::ADCTrajectory &trajectory);
  bool LocateMarker(const apollo::planning::ObjectDecisionType &decision,
                    Decision *world_decision);
  void FindNudgeRegion(const apollo::planning::ObjectDecisionType &decision,
                       const Object &world_obj, Decision *world_decision);
  void UpdateDecision(const apollo::planning::DecisionResult &decision_res,
                      double header_time);
  void UpdateMainStopDecision(
      const apollo::planning::MainDecision &main_decision,
      double update_timestamp_sec, Object *world_main_stop);

  template <typename MainDecision>
  void UpdateMainChangeLaneDecision(const MainDecision &decision,
                                    Object *world_main_decision) {
    if (decision.has_change_lane_type() &&
        (decision.change_lane_type() == apollo::routing::ChangeLaneType::LEFT ||
         decision.change_lane_type() ==
             apollo::routing::ChangeLaneType::RIGHT)) {
      auto *change_lane_decision = world_main_decision->add_decision();
      change_lane_decision->set_change_lane_type(decision.change_lane_type());

      const auto &adc = world_.auto_driving_car();
      change_lane_decision->set_position_x(adc.position_x());
      change_lane_decision->set_position_y(adc.position_y());
      change_lane_decision->set_heading(adc.heading());
    }
  }

  void CreatePredictionTrajectory(
      const apollo::prediction::PredictionObstacle &obstacle,
      Object *world_object);

  void DownsamplePath(const apollo::common::Path &paths,
                      apollo::common::Path *downsampled_path);

  void UpdatePlanningData(const apollo::planning_internal::PlanningData &data);

  void PopulateMapInfo(double radius);

  /**
   * @brief Get the latest observed data from reader to update the
   * SimulationWorld object when triggered by refresh timer.
   */
  template <typename MessageT>
  void UpdateWithLatestObserved(cyber::Reader<MessageT> *reader,
                                bool logging = true) {
    if (reader->Empty()) {
      if (logging) {
        AINFO_EVERY(100) << "Has not received any data from "
                         << reader->GetChannelName();
      }
      return;
    }

    const std::shared_ptr<MessageT> msg = reader->GetLatestObserved();
    UpdateSimulationWorld(*msg);
  }

  /**
   * @brief Get the latest observed data from reader and dump it to a local
   * file.
   */
  template <typename MessageT>
  void DumpMessageFromReader(cyber::Reader<MessageT> *reader) {
    if (reader->Empty()) {
      AWARN << "Has not received any data from " << reader->GetChannelName()
            << ". Cannot dump message!";
      return;
    }

    apollo::common::util::DumpMessage(reader->GetLatestObserved());
  }

  void ReadPlanningCommandFromFile(const std::string &planning_command_file);

  template <typename MessageT>
  void UpdateLatency(const std::string &module_name,
                     cyber::Reader<MessageT> *reader) {
    if (reader->Empty()) {
      return;
    }

    const auto header = reader->GetLatestObserved()->header();
    const double publish_time_sec = header.timestamp_sec();
    const double sensor_time_sec =
        apollo::cyber::Time(
            std::max({header.lidar_timestamp(), header.camera_timestamp(),
                      header.radar_timestamp()}))
            .ToSecond();

    Latency latency;
    latency.set_timestamp_sec(publish_time_sec);
    latency.set_total_time_ms((publish_time_sec - sensor_time_sec) * 1.0e3);
    (*world_.mutable_latency())[module_name] = latency;
  }

  /**
   * @brief update delayes of modules.
   * @detail Delay is calculated based on the received time from a module
   * reader. If the reader has not received any message, delay is -1. Otherwise,
   * it is the max of (current_time - last_received_time) and
   * (last_received_time - second_to_last_received_time)
   */
  void UpdateDelays();

  /**
   * @brief update latencies of modules, where latency is how long it takes for
   * sensor data (lidar, radar and/or camera) to be processed by
   * a module.
   */
  void UpdateLatencies();

  template <typename Points>
  void DownsampleSpeedPointsByInterval(const Points &points,
                                       size_t downsampleInterval,
                                       Points *downsampled_points) {
    if (points.empty()) {
      return;
    }

    for (int i = 0; i + 1 < points.size(); i += downsampleInterval) {
      *downsampled_points->Add() = points[i];
    }

    // add the last point
    *downsampled_points->Add() = *points.rbegin();
  }

  std::unique_ptr<cyber::Node> node_;

  // The underlying SimulationWorld object, owned by the
  // SimulationWorldService instance.
  SimulationWorld world_;

  // Downsampled route paths to be rendered in frontend.
  mutable boost::shared_mutex route_paths_mutex_;
  std::vector<RoutePath> route_paths_;

  // The handle of MapService, not owned by SimulationWorldService.
  const MapService *map_service_;

  // The map holding obstacle string id to the actual object
  std::unordered_map<std::string, Object> obj_map_;

  // A temporary cache for all the monitor messages coming in.
  std::mutex monitor_msgs_mutex_;
  std::list<std::shared_ptr<common::monitor::MonitorMessage>> monitor_msgs_;

  // The SIMULATOR monitor for publishing messages.
  apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  // Whether to clear the SimulationWorld in the next timer cycle, set by
  // frontend request.
  bool to_clear_ = false;

  // Relative map used/retrieved in navigation mode
  apollo::hdmap::Map relative_map_;

  // Whether the sim_world is ready to push to frontend
  std::atomic<bool> ready_to_push_;

  // Latest rss info
  double current_real_dist_ = 0.0;
  double current_rss_safe_dist_ = 0.0;

  // Gear Location
  apollo::canbus::Chassis_GearPosition gear_location_;

  // Readers.
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::Gps>> gps_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::perception::PerceptionObstacles>>
      perception_obstacle_reader_;
  std::shared_ptr<cyber::Reader<apollo::perception::TrafficLightDetection>>
      perception_traffic_light_reader_;
  std::shared_ptr<cyber::Reader<apollo::prediction::PredictionObstacles>>
      prediction_obstacle_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      planning_reader_;
  std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>>
      control_command_reader_;
  std::shared_ptr<cyber::Reader<apollo::relative_map::NavigationInfo>>
      navigation_reader_;
  std::shared_ptr<cyber::Reader<apollo::relative_map::MapMsg>>
      relative_map_reader_;
  std::shared_ptr<cyber::Reader<apollo::audio::AudioEvent>> audio_event_reader_;
  std::shared_ptr<cyber::Reader<apollo::common::DriveEvent>>
      drive_event_reader_;
  std::shared_ptr<cyber::Reader<apollo::common::monitor::MonitorMessage>>
      monitor_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::PlanningCommand>>
      planning_command_reader_;
  std::shared_ptr<cyber::Reader<apollo::storytelling::Stories>>
      storytelling_reader_;
  std::shared_ptr<cyber::Reader<apollo::audio::AudioDetection>>
      audio_detection_reader_;
  std::shared_ptr<cyber::Reader<apollo::task_manager::Task>> task_reader_;

  // Writers.
  std::shared_ptr<cyber::Writer<apollo::relative_map::NavigationInfo>>
      navigation_writer_;
  std::shared_ptr<cyber::Client<apollo::external_command::LaneFollowCommand,
                                apollo::external_command::CommandStatus>>
      lane_follow_command_client_;
  std::shared_ptr<cyber::Client<apollo::external_command::ValetParkingCommand,
                                apollo::external_command::CommandStatus>>
      valet_parking_command_client_;
  std::shared_ptr<cyber::Client<apollo::external_command::ActionCommand,
                                apollo::external_command::CommandStatus>>
      action_command_client_;
  std::shared_ptr<cyber::Writer<apollo::routing::RoutingResponse>>
      routing_response_writer_;
  std::shared_ptr<cyber::Writer<apollo::task_manager::Task>> task_writer_;

  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorSuccess);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorRemove);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorTruncate);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateChassisInfo);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateLatency);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateLocalization);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePerceptionObstacles);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePlanningTrajectory);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateDecision);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePrediction);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateRouting);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateGps);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateControlCommandWithSimpleLonLat);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateControlCommandWithSimpleMpc);
  FRIEND_TEST(SimulationWorldServiceTest, DownsampleSpeedPointsByInterval);
};

}  // namespace dreamview
}  // namespace apollo
