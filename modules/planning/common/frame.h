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

#pragma once

#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/localization_msgs/pose.pb.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/open_space_info.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

namespace apollo {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);

  Frame(uint32_t sequence_num, const LocalView &local_view,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state,
        ReferenceLineProvider *reference_line_provider);

  Frame(uint32_t sequence_num, const LocalView &local_view,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state);

  virtual ~Frame() = default;

  const common::TrajectoryPoint &PlanningStartPoint() const;

  common::Status Init(
      const common::VehicleStateProvider *vehicle_state_provider,
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints,
      const EgoInfo *ego_info);

  common::Status InitForOpenSpace(
      const common::VehicleStateProvider *vehicle_state_provider,
      const EgoInfo *ego_info);

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  const std::list<ReferenceLineInfo> &reference_line_info() const;
  std::list<ReferenceLineInfo> *mutable_reference_line_info();

  Obstacle *Find(const std::string &id);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();

  const ReferenceLineInfo *FindTargetReferenceLineInfo();

  const ReferenceLineInfo *FindFailedReferenceLineInfo();

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

  bool Rerouting(PlanningContext *planning_context);

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

  void set_current_frame_planned_trajectory(
      ADCTrajectory current_frame_planned_trajectory) {
    current_frame_planned_trajectory_ =
        std::move(current_frame_planned_trajectory);
  }

  const ADCTrajectory &current_frame_planned_trajectory() const {
    return current_frame_planned_trajectory_;
  }

  void set_current_frame_planned_path(
      DiscretizedPath current_frame_planned_path) {
    current_frame_planned_path_ = std::move(current_frame_planned_path);
  }

  const DiscretizedPath &current_frame_planned_path() const {
    return current_frame_planned_path_;
  }

  const bool is_near_destination() const {
    return is_near_destination_;
  }

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */
  void UpdateReferenceLinePriority(
      const std::map<std::string, uint32_t> &id_to_priority);

  const LocalView &local_view() const {
    return local_view_;
  }

  ThreadSafeIndexedObstacles *GetObstacleList() {
    return &obstacles_;
  }

  const OpenSpaceInfo &open_space_info() const {
    return open_space_info_;
  }

  OpenSpaceInfo *mutable_open_space_info() {
    return &open_space_info_;
  }

  perception::TrafficLight GetSignal(const std::string &traffic_light_id) const;

  const PadMessage::DrivingAction &GetPadMsgDrivingAction() const {
    return pad_msg_driving_action_;
  }

 private:
  common::Status InitFrameData(
      const common::VehicleStateProvider *vehicle_state_provider,
      const EgoInfo *ego_info);

  bool CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines,
                               const std::list<hdmap::RouteSegments> &segments);

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle(const EgoInfo *ego_info) const;

  /**
   * @brief create a static virtual obstacle
   */
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);

  void AddObstacle(const Obstacle &obstacle);

  void ReadTrafficLights();

  void ReadPadMsgDrivingAction();
  void ResetPadMsgDrivingAction();

 private:
  static PadMessage::DrivingAction pad_msg_driving_action_;
  uint32_t sequence_num_ = 0;
  LocalView local_view_;
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;

  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;

  std::unordered_map<std::string, const perception::TrafficLight *>
      traffic_lights_;

  // current frame published trajectory
  ADCTrajectory current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
  DiscretizedPath current_frame_planned_path_;

  const ReferenceLineProvider *reference_line_provider_ = nullptr;

  OpenSpaceInfo open_space_info_;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 public:
  FrameHistory();
};

}  // namespace planning
}  // namespace apollo
