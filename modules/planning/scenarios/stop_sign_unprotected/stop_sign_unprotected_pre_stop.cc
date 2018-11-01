/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected_pre_stop.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/toolkits/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign_protected {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::HDMapUtil;
using hdmap::LaneInfo;
using hdmap::LaneInfoConstPtr;
using hdmap::OverlapInfoConstPtr;
using perception::PerceptionObstacle;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

Stage::StageStatus StopSignUnprotectedPreStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: PreStop";
  CHECK_NOTNULL(frame);

  const auto& reference_line_info = frame->reference_line_info().front();
  if (CheckADCStop(reference_line_info)) {
    GetContext()->stop_start_time = Clock::NowInSeconds();

    next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_STOP;
    return Stage::FINISHED;
  }

  const PathDecision& path_decision = reference_line_info.path_decision();
  auto& watch_vehicles = GetContext()->watch_vehicles;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    // add to watch_vehicles if adc is still proceeding to stop sign
    AddWatchVehicle(*obstacle, &watch_vehicles);
  }

  bool plan_ok = PlanningOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedPreStop planning error";
  }
  return Stage::RUNNING;
}

/**
 * @brief: add a watch vehicle which arrives at stop sign ahead of adc
 */
int StopSignUnprotectedPreStop::AddWatchVehicle(
    const Obstacle& obstacle, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  const std::string& obstacle_id = std::to_string(perception_obstacle.id());
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);

  // check type
  if (obstacle_type != PerceptionObstacle::UNKNOWN &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::VEHICLE) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]. skip";
    return 0;
  }

  auto point = common::util::MakePointENU(perception_obstacle.position().x(),
                                          perception_obstacle.position().y(),
                                          perception_obstacle.position().z());
  double obstacle_s = 0.0;
  double obstacle_l = 0.0;
  hdmap::LaneInfoConstPtr obstacle_lane;
  if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
          point, 5.0, perception_obstacle.theta(), M_PI / 3.0, &obstacle_lane,
          &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]: Failed to find nearest lane from map for position: "
           << point.DebugString()
           << "; heading:" << perception_obstacle.theta();
    return -1;
  }

  // check obstacle is on an associate lane guarded by stop sign
  std::string obstable_lane_id = obstacle_lane.get()->id().id();
  auto assoc_lane_it = std::find_if(
      GetContext()->associated_lanes.begin(),
      GetContext()->associated_lanes.end(),
      [&obstable_lane_id](
          std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
        return assc_lane.first.get()->id().id() == obstable_lane_id;
      });
  if (assoc_lane_it == GetContext()->associated_lanes.end()) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "] lane_id[" << obstable_lane_id
           << "] not associated with current stop_sign. skip";
    return -1;
  }

  /* skip the speed check to make it less strick
  auto speed = std::hypot(perception_obstacle.velocity().x(),
                          perception_obstacle.velocity().y());
  if (speed > config_.stop_sign().watch_vehicle_max_valid_stop_speed()) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "] velocity[" << speed << "] not stopped. skip";
    return -1;
  }
  */

  // check a valid stop for stop line of the stop_sign
  auto over_lap_info = assoc_lane_it->second.get()->GetObjectOverlapInfo(
      obstacle_lane.get()->id());
  if (over_lap_info == nullptr) {
    AERROR << "can't find over_lap_info for id: " << obstable_lane_id;
    return -1;
  }
  double stop_line_s = over_lap_info->lane_overlap_info().start_s();
  double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
  double distance_to_stop_line = stop_line_s - obstacle_end_s;
  if (distance_to_stop_line > conf_watch_vehicle_max_valid_stop_distance_) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "] distance_to_stop_line[" << distance_to_stop_line
           << "]; stop_line_s" << stop_line_s << "]; obstacle_end_s["
           << obstacle_end_s << "] too far from stop line. skip";
    return -1;
  }

  // use a vector since motocycles/bicycles can be more than one
  std::vector<std::string> vehicles =
      (*watch_vehicles)[obstacle_lane->id().id()];
  if (std::find(vehicles.begin(), vehicles.end(), obstacle_id) ==
      vehicles.end()) {
    ADEBUG << "AddWatchVehicle: lane[" << obstacle_lane->id().id()
           << "] obstacle_id[" << obstacle_id << "]";
    (*watch_vehicles)[obstacle_lane->id().id()].push_back(obstacle_id);
  }

  return 0;
}

/**
 * @brief: check valid stop_sign stop
 */
bool StopSignUnprotectedPreStop::CheckADCStop(
    const ReferenceLineInfo& reference_line_info) {
  double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  if (adc_speed > conf_max_adc_stop_speed_) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  double stop_line_start_s =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.start_s;
  double distance_stop_line_to_adc_front_edge =
      stop_line_start_s - adc_front_edge_s;
  ADEBUG << "distance_stop_line_to_adc_front_edge["
         << distance_stop_line_to_adc_front_edge << "]; stop_line_start_s["
         << stop_line_start_s << "]; adc_front_edge_s[" << adc_front_edge_s
         << "]";

  if (distance_stop_line_to_adc_front_edge > conf_max_valid_stop_distance_) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // TODO(all): check no BICYCLE in between.

  return true;
}

}  // namespace stop_sign_protected
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
