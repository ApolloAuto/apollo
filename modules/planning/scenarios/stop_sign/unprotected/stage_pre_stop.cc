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

#include "modules/planning/scenarios/stop_sign/unprotected/stage_pre_stop.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using apollo::common::TrajectoryPoint;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::OverlapInfoConstPtr;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

Stage::StageStatus StopSignUnprotectedStagePreStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: PreStop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedStagePreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // get overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  static constexpr double kPassStopLineBuffer = 0.3;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - current_stop_sign_overlap->start_s;
  if (distance_adc_pass_stop_sign <= kPassStopLineBuffer) {
    // not passed stop line, check valid stop
    if (CheckADCStop(adc_front_edge_s, current_stop_sign_overlap->start_s)) {
      return FinishStage();
    }
  } else {
    // passed stop line
    return FinishStage();
  }

  // PRE-STOP
  const PathDecision& path_decision = reference_line_info.path_decision();
  auto& watch_vehicles = GetContext()->watch_vehicles;

  std::vector<std::string> watch_vehicle_ids;
  for (const auto& vehicle : watch_vehicles) {
    std::copy(vehicle.second.begin(), vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids));

    // for debug string
    std::string associated_lane_id = vehicle.first;
    std::string s;
    for (const std::string& vehicle_id : vehicle.second) {
      s = s.empty() ? vehicle_id : s + "," + vehicle_id;
    }
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }

  for (const auto* obstacle : path_decision.obstacles().Items()) {
    // add to watch_vehicles if adc is still proceeding to stop sign
    AddWatchVehicle(*obstacle, &watch_vehicles);
  }

  return Stage::RUNNING;
}

/**
 * @brief: add a watch vehicle which arrives at stop sign ahead of adc
 */
int StopSignUnprotectedStagePreStop::AddWatchVehicle(
    const Obstacle& obstacle, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  const PerceptionObstacle& perception_obstacle = obstacle.Perception();
  const std::string& perception_obstacle_id =
      std::to_string(perception_obstacle.id());
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);

  // check type
  if (obstacle_type != PerceptionObstacle::UNKNOWN &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::VEHICLE) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id
           << "] type[" << obstacle_type_name << "]. skip";
    return 0;
  }

  const auto point =
      common::util::PointFactory::ToPointENU(perception_obstacle.position());
  double obstacle_s = 0.0;
  double obstacle_l = 0.0;
  LaneInfoConstPtr obstacle_lane;
  if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
          point, 5.0, perception_obstacle.theta(), M_PI / 3.0, &obstacle_lane,
          &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id
           << "] type[" << obstacle_type_name
           << "]: Failed to find nearest lane from map for position: "
           << point.DebugString()
           << "; heading[" << perception_obstacle.theta() << "]";
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
    ADEBUG << "obstacle_id[" << perception_obstacle_id
           << "] type[" << obstacle_type_name
           << "] lane_id[" << obstable_lane_id
           << "] not associated with current stop_sign. skip";
    return -1;
  }

  // check a valid stop for stop line of the stop_sign
  auto over_lap_info = assoc_lane_it->second.get()->GetObjectOverlapInfo(
      obstacle_lane.get()->id());
  if (over_lap_info == nullptr) {
    AERROR << "can't find over_lap_info for id: " << obstable_lane_id;
    return -1;
  }
  const double stop_line_s = over_lap_info->lane_overlap_info().start_s();
  const double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
  const double distance_to_stop_line = stop_line_s - obstacle_end_s;

  if (distance_to_stop_line >
      scenario_config_.watch_vehicle_max_valid_stop_distance()) {
    ADEBUG << "obstacle_id[" << perception_obstacle_id
           << "] type[" << obstacle_type_name
           << "] distance_to_stop_line[" << distance_to_stop_line
           << "]; stop_line_s" << stop_line_s << "]; obstacle_end_s["
           << obstacle_end_s << "] too far from stop line. skip";
    return -1;
  }

  // use a vector since motocycles/bicycles can be more than one
  std::vector<std::string> vehicles =
      (*watch_vehicles)[obstacle_lane->id().id()];
  if (std::find(vehicles.begin(), vehicles.end(), perception_obstacle_id) ==
      vehicles.end()) {
    ADEBUG << "AddWatchVehicle: lane[" << obstacle_lane->id().id()
           << "] obstacle_id[" << perception_obstacle_id << "]";
    (*watch_vehicles)[obstacle_lane->id().id()].push_back(
        perception_obstacle_id);
  }

  return 0;
}

/**
 * @brief: check valid stop_sign stop
 */
bool StopSignUnprotectedStagePreStop::CheckADCStop(
    const double adc_front_edge_s, const double stop_line_s) {
  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed > max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  const double distance_stop_line_to_adc_front_edge =
      stop_line_s - adc_front_edge_s;
  ADEBUG << "distance_stop_line_to_adc_front_edge["
         << distance_stop_line_to_adc_front_edge << "]";

  if (distance_stop_line_to_adc_front_edge >
      scenario_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // TODO(all): check no BICYCLE in between.

  return true;
}

Stage::StageStatus StopSignUnprotectedStagePreStop::FinishStage() {
  GetContext()->stop_start_time = Clock::NowInSeconds();
  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_STOP;

  return Stage::FINISHED;
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
