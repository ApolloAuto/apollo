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

#include "modules/planning/scenarios/stop_sign/unprotected/stage_stop.h"

#include <algorithm>
#include <utility>

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/tasks/deciders/decider_creep.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using common::TrajectoryPoint;
using common::time::Clock;
using hdmap::HDMapUtil;
using hdmap::LaneInfoConstPtr;
using hdmap::PathOverlap;
using hdmap::OverlapInfoConstPtr;
using perception::PerceptionObstacle;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

Stage::StageStatus StageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedPreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  // check if the stop_sign is still along referenceline
  std::string stop_sign_overlap_id = GetContext()->stop_sign_id;
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });
  if (stop_sign_overlap_it == stop_sign_overlaps.end()) {
    return FinishScenario();
  }

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - stop_sign_overlap_it->start_s;
  constexpr double kPassStopLineBuffer = 1.0;  // unit: m

  // passed stop line too far
  if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
    return FinishStage();
  }

  // check on wait-time
  auto start_time = GetContext()->stop_start_time;
  const double wait_time = Clock::NowInSeconds() - start_time;
  ADEBUG << "stop_start_time[" << start_time << "] wait_time[" << wait_time
         << "]";
  if (wait_time < scenario_config_.stop_duration()) {
    return Stage::RUNNING;
  }

  // check on watch_vehicles
  auto& watch_vehicles = GetContext()->watch_vehicles;
  if (watch_vehicles.empty()) {
    return FinishStage();
  }

  // get all vehicles currently watched
  std::vector<std::string> watch_vehicle_ids;
  for (auto it = watch_vehicles.begin(); it != watch_vehicles.end(); ++it) {
    std::copy(it->second.begin(), it->second.end(),
              std::back_inserter(watch_vehicle_ids));
    // for debug
    std::string associated_lane_id = it->first;
    std::string s;
    for (size_t i = 0; i < watch_vehicle_ids.size(); ++i) {
      std::string vehicle = watch_vehicle_ids[i];
      s = s.empty() ? vehicle : s + "," + vehicle;
    }
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }
  if (watch_vehicle_ids.empty()) {
    return FinishStage();
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles.clear();
  std::copy(
      watch_vehicle_ids.begin(), watch_vehicle_ids.end(),
      std::back_inserter(
          PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles));

  // check timeout while waiting for only one vehicle
  if (wait_time > scenario_config_.stop_timeout() &&
      watch_vehicle_ids.size() <= 1) {
    return FinishStage();
  }

  const PathDecision& path_decision = reference_line_info.path_decision();
  RemoveWatchVehicle(path_decision, &watch_vehicles);

  return Stage::RUNNING;
}

/**
 * @brief: remove a watch vehicle which not stopping at stop sign any more
 */
int StageStop::RemoveWatchVehicle(
    const PathDecision& path_decision,
    StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  for (auto it = watch_vehicles->begin(); it != watch_vehicles->end(); ++it) {
    // associated_lane/stop_sign info
    std::string associated_lane_id = it->first;
    auto assoc_lane_it = std::find_if(
        GetContext()->associated_lanes.begin(),
        GetContext()->associated_lanes.end(),
        [&associated_lane_id](
            std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
          return assc_lane.first.get()->id().id() == associated_lane_id;
        });
    if (assoc_lane_it == GetContext()->associated_lanes.end()) {
      continue;
    }
    auto stop_sign_over_lap_info =
        assoc_lane_it->second.get()->GetObjectOverlapInfo(
            hdmap::MakeMapId(associated_lane_id));
    if (stop_sign_over_lap_info == nullptr) {
      AERROR << "can't find stop_sign_over_lap_info for id: "
             << associated_lane_id;
      continue;
    }
    const double stop_line_end_s =
        stop_sign_over_lap_info->lane_overlap_info().end_s();

    const auto lane =
        HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(associated_lane_id));
    if (lane == nullptr) {
      continue;
    }
    auto stop_sign_point = lane.get()->GetSmoothPoint(stop_line_end_s);

    std::vector<std::string> remove_vehicles;
    for (auto obstacle_id : it->second) {
      // watched-vehicle info
      auto *obstacle = path_decision.Find(obstacle_id);
      if (!obstacle) {
        AERROR << "mark ERASE obstacle_id[" << obstacle_id << "] not exist";
        remove_vehicles.push_back(obstacle_id);
        continue;
      }

      const PerceptionObstacle& perception_obstacle = obstacle->Perception();
      PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);
      auto obstacle_point = common::util::MakePointENU(
          perception_obstacle.position().x(),
          perception_obstacle.position().y(),
          perception_obstacle.position().z());

      double distance = common::util::DistanceXY(
          stop_sign_point, obstacle_point);
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] distance[" << distance << "]";

      // TODO(all): move 10.0 to conf
      if (distance > 10.0) {
        ADEBUG << "mark ERASE obstacle_id[" << obstacle_id << "]";
        remove_vehicles.push_back(obstacle_id);
      }
    }
    for (auto obstacle_id : remove_vehicles) {
      ADEBUG << "ERASE obstacle_id[" << obstacle_id << "]";
      it->second.erase(
          std::remove(it->second.begin(), it->second.end(), obstacle_id),
          it->second.end());
    }
  }

  return 0;
}

Stage::StageStatus StageStop::FinishScenario() {
  PlanningContext::GetScenarioInfo()->stop_done_overlap_id = "";
  PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles.clear();

  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

Stage::StageStatus StageStop::FinishStage() {
  PlanningContext::GetScenarioInfo()->stop_done_overlap_id =
      GetContext()->stop_sign_id;
  PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles.clear();
  GetContext()->creep_start_time = Clock::NowInSeconds();

  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_CREEP;
  return Stage::FINISHED;
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
