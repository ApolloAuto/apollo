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

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/point_factory.h"
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

Stage::StageStatus StopSignUnprotectedStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "StopSignUnprotectedPreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  std::string stop_sign_overlap_id = GetContext()->current_stop_sign_overlap_id;

  // refresh overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      scenario::util::GetOverlapOnReferenceLine(reference_line_info,
                                                stop_sign_overlap_id,
                                                ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  static constexpr double kPassStopLineBuffer = 1.0;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - stop_sign_start_s;
  // passed stop line too far
  if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
    return FinishStage();
  }

  // check on wait-time
  auto start_time = GetContext()->stop_start_time;
  const double wait_time = Clock::NowInSeconds() - start_time;
  ADEBUG << "stop_start_time[" << start_time << "] wait_time[" << wait_time
         << "]";
  if (wait_time < scenario_config_.stop_duration_sec()) {
    return Stage::RUNNING;
  }

  // check on watch_vehicles
  auto& watch_vehicles = GetContext()->watch_vehicles;
  if (watch_vehicles.empty()) {
    return FinishStage();
  }

  // get all vehicles currently watched
  std::vector<std::string> watch_vehicle_ids;
  for (const auto& watch_vehicle : watch_vehicles) {
    std::copy(watch_vehicle.second.begin(), watch_vehicle.second.end(),
              std::back_inserter(watch_vehicle_ids));
    // for debug
    std::string s;
    for (const std::string& vehicle : watch_vehicle.second) {
      s = s.empty() ? vehicle : s + "," + vehicle;
    }
    const std::string& associated_lane_id = watch_vehicle.first;
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  // remove duplicates (caused when same vehicle on mutiple lanes)
  watch_vehicle_ids.erase(
      unique(watch_vehicle_ids.begin(), watch_vehicle_ids.end()),
      watch_vehicle_ids.end());

  if (watch_vehicle_ids.empty()) {
    return FinishStage();
  }

  // pass vehicles being watched to DECIDER_RULE_BASED_STOP task
  // for visualization
  for (const auto& perception_obstacle_id : watch_vehicle_ids) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->add_wait_for_obstacle_id(perception_obstacle_id);
  }

  // check timeout while waiting for only one vehicle
  if (wait_time > scenario_config_.stop_timeout_sec() &&
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
int StopSignUnprotectedStageStop::RemoveWatchVehicle(
    const PathDecision& path_decision, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  for (auto& vehicle : *watch_vehicles) {
    // associated_lane/stop_sign info
    std::string associated_lane_id = vehicle.first;
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
    auto& vehicles = vehicle.second;
    for (const auto& perception_obstacle_id : vehicles) {
      // watched-vehicle info
      const PerceptionObstacle* perception_obstacle =
          path_decision.FindPerceptionObstacle(perception_obstacle_id);
      if (!perception_obstacle) {
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id
               << "] not exist";
        remove_vehicles.push_back(perception_obstacle_id);
        continue;
      }

      PerceptionObstacle::Type obstacle_type = perception_obstacle->type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);
      auto obstacle_point = common::util::PointFactory::ToPointENU(
          perception_obstacle->position());

      double distance =
          common::util::DistanceXY(stop_sign_point, obstacle_point);
      ADEBUG << "obstacle_id[" << perception_obstacle_id << "] distance["
             << distance << "]";

      // TODO(all): move 10.0 to conf
      if (distance > 10.0) {
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id << "]";
        remove_vehicles.push_back(perception_obstacle_id);
      }
    }
    for (const auto& perception_obstacle_id : remove_vehicles) {
      ADEBUG << "ERASE obstacle_id[" << perception_obstacle_id << "]";
      vehicles.erase(
          std::remove(vehicles.begin(), vehicles.end(), perception_obstacle_id),
          vehicles.end());
    }
  }

  return 0;
}

Stage::StageStatus StopSignUnprotectedStageStop::FinishStage() {
  // update PlanningContext
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->set_done_stop_sign_overlap_id(
          GetContext()->current_stop_sign_overlap_id);
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->clear_wait_for_obstacle_id();

  GetContext()->creep_start_time = Clock::NowInSeconds();

  next_stage_ = ScenarioConfig::STOP_SIGN_UNPROTECTED_CREEP;
  return Stage::FINISHED;
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
