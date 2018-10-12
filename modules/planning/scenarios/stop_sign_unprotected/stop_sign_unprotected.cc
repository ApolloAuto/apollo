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

#include <algorithm>
#include <limits>

#include "modules/planning/scenarios/stop_sign_unprotected/stop_sign_unprotected.h"  // NOINT

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "cybertron/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/toolkits/deciders/decider_creep.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"

namespace apollo {
namespace planning {

using common::ErrorCode;
using common::Status;
using common::time::Clock;
using common::TrajectoryPoint;
using hdmap::HDMapUtil;
using hdmap::PathOverlap;
using hdmap::StopSignInfo;
using hdmap::StopSignInfoConstPtr;
using hdmap::LaneInfo;
using hdmap::LaneInfoConstPtr;
using hdmap::OverlapInfoConstPtr;
using hdmap::PathOverlap;
using hdmap::StopSignInfo;
using hdmap::StopSignInfoConstPtr;
using perception::PerceptionObstacle;

void StopSignUnprotectedScenario::RegisterTasks() {
  // deciders
  task_factory_.Register(DECIDER_CREEP,
                         []() -> Task* { return new DeciderCreep(); });
  // optimizers
  task_factory_.Register(DP_POLY_PATH_OPTIMIZER,
                         []() -> Task* { return new DpPolyPathOptimizer(); });
  task_factory_.Register(PATH_DECIDER,
                         []() -> Task* { return new PathDecider(); });
  task_factory_.Register(DP_ST_SPEED_OPTIMIZER,
                         []() -> Task* { return new DpStSpeedOptimizer(); });
  task_factory_.Register(SPEED_DECIDER,
                         []() -> Task* { return new SpeedDecider(); });
  task_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, []() -> Task* {
    return new QpSplineStSpeedOptimizer();
  });
}

bool StopSignUnprotectedScenario::Init() {
  if (is_init_) {
    return true;
  }

  CHECK(apollo::common::util::GetProtoFromFile(
      FLAGS_scenario_stop_sign_unprotected_config_file, &config_));

  RegisterTasks();

  is_init_ = true;
  return true;
}

void StopSignUnprotectedScenario::Observe(Frame* const frame) {
  CHECK_NOTNULL(frame);

  const auto& reference_line_info = frame->reference_line_info().front();

  if (!FindNextStopSign(reference_line_info)) {
    ADEBUG << "no stop sign found";
    return;
  }

  GetAssociatedLanes(*next_stop_sign_);

  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  adc_distance_to_stop_sign_ =
      next_stop_sign_overlap_.start_s - adc_front_edge_s;
}

Status StopSignUnprotectedScenario::Process(
    const TrajectoryPoint& planning_start_point,
    Frame* frame) {
  CHECK_NOTNULL(frame);

  if (!stage_init_) {
    const int current_stage_index = StageIndexInConf(stage_);
    if (!InitTasks(config_, current_stage_index, &tasks_)) {
      return Status(ErrorCode::PLANNING_ERROR, "failed to init tasks");
    }
    stage_init_ = true;
  }

  // init while scenario just entered
  if (scenario_status_ == ScenarioStatus::STATUS_UNKNOWN ||
      scenario_status_ == ScenarioStatus::STATUS_START) {
    watch_vehicles_.clear();
  }
  scenario_status_ = ScenarioStatus::STATUS_PROCESSING;

  const auto& reference_line_info = frame->reference_line_info().front();

  Status status = Status(ErrorCode::PLANNING_ERROR,
                         "Failed to process stage in stop_sign_upprotected.");
  switch (stage_) {
    case StopSignUnprotectedStage::PRE_STOP: {
      status = PreStop(reference_line_info, frame);
      break;
    }
    case StopSignUnprotectedStage::STOP: {
      status = Stop(reference_line_info, frame);
      break;
    }
    case StopSignUnprotectedStage::CREEP: {
      status = Creep(reference_line_info, frame);
      break;
    }
    case StopSignUnprotectedStage::INTERSECTION_CRUISE: {
      status = IntersectionCruise(
          planning_start_point, reference_line_info, frame);
      break;
    }
    default:
      break;
  }

  return Status::OK();
}

bool StopSignUnprotectedScenario::IsTransferable(
    const Scenario& current_scenario,
    const common::TrajectoryPoint& ego_point,
    const Frame& frame) const {
  if (next_stop_sign_ == nullptr) {
    return false;
  }

  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  const uint32_t time_distance = ceil(adc_distance_to_stop_sign_ / adc_speed);

  switch (current_scenario.scenario_type()) {
    case ScenarioConfig::LANE_FOLLOW:
    case ScenarioConfig::CHANGE_LANE:
    case ScenarioConfig::SIDE_PASS:
    case ScenarioConfig::APPROACH:
      return time_distance <= conf_start_stop_sign_timer ? true : false;
    case ScenarioConfig::STOP_SIGN_PROTECTED:
      return false;
    case ScenarioConfig::STOP_SIGN_UNPROTECTED:
      return true;
    case ScenarioConfig::TRAFFIC_LIGHT_LEFT_TURN_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_LEFT_TURN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_PROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED:
    case ScenarioConfig::TRAFFIC_LIGHT_GO_THROUGH:
    default:
      break;
  }

  return false;
}

int StopSignUnprotectedScenario::StageIndexInConf(
    const StopSignUnprotectedStage& stage) {
  // note: this is the index in scenario conf file.  must be consistent
  if (stage == StopSignUnprotectedStage::CREEP) {
    return 0;
  } else if (stage == StopSignUnprotectedStage::INTERSECTION_CRUISE) {
    return 1;
  }
  return -1;
}

StopSignUnprotectedScenario::StopSignUnprotectedStage
    StopSignUnprotectedScenario::GetNextStage(
        const StopSignUnprotectedStage& current_stage) {
  StopSignUnprotectedStage stage = current_stage;
  StopSignUnprotectedStage next_stage;

  bool next_stage_found = false;
  while (!next_stage_found) {
    if (stage == StopSignUnprotectedStage::PRE_STOP) {
      next_stage = StopSignUnprotectedStage::STOP;
      const int next_stage_index = StageIndexInConf(next_stage);
      if (config_.stage(next_stage_index).enabled()) {
        next_stage_found = true;
        break;
      }
      stage = StopSignUnprotectedStage::STOP;
    } else if (stage == StopSignUnprotectedStage::STOP) {
      next_stage = StopSignUnprotectedStage::CREEP;
      const int next_stage_index = StageIndexInConf(next_stage);
      if (config_.stage(next_stage_index).enabled()) {
        next_stage_found = true;
        break;
      }
      stage = StopSignUnprotectedStage::CREEP;
    } else if (stage == StopSignUnprotectedStage::CREEP) {
      next_stage = StopSignUnprotectedStage::INTERSECTION_CRUISE;
      const int next_stage_index = StageIndexInConf(next_stage);
      if (config_.stage(next_stage_index).enabled()) {
        next_stage_found = true;
      }
      break;  // exit at last stage
    }
  }

  return next_stage_found ? next_stage : StopSignUnprotectedStage::UNKNOWN;
}

common::Status StopSignUnprotectedScenario::PreStop(
    const ReferenceLineInfo& reference_line_info,
    Frame* frame) {
  CHECK_NOTNULL(frame);

  if (CheckADCStop(reference_line_info)) {
    stop_start_time_ = Clock::NowInSeconds();
    ADEBUG << "stop_start_time[" << stop_start_time_ << "]";

    stage_ = GetNextStage(stage_);
    stage_init_ = false;
    return Status::OK();
  }

  const PathDecision& path_decision = reference_line_info.path_decision();
  for (const auto* path_obstacle :
       path_decision.path_obstacles().Items()) {
    // add to watch_vehicles if adc is still proceeding to stop sign
    AddWatchVehicle(*path_obstacle, &watch_vehicles_);
  }

  // TODO(all): call decider to add stop fence

  return Status::OK();
}

common::Status StopSignUnprotectedScenario::Stop(
    const ReferenceLineInfo& reference_line_info,
    Frame* frame) {
  CHECK_NOTNULL(frame);

  double wait_time = Clock::NowInSeconds() - stop_start_time_;
  ADEBUG << "stop_start_time[" << stop_start_time_
         << "] wait_time[" << wait_time << "]";
  if (wait_time >= conf_stop_duration && watch_vehicles_.empty()) {
    stage_ = GetNextStage(stage_);
    stage_init_ = false;
    return Status::OK();
  }

  // get all vehicles currently watched
  std::vector<std::string> watch_vehicle_ids;
  for (StopSignLaneVehicles::iterator it = watch_vehicles_.begin();
       it != watch_vehicles_.end(); ++it) {
    std::copy(it->second.begin(), it->second.end(),
              std::back_inserter(watch_vehicle_ids));
  }

  const PathDecision& path_decision = reference_line_info.path_decision();
  for (const auto* path_obstacle :
       path_decision.path_obstacles().Items()) {
    // remove from watch_vehicles_ if adc is stopping/waiting at stop sign
    RemoveWatchVehicle(*path_obstacle, watch_vehicle_ids, &watch_vehicles_);
  }

  // TODO(all):
  return Status::OK();
}

common::Status StopSignUnprotectedScenario::Creep(
    const ReferenceLineInfo& reference_line_info,
    Frame* frame) {
  // TODO(all)
  return Status::OK();
}

common::Status StopSignUnprotectedScenario::IntersectionCruise(
    const common::TrajectoryPoint& planning_start_point,
    const ReferenceLineInfo& reference_line_info,
    Frame* frame) {
  // TODO(all)
  return Status::OK();
}

/**
 * @brief: fine next stop sign ahead of adc along reference line
 */
bool StopSignUnprotectedScenario::FindNextStopSign(
    const ReferenceLineInfo& reference_line_info) {
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  double min_start_s = std::numeric_limits<double>::max();
  for (const PathOverlap& stop_sign_overlap : stop_sign_overlaps) {
    if (adc_front_edge_s - stop_sign_overlap.end_s <=
        conf_min_pass_s_distance &&
        stop_sign_overlap.start_s < min_start_s) {
      min_start_s = stop_sign_overlap.start_s;
      next_stop_sign_overlap_ = stop_sign_overlap;
    }
  }

  if (next_stop_sign_overlap_.object_id.empty()) {
    return false;
  }

  next_stop_sign_ = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(next_stop_sign_overlap_.object_id));
  if (!next_stop_sign_) {
    AERROR << "Could not find stop sign: " << next_stop_sign_overlap_.object_id;
    return false;
  }

  return true;
}

/*
 * get all the lanes associated/guarded by a stop sign
 */
int StopSignUnprotectedScenario::GetAssociatedLanes(
    const StopSignInfo& stop_sign_info) {
  associated_lanes_.clear();

  std::vector<StopSignInfoConstPtr> associated_stop_signs;
  HDMapUtil::BaseMap().GetStopSignAssociatedStopSigns(
      stop_sign_info.id(), &associated_stop_signs);

  for (const auto stop_sign : associated_stop_signs) {
    if (stop_sign == nullptr) {
      continue;
    }

    const auto& associated_lane_ids = stop_sign->OverlapLaneIds();
    for (const auto& lane_id : associated_lane_ids) {
      const auto lane = HDMapUtil::BaseMap().GetLaneById(lane_id);
      if (lane == nullptr) {
        continue;
      }
      const auto& stop_sign_overlaps = lane->stop_signs();
      for (auto stop_sign_overlap : stop_sign_overlaps) {
        auto over_lap_info =
            stop_sign_overlap->GetObjectOverlapInfo(stop_sign.get()->id());
        if (over_lap_info != nullptr) {
          associated_lanes_.push_back(std::make_pair(lane, stop_sign_overlap));
          ADEBUG << "stop_sign: " << stop_sign_info.id().id()
                 << "; associated_lane: " << lane_id.id()
                 << "; associated_stop_sign: " << stop_sign.get()->id().id();
        }
      }
    }
  }

  return 0;
}

/**
 * @brief: add a watch vehicle which arrives at stop sign ahead of adc
 */
int StopSignUnprotectedScenario::AddWatchVehicle(
    const PathObstacle& path_obstacle,
    StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  const PerceptionObstacle& perception_obstacle =
      path_obstacle.obstacle()->Perception();
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
      associated_lanes_.begin(), associated_lanes_.end(),
      [&obstable_lane_id](
          std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
        return assc_lane.first.get()->id().id() == obstable_lane_id;
      });
  if (assoc_lane_it == associated_lanes_.end()) {
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
  if (distance_to_stop_line > conf_watch_vehicle_max_valid_stop_distance) {
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
 * @brief: remove a watch vehicle which not stopping at stop sign any more
 */
int StopSignUnprotectedScenario::RemoveWatchVehicle(
    const PathObstacle& path_obstacle,
    const std::vector<std::string>& watch_vehicle_ids,
    StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  const PerceptionObstacle& perception_obstacle =
      path_obstacle.obstacle()->Perception();
  const std::string& obstacle_id = std::to_string(perception_obstacle.id());
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);

  // check if being watched
  if (std::find(watch_vehicle_ids.begin(), watch_vehicle_ids.end(),
                obstacle_id) == watch_vehicle_ids.end()) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "] not being watched. skip";
    return 0;
  }

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
  LaneInfoConstPtr obstacle_lane;
  if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
          point, 5.0, perception_obstacle.theta(), M_PI / 3.0, &obstacle_lane,
          &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]: Failed to find nearest lane from map for position: "
           << point.DebugString()
           << "; heading:" << perception_obstacle.theta();
    return -1;
  }

  bool erase = false;

  bool is_path_cross = !path_obstacle.reference_line_st_boundary().IsEmpty();

  // check obstacle is on an associate lane guarded by stop sign
  const std::string& obstable_lane_id = obstacle_lane.get()->id().id();
  auto assoc_lane_it = std::find_if(
      associated_lanes_.begin(), associated_lanes_.end(),
      [&obstable_lane_id](
          std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
        return assc_lane.first.get()->id().id() == obstable_lane_id;
      });
  if (assoc_lane_it != associated_lanes_.end()) {
    // check pass stop line of the stop_sign
    auto over_lap_info = assoc_lane_it->second.get()->GetObjectOverlapInfo(
        obstacle_lane.get()->id());
    if (over_lap_info == nullptr) {
      AERROR << "can't find over_lap_info for id: " << obstable_lane_id;
      return -1;
    }

    double stop_line_end_s = over_lap_info->lane_overlap_info().end_s();
    double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
    double distance_pass_stop_line = obstacle_end_s - stop_line_end_s;
    if (distance_pass_stop_line > conf_min_pass_s_distance && !is_path_cross) {
      erase = true;

      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] distance_pass_stop_line[" << distance_pass_stop_line
             << "] stop_line_end_s[" << stop_line_end_s << "] obstacle_end_s["
             << obstacle_end_s << "] is_path_cross[" << is_path_cross
             << "] passed stop sign, AND path not crosses. "
             << "erase from watch_vehicles";
    }
  } else {
    // passes associated lane (in junction)
    if (!is_path_cross) {
      erase = true;
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] obstable_lane_id[" << obstable_lane_id << "] is_path_cross["
             << is_path_cross
             << "] passed associated lane, AND path not crosses. "
             << "erase from watch_vehicles";
    }
  }

  // check if obstacle stops
  /*
  if (!erase) {
    auto speed = std::hypot(perception_obstacle.velocity().x(),
                            perception_obstacle.velocity().y());
    if (speed > config_.stop_sign().max_watch_vehicle_stop_speed()) {
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name
          << "] velocity[" << speed
          << "] not stopped. erase from watch_vehicles";
      erase = true;
    }
  }
  */

  if (erase) {
    for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
         it != watch_vehicles->end(); ++it) {
      std::vector<std::string>& vehicles = it->second;
      vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), obstacle_id),
                     vehicles.end());
    }
  }

  return 0;
}

/**
 * @brief: check valid stop_sign stop
 */
bool StopSignUnprotectedScenario::CheckADCStop(
    const ReferenceLineInfo& reference_line_info) {
  double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  if (adc_speed > conf_max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  double stop_line_start_s = next_stop_sign_overlap_.start_s;
  double distance_stop_line_to_adc_front_edge =
      stop_line_start_s - adc_front_edge_s;
  ADEBUG << "distance_stop_line_to_adc_front_edge["
         << distance_stop_line_to_adc_front_edge << "]; stop_line_start_s["
         << stop_line_start_s << "]; adc_front_edge_s[" << adc_front_edge_s
         << "]";

  if (distance_stop_line_to_adc_front_edge > conf_max_valid_stop_distance) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // TODO(all): check no BICYCLE in between.

  return true;
}

}  // namespace planning
}  // namespace apollo
