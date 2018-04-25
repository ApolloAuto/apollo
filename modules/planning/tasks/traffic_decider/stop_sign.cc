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

#include "modules/planning/tasks/traffic_decider/stop_sign.h"

#include <algorithm>
#include <limits>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/tasks/traffic_decider/util.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::common::util::WithinBound;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfo;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::OverlapInfoConstPtr;
using apollo::hdmap::PathOverlap;
using apollo::hdmap::StopSignInfo;
using apollo::hdmap::StopSignInfoConstPtr;
using apollo::perception::PerceptionObstacle;
using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;
using apollo::planning::util::GetPlanningStatus;

StopSign::StopSign(const TrafficRuleConfig& config) : TrafficRule(config) {}

bool StopSign::ApplyRule(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FindNextStopSign(reference_line_info)) {
    return true;
  }

  MakeDecisions(frame, reference_line_info);

  return true;
}

/**
 * @brief: make decision
 */
void StopSign::MakeDecisions(Frame* frame,
                             ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check & update stop status
  ProcessStopStatus(reference_line_info, *next_stop_sign_);

  StopSignLaneVehicles watch_vehicles;
  GetWatchVehicles(*next_stop_sign_, &watch_vehicles);

  auto* path_decision = reference_line_info->path_decision();
  if (stop_status_ == StopSignStatus::TO_STOP) {
    for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
      // add to watch_vehicles if adc is still proceeding to stop sign
      AddWatchVehicle(*path_obstacle, &watch_vehicles);
    }
  } else if (!watch_vehicles.empty() &&
             (stop_status_ == StopSignStatus::STOPPING ||
              stop_status_ == StopSignStatus::STOP_DONE)) {
    // get all vehicles currently watched
    std::vector<std::string> watch_vehicle_ids;
    for (StopSignLaneVehicles::iterator it = watch_vehicles.begin();
         it != watch_vehicles.end(); ++it) {
      std::copy(it->second.begin(), it->second.end(),
                std::back_inserter(watch_vehicle_ids));
    }

    for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
      // remove from watch_vehicles if adc is stopping/waiting at stop sign
      RemoveWatchVehicle(*path_obstacle, watch_vehicle_ids, &watch_vehicles);
    }
  }

  ClearWatchVehicle(reference_line_info, &watch_vehicles);

  UpdateWatchVehicles(&watch_vehicles);

  std::string stop_sign_id = next_stop_sign_->id().id();
  if (stop_status_ == StopSignStatus::STOP_DONE && watch_vehicles.empty()) {
    // stop done and no vehicles to wait for
    ADEBUG << "stop_sign_id[" << stop_sign_id << "] DONE";
  } else if (stop_status_ == StopSignStatus::CREEPING) {
    auto* next_overlap =
        reference_line_info->reference_line().map_path().NextLaneOverlap(
            reference_line_info->AdcSlBoundary().end_s());
    BuildStopDecision(frame, reference_line_info,
                      const_cast<PathOverlap*>(next_overlap),
                      config_.stop_sign().creep().stop_distance());
  } else {
    // stop decision
    double stop_deceleration = util::GetADCStopDeceleration(
        reference_line_info, next_stop_sign_overlap_->start_s,
        config_.stop_sign().min_pass_s_distance());
    if (stop_deceleration < FLAGS_max_stop_deceleration) {
      BuildStopDecision(frame, reference_line_info,
                        const_cast<PathOverlap*>(next_stop_sign_overlap_),
                        config_.stop_sign().stop_distance());
    }
    ADEBUG << "stop_sign_id[" << stop_sign_id << "] STOP";
  }
}

/**
 * @brief: fine next stop sign ahead of adc along reference line
 */
bool StopSign::FindNextStopSign(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  next_stop_sign_overlap_ = nullptr;
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();

  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double min_start_s = std::numeric_limits<double>::max();
  for (const PathOverlap& stop_sign_overlap : stop_sign_overlaps) {
    if (adc_front_edge_s - stop_sign_overlap.end_s <=
            config_.stop_sign().min_pass_s_distance() &&
        stop_sign_overlap.start_s < min_start_s) {
      min_start_s = stop_sign_overlap.start_s;
      next_stop_sign_overlap_ = const_cast<PathOverlap*>(&stop_sign_overlap);
    }
  }

  if (next_stop_sign_overlap_ == nullptr) {
    GetPlanningStatus()->clear_stop_sign();
    return false;
  }

  auto next_stop_sign_ptr = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(next_stop_sign_overlap_->object_id));
  next_stop_sign_ =
      std::move(const_cast<StopSignInfo*>(next_stop_sign_ptr.get()));

  // clear status for unrelavant stop signs
  if (GetPlanningStatus()->has_stop_sign()) {
    auto stop_sign_status = GetPlanningStatus()->stop_sign();
    if (stop_sign_status.stop_sign_id() != next_stop_sign_->id().id()) {
      GetPlanningStatus()->clear_stop_sign();
      GetPlanningStatus()->mutable_stop_sign()->set_stop_sign_id(
          next_stop_sign_->id().id());
    }
  }

  // find all the lanes associated/guarded by the stop sign
  GetAssociatedLanes(*next_stop_sign_);

  return true;
}

/*
 * get all the lanes associated/guarded by a stop sign
 */
int StopSign::GetAssociatedLanes(const StopSignInfo& stop_sign_info) {
  associated_lanes_.clear();

  std::vector<std::string> associated_lanes;
  std::vector<StopSignInfoConstPtr> associated_stop_signs;
  HDMapUtil::BaseMap().GetStopSignAssociatedStopSigns(stop_sign_info.id(),
                                                      &associated_stop_signs);

  for (const auto& stop_sign : associated_stop_signs) {
    if (stop_sign == nullptr) {
      continue;
    }

    const auto associated_lane_ids = stop_sign->OverlapLaneIds();
    for (const auto lane_id : associated_lane_ids) {
      const auto& lane = HDMapUtil::BaseMap().GetLaneById(lane_id);
      if (lane == nullptr) {
        continue;
      }
      auto stop_sign_overlaps = lane->stop_signs();
      for (auto stop_sign_overlap : stop_sign_overlaps) {
        auto over_lap_info =
            stop_sign_overlap->GetObjectOverlapInfo(stop_sign.get()->id());
        if (over_lap_info != nullptr) {
          associated_lanes.push_back(lane_id.id());
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
 * @brief: process & update stop status
 */
int StopSign::ProcessStopStatus(ReferenceLineInfo* const reference_line_info,
                                const StopSignInfo& stop_sign_info) {
  CHECK_NOTNULL(reference_line_info);

  // get stop status from PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  if (!stop_sign_status->has_status()) {
    stop_sign_status->set_status(StopSignStatus::UNKNOWN);
  }
  stop_status_ = stop_sign_status->status();

  // get stop start time from PlanningStatus
  double stop_start_time = Clock::NowInSeconds() + 1;
  if (stop_sign_status->has_stop_start_time()) {
    stop_start_time = stop_sign_status->stop_start_time();
  }
  double wait_time = Clock::NowInSeconds() - stop_start_time;
  ADEBUG << "stop_start_time: " << stop_start_time
         << "; wait_time: " << wait_time;

  // adjust status. this may happen if there's bad data
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_start_s = next_stop_sign_overlap_->start_s;
  if (stop_line_start_s - adc_front_edge_s >
      config_.stop_sign().max_valid_stop_distance()) {
    ADEBUG << "adjust stop status. too far from stop line. distance["
           << stop_line_start_s - adc_front_edge_s << "]";
    stop_status_ = StopSignStatus::TO_STOP;
  }

  // check & update stop status
  switch (stop_status_) {
    case StopSignStatus::UNKNOWN:
    case StopSignStatus::TO_STOP:
      if (!CheckADCkStop(reference_line_info)) {
        stop_status_ = StopSignStatus::TO_STOP;
      } else {
        stop_start_time = Clock::NowInSeconds();
        stop_status_ = StopSignStatus::STOPPING;

        // update PlanningStatus: stop start time
        stop_sign_status->set_stop_start_time(stop_start_time);
        ADEBUG << "update stop_start_time: " << stop_start_time;
      }
      break;
    case StopSignStatus::STOPPING:
      if (wait_time >= config_.stop_sign().stop_duration()) {
        if (config_.stop_sign().creep().enabled() &&
            (stop_sign_info.stop_sign().type() == hdmap::StopSign::ONE_WAY ||
             stop_sign_info.stop_sign().type() == hdmap::StopSign::TWO_WAY)) {
          stop_status_ = StopSignStatus::CREEPING;
        } else {
          stop_status_ = StopSignStatus::STOP_DONE;
        }
      }
      break;
    case StopSignStatus::CREEPING: {
      constexpr double kDeltaS = 0.5;
      auto* path_overlap =
          reference_line_info->reference_line().map_path().NextLaneOverlap(
              reference_line_info->AdcSlBoundary().end_s());
      if (path_overlap != nullptr &&
          path_overlap->start_s - reference_line_info->AdcSlBoundary().end_s() >
              kDeltaS) {
        // keep in CREEPING status
      } else {
        bool all_far_away = true;
        for (auto* obstacle :
             reference_line_info->path_decision()->path_obstacles().Items()) {
          if (obstacle->reference_line_st_boundary().min_t() <
              config_.stop_sign().creep().min_boundary_t()) {
            all_far_away = false;
            break;
          }
        }
        if (all_far_away) {
          stop_status_ = StopSignStatus::STOP_DONE;
        }
      }
      break;
    }
    case StopSignStatus::STOP_DONE:
      break;
    default:
      break;
  }

  // update PlanningStatus: stop status
  stop_sign_status->set_status(stop_status_);
  ADEBUG << "update stop_status: " << StopSignStatus_Status_Name(stop_status_);

  return 0;
}

/**
 * @brief: check valid stop_sign stop
 */
bool StopSign::CheckADCkStop(ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  double adc_speed = reference_line_info->AdcPlanningPoint().v();
  if (adc_speed > config_.stop_sign().max_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_start_s = next_stop_sign_overlap_->start_s;
  double distance_stop_line_to_adc_front_edge =
      stop_line_start_s - adc_front_edge_s;
  ADEBUG << "distance_stop_line_to_adc_front_edge["
         << distance_stop_line_to_adc_front_edge << "]; stop_line_start_s["
         << stop_line_start_s << "]; adc_front_edge_s[" << adc_front_edge_s
         << "]";

  if (distance_stop_line_to_adc_front_edge >
      config_.stop_sign().max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // TODO(all): check no BICYCLE in between.

  return true;
}

/**
 * @brief: read watch vehicles from PlanningStatus
 */
int StopSign::GetWatchVehicles(const StopSignInfo& stop_sign_info,
                               StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  watch_vehicles->clear();

  StopSignStatus stop_sign_status = GetPlanningStatus()->stop_sign();
  for (int i = 0; i < stop_sign_status.lane_watch_vehicles_size(); ++i) {
    auto lane_watch_vehicles = stop_sign_status.lane_watch_vehicles(i);
    std::string associated_lane_id = lane_watch_vehicles.lane_id();
    std::string s;
    for (int j = 0; j < lane_watch_vehicles.watch_vehicles_size(); ++j) {
      std::string vehicle = lane_watch_vehicles.watch_vehicles(j);
      s = s.empty() ? vehicle : s + "," + vehicle;
      (*watch_vehicles)[associated_lane_id].push_back(vehicle);
    }
    ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
           << s << "]";
  }

  return 0;
}

/**
 * @brief: update PlanningStatus with watch vehicles
 */
int StopSign::UpdateWatchVehicles(StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->clear_lane_watch_vehicles();

  for (auto it = watch_vehicles->begin(); it != watch_vehicles->end(); ++it) {
    auto* lane_watch_vehicles = stop_sign_status->add_lane_watch_vehicles();
    lane_watch_vehicles->set_lane_id(it->first);
    for (size_t i = 0; i < it->second.size(); ++i) {
      lane_watch_vehicles->add_watch_vehicles(it->second[i]);
    }
  }

  return 0;
}

/**
 * @brief: add a watch vehicle which arrives at stop sign ahead of adc
 */
int StopSign::AddWatchVehicle(const PathObstacle& path_obstacle,
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
    ADEBUG << "can't find over_lap_info for id: " << obstable_lane_id;
    return -1;
  }
  double stop_line_s = over_lap_info->lane_overlap_info().start_s();
  double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
  double distance_to_stop_line = stop_line_s - obstacle_end_s;
  if (distance_to_stop_line >
      config_.stop_sign().watch_vehicle_max_valid_stop_distance()) {
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
int StopSign::RemoveWatchVehicle(
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
  std::string obstable_lane_id = obstacle_lane.get()->id().id();
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
    if (distance_pass_stop_line > config_.stop_sign().min_pass_s_distance() &&
        !is_path_cross) {
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
      std::vector<std::string> vehicles = it->second;
      vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), obstacle_id),
                     vehicles.end());
      it->second = vehicles;
    }
  }

  return 0;
}

int StopSign::ClearWatchVehicle(ReferenceLineInfo* const reference_line_info,
                                StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(watch_vehicles);

  auto path_obstacles =
      reference_line_info->path_decision()->path_obstacles().Items();
  std::vector<std::string> obstacle_ids;
  std::transform(
      path_obstacles.begin(), path_obstacles.end(),
      std::back_inserter(obstacle_ids), [](const PathObstacle* path_obstacle) {
        return std::to_string(path_obstacle->obstacle()->Perception().id());
      });

  for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
       it != watch_vehicles->end();
       /*no increment*/) {
    std::vector<std::string> vehicles = it->second;
    // clean obstacles not in current perception
    for (auto obstacle_it = vehicles.begin(); obstacle_it != vehicles.end();
         /*no increment*/) {
      if (std::find(obstacle_ids.begin(), obstacle_ids.end(), *obstacle_it) ==
          obstacle_ids.end()) {
        ADEBUG << "lane[" << it->first << "] obstacle[" << *obstacle_it
               << "] not exist any more. erase.";
        obstacle_it = vehicles.erase(obstacle_it);
      } else {
        ++obstacle_it;
      }
    }
    it->second = vehicles;

    if (vehicles.empty()) {
      watch_vehicles->erase(it++);
    } else {
      ++it;
    }
  }

  /* debug
  for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
       it != watch_vehicles->end(); it++) {
    std::string s;
    std::for_each(it->second.begin(),
                  it->second.end(),
                  [&](std::string &id) { s = s.empty() ? id : s + "," + id; });
    ADEBUG << "ClearWatchVehicle: lane_id[" << it->first << "] vehicle["
        << s << "]; size[" << it->second.size() << "]";
  }
  */

  return 0;
}

bool StopSign::BuildStopDecision(Frame* frame,
                                 ReferenceLineInfo* const reference_line_info,
                                 PathOverlap* const overlap,
                                 const double stop_distance) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(overlap);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), overlap->start_s)) {
    ADEBUG << "stop_sign " << overlap->object_id << " is not on reference line";
    return true;
  }

  // create virtual stop wall
  std::string virtual_obstacle_id = STOP_SIGN_VO_ID_PREFIX + overlap->object_id;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, overlap->start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return false;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
    return false;
  }

  // build stop decision
  const double stop_s = overlap->start_s - stop_distance;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_STOP_SIGN);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
