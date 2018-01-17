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
 * @file stop_sign.cc
 **/

#include "modules/planning/tasks/traffic_decider/stop_sign.h"

#include <limits>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/dropbox.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::common::util::Dropbox;
using apollo::common::util::WithinBound;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfo;
using apollo::hdmap::StopSignInfoConstPtr;

StopSign::StopSign(const RuleConfig& config) : TrafficRule(config) {}

bool StopSign::ApplyRule(Frame* frame,
                         ReferenceLineInfo* const reference_line_info) {
  if (!FLAGS_enable_stop_sign) {
    return true;
  }

  if (!FindNextStopSign(reference_line_info)) {
    return true;
  }

  MakeDecisions(frame, reference_line_info);

  return true;
}

/**
 * @brief: make decision
 */
void StopSign::MakeDecisions(
    Frame* frame,
    ReferenceLineInfo* const reference_line_info) {

  // check & update stop status
  ProcessStopStatus(reference_line_info, *next_stop_sign_);

  StopSignLaneVehicles watch_vehicles;
  GetWatchVehicles(*next_stop_sign_, &watch_vehicles);

  auto* path_decision = reference_line_info->path_decision();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const PerceptionObstacle& perception_obstacle =
        path_obstacle->obstacle()->Perception();
    const std::string& obstacle_id = path_obstacle->Id();
    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle_type);

    // check type
    if (obstacle_type != PerceptionObstacle::UNKNOWN &&
        obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
        obstacle_type != PerceptionObstacle::BICYCLE &&
        obstacle_type != PerceptionObstacle::VEHICLE) {
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name << "]. skip";
      continue;
    }

    if (stop_status_ == StopSignStopStatus::TO_STOP) {
      // add to watch_vehicles if adc is still proceeding to stop sign
      AddWatchVehicle(*path_obstacle, &watch_vehicles);
    } else if (stop_status_ == StopSignStopStatus::STOPPING ||
        stop_status_ == StopSignStopStatus::STOP_DONE) {
      // remove from watch_vehicles if adc is stopping/waiting at stop sign
      RemoveWatchVehicle(*path_obstacle, &watch_vehicles);
    }
  }


  std::string stop_sign_id = next_stop_sign_->id().id();
  if (stop_status_ == StopSignStopStatus::STOP_DONE &&
      watch_vehicles.empty()) {
    // stop done and no vehicles to wait for
    ClearDropbox(stop_sign_id);
    ADEBUG << "stop_sign_id[" << stop_sign_id << "] done";
  } else {
    // skip stop_sign if master vehicle body already passes the stop line
    double stop_line_start_s = next_stop_sign_overlap_->start_s;
    double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
    if (stop_line_start_s + FLAGS_stop_max_distance_buffer <=
        adc_front_edge_s) {
      ADEBUG << "skip: adc_front_edge passes stop_line+buffer. "
          << "stop_sign_id[" << stop_sign_id
          << "]; stop_line_start_s[" << stop_line_start_s
          << "]; adc_front_edge_s[" << adc_front_edge_s << "]";
    } else {
      // stop decision
      double stop_deceleration =
          GetStopDeceleration(reference_line_info, next_stop_sign_overlap_);
      if (stop_deceleration < FLAGS_stop_max_deceleration) {
        BuildStopDecision(frame, reference_line_info, next_stop_sign_overlap_);
      }
    }
  }
}

/**
 * @brief: fine next stop sign ahead of adc along reference line
 */
bool StopSign::FindNextStopSign(
    ReferenceLineInfo* const reference_line_info) {
  next_stop_sign_overlap_ = nullptr;
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();

  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double min_start_s = std::numeric_limits<double>::max();
  for (const PathOverlap& stop_sign_overlap : stop_sign_overlaps) {
    if (stop_sign_overlap.start_s + FLAGS_stop_max_distance_buffer
        > adc_front_edge_s &&
        stop_sign_overlap.start_s < min_start_s ) {
      min_start_s = stop_sign_overlap.start_s;
      next_stop_sign_overlap_ = const_cast<PathOverlap*>(&stop_sign_overlap);
    }
  }

  if (next_stop_sign_overlap_ == nullptr) {
    return false;
  }

  auto next_stop_sign_ptr = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(next_stop_sign_overlap_->object_id));
  next_stop_sign_ = std::move(
      const_cast<StopSignInfo*>(next_stop_sign_ptr.get()));

  // find all the lanes associated/guarded by the stop sign
  GetAssociateLanes(*next_stop_sign_);

  return true;
}

/*
 * get all the lanes associated/guarded by a stop sign
 */
int StopSign::GetAssociateLanes(const StopSignInfo& stop_sign_info) {
  associate_lanes_.clear();
  HDMapUtil::BaseMap().GetStopSignAssociateLanes(
      stop_sign_info.id(),
      &associate_lanes_);

  ADEBUG << "stop_sign: " << stop_sign_info.id().id()
      << "; associate_lanes_: ["
      << accumulate(associate_lanes_.begin(),
                    associate_lanes_.end(),
                    std::string{},
                    [](std::string &s, const LaneInfoConstPtr lane) {
                      return s += lane.get()->id().id(); })
      << "]; size[" << associate_lanes_.size() << "]";

  return 0;
}


/**
 * @brief: process & update stop status
 */
int StopSign::ProcessStopStatus(
    ReferenceLineInfo* const reference_line_info,
    const StopSignInfo& stop_sign_info) {
  if (reference_line_info == nullptr) {
    AWARN << "reference_line_info is nullptr. skip";
    return 0;
  }

  // get stop status from dropbox
  std::string stop_sign_id = stop_sign_info.id().id();
  std::string db_key_stop_status =
      db_key_stop_sign_stop_status_prefix_ + stop_sign_id;
  StopSignStopStatus* status = Dropbox<StopSignStopStatus>::Open()->Get(
      db_key_stop_status);
  stop_status_ = (status == nullptr) ?
      StopSignStopStatus::TO_STOP : *status;
  ADEBUG << "get stop_status_: "
      << static_cast<typename std::underlying_type<StopSignStopStatus>::type>(
          stop_status_);

  // get stop start time from dropbox
  std::string db_key_stop_starttime =
      db_key_stop_sign_stop_starttime_prefix_ + stop_sign_id;
  double* start_time = Dropbox<double>::Open()->Get(
      db_key_stop_starttime);
  double stop_start_time =  (start_time == nullptr) ?
      Clock::NowInSeconds() + 1 : *start_time;
  double wait_time = Clock::NowInSeconds() - stop_start_time;
  ADEBUG << "stop_start_time: " << stop_start_time
      << "; wait_time: " << wait_time;

  // check & update stop status
  switch (stop_status_) {
    case StopSignStopStatus::TO_STOP:
      if (ChecADCkStop(reference_line_info)) {
        stop_start_time = Clock::NowInSeconds();
        stop_status_ = StopSignStopStatus::STOPPING;
      }
      break;
    case StopSignStopStatus::STOPPING:
      if (wait_time > FLAGS_stop_duration_for_stop_sign) {
        stop_status_ = StopSignStopStatus::STOP_DONE;
      }
      break;
    case StopSignStopStatus::STOP_DONE:
      break;
  }

  // update dropbox: stop status
  Dropbox<StopSignStopStatus>::Open()->Set(db_key_stop_status,
                                           stop_status_);
  ADEBUG<< "update dropbox: [" << db_key_stop_status << "] = "
      << static_cast<typename std::underlying_type<StopSignStopStatus>::type>(
          stop_status_);

  // update dropbox: stop start time
  Dropbox<double>::Open()->Set(db_key_stop_starttime,
                               stop_start_time);
  ADEBUG << "update dropbox: [" << db_key_stop_starttime
      << "] = " << stop_start_time;

  return 0;
}

/**
 * @brief: check valid stop_sign stop
 */
bool StopSign::ChecADCkStop(
    ReferenceLineInfo* const reference_line_info) {
  double adc_speed = reference_line_info->AdcPlanningPoint().v();
  if (adc_speed > FLAGS_stop_min_speed) {
    ADEBUG << "master vehicle not stopped";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_start_s = next_stop_sign_overlap_->start_s;
  double distance_stop_line_to_adc_front_edge =
      stop_line_start_s - adc_front_edge_s;
  ADEBUG << "distance_stop_line_to_adc_front_edge["
      << distance_stop_line_to_adc_front_edge
      << "]; stop_line_start_s" << stop_line_start_s
      << "]; adc_front_edge_s[" << adc_front_edge_s << "]";

  if (distance_stop_line_to_adc_front_edge > FLAGS_max_valid_stop_distance) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // TODO(all): check no BICYCLE in between.

  return true;
}

/**
 * @brief: read watch vehicles from drop box
 */
int StopSign::GetWatchVehicles(
    const StopSignInfo& stop_sign_info,
    StopSignLaneVehicles* watch_vehicles) {
  watch_vehicles->clear();

  // get watch vehicles for associate_lanes
  for (const LaneInfoConstPtr associate_lane_info_ptr : associate_lanes_) {
    const LaneInfo* associate_lane_info = associate_lane_info_ptr.get();
    std::string associate_lane_id = associate_lane_info->id().id();

    if (stop_status_ == StopSignStopStatus::TO_STOP) {
      ADEBUG << "ADC not stopped. clean/init watch vehicles.";
      (*watch_vehicles)[associate_lane_id].clear();
    } else {
      // get watch vehicles for associate_lanes from dropbox
      std::string db_key_watch_vehicle =
          db_key_stop_sign_watch_vehicle_prefix_ + associate_lane_id;
      std::vector<std::string> *value =
          Dropbox<std::vector<std::string>>::Open()->Get(
              db_key_watch_vehicle);
      std::vector<std::string> watch_vehicle_ids;
      if (value != nullptr) {
        watch_vehicle_ids = *value;
      }

      ADEBUG << "watch_vehicle: lane_id[" << associate_lane_id << "] vehicle["
          << accumulate(watch_vehicle_ids.begin(),
                        watch_vehicle_ids.end(), std::string(","))
          << "]; size[" << watch_vehicle_ids.size() << "]";

      (*watch_vehicles)[associate_lane_id] = watch_vehicle_ids;
    }
  }

  return 0;
}

/**
 * @brief: update drop box with watch vehicles
 */
int StopSign::UpdateWatchVehicles(StopSignLaneVehicles* watch_vehicles) {
  for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
      it != watch_vehicles->end(); ++it) {
    std::string associate_lane_id = it->first;
    std::string db_key_watch_vehicle =
        db_key_stop_sign_watch_vehicle_prefix_ + associate_lane_id;
    Dropbox<std::vector<std::string>>::Open()->Set(
        db_key_watch_vehicle, it->second);
  }

  return 0;
}

/**
 * @brief: add a watch vehicle which arrives at stop sign ahead of adc
 */
int StopSign::AddWatchVehicle(
    const PathObstacle& obstacle,
    StopSignLaneVehicles* watch_vehicles) {
  const std::string& obstacle_id = obstacle.Id();
  const PerceptionObstacle& perception_obstacle =
      obstacle.obstacle()->Perception();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name =
      PerceptionObstacle_Type_Name(obstacle_type);

  auto point = common::util::MakePointENU(
      perception_obstacle.position().x(),
      perception_obstacle.position().y(),
      perception_obstacle.position().z());
  double obstacle_s = 0.0;
  double obstacle_l = 0.0;
  hdmap::LaneInfoConstPtr obstacle_lane;
  const hdmap::HDMap *hdmap = hdmap::HDMapUtil::BaseMapPtr();
  if (hdmap->GetNearestLaneWithHeading(
      point, 5.0, perception_obstacle.theta(), M_PI / 3.0,
      &obstacle_lane, &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name
        << "]: Failed to find nearest lane from map for position: "
        << point.DebugString() << "; heading:" << perception_obstacle.theta();
    return -1;
  }

  // check obstacle is on an associate lane guarded by stop sign
  std::string obstable_lane_id =  obstacle_lane.get()->id().id();
  auto it = std::find_if(
      associate_lanes_.begin(),
      associate_lanes_.end(),
      [&obstable_lane_id](const LaneInfoConstPtr lane) {
          return lane.get()->id().id() == obstable_lane_id; });
  if (it == associate_lanes_.end()) {
    ADEBUG << "obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name
        << "] lane_id[" << obstable_lane_id
        << "] not associated with current stop_sign. skip";
    return -1;
  }

  auto speed = std::hypot(perception_obstacle.velocity().x(),
                          perception_obstacle.velocity().y());
  if (speed > FLAGS_stop_min_speed) {
    ADEBUG << "obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name
        << "] velocity[" << speed
        << "] not stopped. skip";
    return -1;
  }

  // check stop close enough to stop line of the stop_sign
  // TODO(all): find stop_line_s of associated stop sign
  double stop_line_s = 0;
  double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
  double distance_to_stop_line = stop_line_s - obstacle_end_s;
  if (distance_to_stop_line > FLAGS_max_valid_stop_distance) {
    ADEBUG << "distance_to_stop_line["
        << distance_to_stop_line
        << "]; stop_line_s" << stop_line_s
        << "]; obstacle_end_s[" << obstacle_end_s
        << "] too far from stop line. skip";
    return -1;
  }

  // use a vector since motocycles/bicycles can be more than one
  (*watch_vehicles)[obstacle_lane->id().id()].push_back(obstacle_id);

  return 0;
}

/**
 * @brief: remove a watch vehicle which not stopping at stop sign any more
 */
int StopSign::RemoveWatchVehicle(
    const PathObstacle& obstacle,
    StopSignLaneVehicles* watch_vehicles) {
  const std::string& obstacle_id = obstacle.Id();
  const PerceptionObstacle& perception_obstacle =
      obstacle.obstacle()->Perception();
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name =
      PerceptionObstacle_Type_Name(obstacle_type);

  auto point = common::util::MakePointENU(
      perception_obstacle.position().x(),
      perception_obstacle.position().y(),
      perception_obstacle.position().z());
  double obstacle_s = 0.0;
  double obstacle_l = 0.0;
  hdmap::LaneInfoConstPtr obstacle_lane;
  const hdmap::HDMap *hdmap = hdmap::HDMapUtil::BaseMapPtr();
  if (hdmap->GetNearestLaneWithHeading(
      point, 5.0, perception_obstacle.theta(),
      M_PI / 3.0, &obstacle_lane,
      &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name
        << "]: Failed to find nearest lane from map for position: "
        << point.DebugString() << "; heading:" << perception_obstacle.theta();
    return -1;
  }

  bool erase = false;

  // check obstacle is on an associate lane guarded by stop sign
  std::string obstable_lane_id =  obstacle_lane.get()->id().id();
  auto it = std::find_if(
      associate_lanes_.begin(),
      associate_lanes_.end(),
      [&obstable_lane_id](const LaneInfoConstPtr lane) {
          return lane.get()->id().id() == obstable_lane_id; });
  if (it == associate_lanes_.end()) {
    ADEBUG << "obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name
        << "] lane_id[" << obstable_lane_id
        << "] not associated with current stop_sign. erase from watch_vehicles";
    erase = true;
  }

  auto speed = std::hypot(perception_obstacle.velocity().x(),
                          perception_obstacle.velocity().y());
  if (speed > FLAGS_stop_min_speed) {
    ADEBUG << "obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name
        << "] velocity[" << speed
        << "] not stopped. erase from watch_vehicles";
    erase = true;
  }

  // check pass stop line of the stop_sign
  // TODO(all): find stop_line_s of associated stop sign
  double stop_line_s = 0;
  double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
  double distance_pass_stop_line = obstacle_end_s - stop_line_s;
  if (distance_pass_stop_line > FLAGS_max_valid_stop_distance) {
    ADEBUG << "distance_pass_stop_line["
        << distance_pass_stop_line
        << "]; stop_line_s" << stop_line_s
        << "]; obstacle_end_s[" << obstacle_end_s
        << "] passed stop line.  erase from watch_vehicles";
    erase = true;
  }

  if (erase) {
    for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
        it != watch_vehicles->end(); ++it) {
      std::vector<std::string> vehicles = it->second;
      vehicles.erase(std::remove(vehicles.begin(), vehicles.end(),
                                 obstacle_id));
    }
  }

  return 0;
}


double StopSign::GetStopDeceleration(
    ReferenceLineInfo* const reference_line_info,
    const PathOverlap* stop_sign_overlap) {
  double adc_speed =
      common::VehicleStateProvider::instance()->linear_velocity();
  if (adc_speed < FLAGS_stop_min_speed) {
    return 0.0;
  }
  double stop_distance = 0;
  double adc_front_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_s = stop_sign_overlap->start_s;

  if (stop_line_s > adc_front_s) {
    stop_distance = stop_line_s - adc_front_s;
  } else {
    stop_distance = stop_line_s + FLAGS_stop_max_distance_buffer - adc_front_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

bool StopSign::BuildStopDecision(
    Frame* frame,
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* stop_sign_overlap) {

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), stop_sign_overlap->start_s)) {
    ADEBUG << "stop_sign " << stop_sign_overlap->object_id
           << " is not on reference line";
    return true;
  }

  // create virtual stop wall
  std::string virtual_object_id =
      FLAGS_stop_sign_virtual_object_id_prefix + stop_sign_overlap->object_id;
  auto* obstacle = frame->AddVirtualStopObstacle(
      reference_line_info,
      virtual_object_id,
      stop_sign_overlap->start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle " << virtual_object_id << " in frame";
    return false;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for " << virtual_object_id;
    return false;
  }

  // build stop decision
  const double stop_s =
      stop_sign_overlap->start_s - FLAGS_stop_distance_stop_sign;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_STOP_SIGN);
  stop_decision->set_distance_s(-FLAGS_stop_distance_stop_sign);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      RuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return true;
}

void StopSign::ClearDropbox(const std::string& stop_sign_id) {
  // clear stop status from dropbox
  std::string db_key_stop_status =
      db_key_stop_sign_stop_status_prefix_ + stop_sign_id;
  Dropbox<StopSignStopStatus>::Open()->Remove(db_key_stop_status);
  ADEBUG << "remove dropbox item: " << db_key_stop_status;

  // clear stop start time from dropbox
  std::string db_key_stop_starttime =
      db_key_stop_sign_stop_starttime_prefix_ + stop_sign_id;
  Dropbox<double>::Open()->Remove(db_key_stop_starttime);
  ADEBUG << "remove dropbox item: " << db_key_stop_starttime;

  // clear watch vehicles from dropbox
  for (const LaneInfoConstPtr associate_lane_info_ptr : associate_lanes_) {
    const LaneInfo* associate_lane_info = associate_lane_info_ptr.get();
    std::string associate_lane_id = associate_lane_info->id().id();
    std::string db_key_watch_vehicle
             = db_key_stop_sign_watch_vehicle_prefix_ + associate_lane_id;
    Dropbox<std::vector<std::string>>::Open()->Remove(db_key_watch_vehicle);
    ADEBUG << "remove dropbox item: " << db_key_watch_vehicle;
  }
}

}  // namespace planning
}  // namespace apollo
