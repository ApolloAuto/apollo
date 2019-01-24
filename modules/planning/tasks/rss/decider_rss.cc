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

#include "modules/planning/tasks/rss/decider_rss.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

RssDecider::RssDecider(const TaskConfig &config) : Task(config) {
  SetName("RssDecider");
}

apollo::common::Status RssDecider::Execute(
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  return Process(frame, reference_line_info);
}

Status RssDecider::Process(Frame *frame,
                           ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (reference_line_info->path_data().Empty() ||
      reference_line_info->speed_data().empty()) {
    std::string msg("Empty path or speed data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  double adc_velocity = frame->vehicle_state().linear_velocity();
  const PathDecision *path_decision = reference_line_info->path_decision();
  const double ego_v_s_start = reference_line_info->AdcSlBoundary().start_s();
  const double ego_v_s_end = reference_line_info->AdcSlBoundary().end_s();
  const double ego_v_l_start = reference_line_info->AdcSlBoundary().start_l();
  const double ego_v_l_end = reference_line_info->AdcSlBoundary().end_l();
  double nearest_obs_s_start = 0.0;
  double nearest_obs_s_end = 0.0;
  double nearest_obs_l_start = 0.0;
  double nearest_obs_l_end = 0.0;
  double nearest_obs_speed = 0.0;

  double front_obstacle_distance = std::numeric_limits<double>::max();
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    bool is_on_road = reference_line_info->reference_line().HasOverlap(
        obstacle->PerceptionBoundingBox());
    if (!is_on_road) {
      continue;
    }
    const auto &obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle_sl.end_s() <= ego_v_s_start) {
      continue;
    }
    double distance = obstacle_sl.start_s() - ego_v_s_end;
    if (distance > 0 && distance < front_obstacle_distance) {
      front_obstacle_distance = distance;
      nearest_obs_s_start = obstacle_sl.start_s();
      nearest_obs_s_end = obstacle_sl.end_s();
      nearest_obs_l_start = obstacle_sl.start_l();
      nearest_obs_l_end = obstacle_sl.end_l();
      nearest_obs_speed = obstacle->speed();
    }
  }

  bool in_test_mode = false;
  if (in_test_mode) {
    double obs_s_dist = nearest_obs_s_end - nearest_obs_s_start;
    nearest_obs_s_start = ego_v_s_end + 5.0;
    nearest_obs_s_end = nearest_obs_s_start + obs_s_dist;
    front_obstacle_distance = nearest_obs_s_start - ego_v_s_end;
  }

  double left_width_ego = 0.0;
  double right_width_ego = 0.0;
  reference_line_info->reference_line().GetLaneWidth(
      ego_v_s_start, &left_width_ego, &right_width_ego);

  double left_width_obs = 0.0;
  double right_width_obs = 0.0;
  reference_line_info->reference_line().GetLaneWidth(
      nearest_obs_s_start, &left_width_obs, &right_width_obs);

  double total_lane_width_ego = left_width_ego + right_width_ego;
  double total_lane_width_obs = left_width_obs + right_width_obs;
  double total_lane_length = std::max(nearest_obs_s_end, ego_v_s_end);

  ADEBUG << "Task " << Name()
         << " nearest_obstacle_distance = " << front_obstacle_distance
         << " obs_s_start = " << nearest_obs_s_start
         << " obs_s_end = " << nearest_obs_s_end
         << " obs_l_start = " << nearest_obs_l_start
         << " obs_l_end = " << nearest_obs_l_end
         << " obs_speed = " << nearest_obs_speed
         << " ego_v_s_start = " << ego_v_s_start
         << " ego_v_s_end = " << ego_v_s_end
         << " ego_v_l_start = " << ego_v_l_start
         << " ego_v_l_end = " << ego_v_l_end
         << " lane_width_ego = " << total_lane_width_ego
         << " lane_width_obs = " << total_lane_width_obs
         << " left_width_obs = " << left_width_obs
         << " lane_length = " << total_lane_length;

  rss::world::Object followingObject;
  rss::world::Object leadingObject;
  rss::world::RoadArea roadArea;
  rss::world::Scene scene;

  scene.setSituationType(rss::situation::SituationType::SameDirection);

  leadingObject =
      rss::createObject(nearest_obs_speed, 0.0);  // TODO(l1212s): refine
  leadingObject.objectId = 0;
  rss::world::OccupiedRegion occupiedRegion_leading;
  occupiedRegion_leading.segmentId = 0;
  occupiedRegion_leading.lonRange.minimum =
      nearest_obs_s_start / total_lane_length;
  occupiedRegion_leading.lonRange.maximum =
      nearest_obs_s_end / total_lane_length;
  occupiedRegion_leading.latRange.minimum = std::max(
      (left_width_obs - nearest_obs_l_end) / total_lane_width_obs, 0.0);
  occupiedRegion_leading.latRange.maximum = std::min(
      (left_width_obs - nearest_obs_l_start) / total_lane_width_obs, 1.0);
  leadingObject.occupiedRegions.push_back(occupiedRegion_leading);

  ADEBUG << " occupiedRegion_leading.lonRange.minimum = "
         << occupiedRegion_leading.lonRange.minimum
         << " occupiedRegion_leading.lonRange.maximum = "
         << occupiedRegion_leading.lonRange.maximum
         << " occupiedRegion_leading.latRange.minimum = "
         << occupiedRegion_leading.latRange.minimum
         << " occupiedRegion_leading.latRange.maximum = "
         << occupiedRegion_leading.latRange.maximum;

  followingObject =
      rss::createObject(adc_velocity, 0.0);  // TODO(l1212s): refine
  followingObject.objectId = 1;
  rss::world::OccupiedRegion occupiedRegion_following;
  occupiedRegion_following.segmentId = 0;
  occupiedRegion_following.lonRange.minimum = ego_v_s_start / total_lane_length;
  occupiedRegion_following.lonRange.maximum = ego_v_s_end / total_lane_length;
  occupiedRegion_following.latRange.minimum =
      (left_width_ego - ego_v_l_end) / total_lane_width_ego;
  occupiedRegion_following.latRange.maximum =
      (left_width_ego - ego_v_l_start) / total_lane_width_ego;
  followingObject.occupiedRegions.push_back(occupiedRegion_following);

  ADEBUG << " adc_velocity = " << adc_velocity
         << " occupiedRegion_following.lonRange.minimum = "
         << occupiedRegion_following.lonRange.minimum
         << " occupiedRegion_following.lonRange.maximum = "
         << occupiedRegion_following.lonRange.maximum
         << " occupiedRegion_following.latRange.minimum = "
         << occupiedRegion_following.latRange.minimum
         << " occupiedRegion_following.latRange.maximum = "
         << occupiedRegion_following.latRange.maximum;

  rss::world::RoadSegment roadSegment;
  rss::world::LaneSegment laneSegment;

  laneSegment.id = 0;
  laneSegment.length.minimum = total_lane_length;
  laneSegment.length.maximum = total_lane_length;
  laneSegment.width.minimum =
      std::min(total_lane_width_ego, total_lane_width_obs);
  laneSegment.width.maximum =
      std::max(total_lane_width_ego, total_lane_width_obs);
  roadSegment.push_back(laneSegment);

  roadArea.push_back(roadSegment);

  ADEBUG << " laneSegment.length.min = " << laneSegment.length.minimum
         << " laneSegment.length.maximum = " << laneSegment.length.maximum
         << " laneSegment.width.minimum = " << laneSegment.width.minimum
         << " laneSegment.width.maximum = " << laneSegment.width.maximum;

  rss::world::WorldModel worldModel;
  worldModel.egoVehicle = followingObject;
  scene.object = leadingObject;
  scene.egoVehicleRoad = roadArea;
  worldModel.scenes.push_back(scene);
  worldModel.timeIndex = 1;

  double dMin_lon =
      rss::calculateLongitudinalMinSafeDistance(followingObject, leadingObject);
  rss::world::AccelerationRestriction accelerationRestriction;
  rss::core::RssCheck rssCheck;
  rssCheck.calculateAccelerationRestriction(worldModel,
                                            accelerationRestriction);

  if (front_obstacle_distance > dMin_lon) {
    ADEBUG << "Task " << Name() << " Distance is RSS-Safe";
    reference_line_info->mutable_rss_info()->set_is_rss_safe(true);
  } else {
    ADEBUG << "Task " << Name() << " Distance is not RSS-Safe";
    reference_line_info->mutable_rss_info()->set_is_rss_safe(false);
    if (FLAGS_enable_rss_fallback) {
      reference_line_info->mutable_speed_data()->clear();
    }
  }

  static const double kTrajectoryLenFactor = 10.0;
  // skip to populate RSS cur_dist_lon if it is too large.
  if (front_obstacle_distance <
      kTrajectoryLenFactor * reference_line_info->TrajectoryLength()) {
    reference_line_info->mutable_rss_info()->set_cur_dist_lon(
        front_obstacle_distance);
  }

  reference_line_info->mutable_rss_info()->set_rss_safe_dist_lon(dMin_lon);
  reference_line_info->mutable_rss_info()->set_acc_lon_range_minimum(
      accelerationRestriction.longitudinalRange.minimum);
  reference_line_info->mutable_rss_info()->set_acc_lon_range_maximum(
      accelerationRestriction.longitudinalRange.maximum);
  reference_line_info->mutable_rss_info()->set_acc_lat_left_range_minimum(
      accelerationRestriction.lateralLeftRange.minimum);
  reference_line_info->mutable_rss_info()->set_acc_lat_left_range_maximum(
      accelerationRestriction.lateralLeftRange.maximum);
  reference_line_info->mutable_rss_info()->set_acc_lat_right_range_minimum(
      accelerationRestriction.lateralRightRange.minimum);
  reference_line_info->mutable_rss_info()->set_acc_lat_right_range_maximum(
      accelerationRestriction.lateralRightRange.maximum);

  ADEBUG << " is_rss_safe : " << reference_line_info->rss_info().is_rss_safe();
  ADEBUG << " cur_dist_lon: " << reference_line_info->rss_info().cur_dist_lon();
  ADEBUG << " rss_safe_dist_lon: "
         << reference_line_info->rss_info().rss_safe_dist_lon();
  ADEBUG << " acc_longitudianlRange_minimum: "
         << reference_line_info->rss_info().acc_lon_range_minimum();
  ADEBUG << " acc_longitudinalRange_maximum: "
         << reference_line_info->rss_info().acc_lon_range_maximum();
  ADEBUG << " acc_lateralLeftRange_minimum: "
         << reference_line_info->rss_info().acc_lat_left_range_minimum();
  ADEBUG << " acc_lateralLeftRange_maximum: "
         << reference_line_info->rss_info().acc_lat_left_range_maximum();
  ADEBUG << " acc_lateralRightRange: "
         << reference_line_info->rss_info().acc_lat_right_range_minimum();
  ADEBUG << " acc_lateralRightRange_maximum: "
         << reference_line_info->rss_info().acc_lat_right_range_maximum();

  return Status::OK();
}

}  //  namespace planning
}  //  namespace apollo
