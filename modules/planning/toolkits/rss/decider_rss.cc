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

#include "modules/planning/toolkits/rss/decider_rss.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

using rss::world::Object;
using rss::world::RoadArea;
using rss::world::Scene;

RssDecider::RssDecider(const TaskConfig &config) : Task(config) {
  SetName("RssDecider");
}

apollo::common::Status RssDecider::Execute(
    Frame *frame, ReferenceLineInfo *reference_line_info) {

  return Process(frame, reference_line_info);
}

Status RssDecider::Process(
    Frame *frame, ReferenceLineInfo *reference_line_info) {

  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

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
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    bool is_on_road = reference_line_info->reference_line().HasOverlap(
        obstacle->PerceptionBoundingBox());
    if (!is_on_road) {
      continue;
    }
    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
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

  double left_width = 0.0;
  double right_width = 0.0;
  reference_line_info->reference_line().GetLaneWidth(
      ego_v_s_start, &left_width, &right_width);

  double total_lane_width = left_width + right_width;  // TODO(l1212s): refine
  double total_lane_length = std::max(nearest_obs_s_end, ego_v_s_end);

  ADEBUG << "Task " << Name() << " nearest_obstacle_distance = "
        << front_obstacle_distance << " obs_s_start = "
        << nearest_obs_s_start << " obs_s_end = "
        << nearest_obs_s_end << " obs_l_start = "
        << nearest_obs_l_start << " obs_l_end = "
        << nearest_obs_l_end << " obs_speed = "
        << nearest_obs_speed << " ego_v_s_start = "
        << ego_v_s_start << " ego_v_s_end = "
        << ego_v_s_end << " ego_v_l_start = "
        << ego_v_l_start << " ego_v_l_end = "
        << ego_v_l_end << " lane_width = "
        << total_lane_width << " lane_length = "
        << total_lane_length;

  rss::world::Object followingObject;
  rss::world::Object leadingObject;
  rss::world::RoadArea roadArea;
  rss::world::Scene scene;

  scene.setSituationType(rss::situation::SituationType::SameDirection);
  leadingObject = rss::createObject(nearest_obs_speed, 0.0);
  leadingObject.objectId = 0;
  rss::world::OccupiedRegion occupiedRegion_leading;
  occupiedRegion_leading.segmentId = 0;
  occupiedRegion_leading.lonRange.minimum =
      nearest_obs_s_start / total_lane_length;
  occupiedRegion_leading.lonRange.maximum =
      nearest_obs_s_end / total_lane_length;
  occupiedRegion_leading.latRange.minimum =
      (left_width - nearest_obs_l_end) / total_lane_width;
  occupiedRegion_leading.latRange.maximum =
      (left_width - nearest_obs_l_start) / total_lane_width;
  leadingObject.occupiedRegions.push_back(occupiedRegion_leading);

  followingObject = rss::createObject(
                        adc_velocity, 0.0);  // TODO(l1212s): refine
  followingObject.objectId = 1;
  rss::world::OccupiedRegion occupiedRegion_following;
  occupiedRegion_following.segmentId = 0;
  occupiedRegion_following.lonRange.minimum =
      ego_v_s_start / total_lane_length;
  occupiedRegion_following.lonRange.maximum =
      ego_v_s_end / total_lane_length;
  occupiedRegion_following.latRange.minimum =
      (left_width - ego_v_l_end) / total_lane_width;
  occupiedRegion_following.latRange.maximum =
      (left_width - ego_v_l_start) / total_lane_width;
  followingObject.occupiedRegions.push_back(occupiedRegion_following);

  rss::world::RoadSegment roadSegment;
  rss::world::LaneSegment laneSegment;

  laneSegment.id = 0;
  laneSegment.length.minimum = total_lane_length;
  laneSegment.length.maximum = total_lane_length;
  laneSegment.width.minimum = total_lane_width;
  laneSegment.width.maximum = total_lane_width;
  roadSegment.push_back(laneSegment);

  roadArea.push_back(roadSegment);

  rss::world::WorldModel worldModel;
  worldModel.egoVehicle = followingObject;
  scene.object = leadingObject;
  scene.egoVehicleRoad = roadArea;
  worldModel.scenes.push_back(scene);
  worldModel.timeIndex = 1;

  double dMin_lon = rss::calculateLongitudinalMinSafeDistance(followingObject,
                        leadingObject);

  rss::world::AccelerationRestriction accelerationRestriction;
  rss::core::RssCheck rssCheck;
  rssCheck.calculateAccelerationRestriction(
      worldModel, accelerationRestriction);

  if (front_obstacle_distance >= dMin_lon) {
    ADEBUG << "Task " << Name() << " Distance is RSS-Safe";
  } else {
    if (accelerationRestriction.longitudinalRange.maximum ==
        -1. * worldModel.egoVehicle.dynamics.alphaLon.brakeMin) {
      ADEBUG << "Task " << Name() << " Distance is not RSS-Safe,"
            << "regulate acceleration";
    }
  }

  return Status::OK();
}

}  //  namespace planning
}  //  namespace apollo
