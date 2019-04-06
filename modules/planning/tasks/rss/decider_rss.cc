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
using apollo::common::VehicleConfigHelper;

using ad_rss::physics::Distance;
using ad_rss::physics::ParametricValue;
using ad_rss::physics::Speed;
using ad_rss::situation::VehicleState;
using ad_rss::world::Object;
using ad_rss::world::RoadArea;
using ad_rss::world::Scene;

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

  // there is no obstacle in front of adc
  if (front_obstacle_distance == std::numeric_limits<double>::max()) {
    ::ad_rss::world::Dynamics dynamics;
    rss_config_default_dynamics(&dynamics);

    reference_line_info->mutable_rss_info()->set_is_rss_safe(true);
    reference_line_info->mutable_rss_info()->set_cur_dist_lon(
        front_obstacle_distance);
    reference_line_info->mutable_rss_info()->set_rss_safe_dist_lon(
        front_obstacle_distance);

    reference_line_info->mutable_rss_info()->set_acc_lon_range_minimum(
        -1 * static_cast<double>(dynamics.alphaLon.brakeMax));
    reference_line_info->mutable_rss_info()->set_acc_lon_range_maximum(
        static_cast<double>(dynamics.alphaLon.accelMax));
    reference_line_info->mutable_rss_info()->set_acc_lat_left_range_minimum(
        -1 * static_cast<double>(dynamics.alphaLat.brakeMin));
    reference_line_info->mutable_rss_info()->set_acc_lat_left_range_maximum(
        static_cast<double>(dynamics.alphaLat.accelMax));
    reference_line_info->mutable_rss_info()->set_acc_lat_right_range_minimum(
        -1 * static_cast<double>(dynamics.alphaLat.brakeMin));
    reference_line_info->mutable_rss_info()->set_acc_lat_right_range_maximum(
        static_cast<double>(dynamics.alphaLat.accelMax));

    return Status::OK();
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

  Object followingObject;
  Object leadingObject;
  RoadArea roadArea;
  Scene scene;

  scene.situationType = ad_rss::situation::SituationType::SameDirection;

  rss_create_other_object(&leadingObject, nearest_obs_speed, 0);

  ad_rss::world::OccupiedRegion occupiedRegion_leading;
  occupiedRegion_leading.segmentId = 0;
  occupiedRegion_leading.lonRange.minimum =
      ParametricValue(nearest_obs_s_start / total_lane_length);
  occupiedRegion_leading.lonRange.maximum =
      ParametricValue(nearest_obs_s_end / total_lane_length);
  occupiedRegion_leading.latRange.minimum = ParametricValue(std::max(
      (left_width_obs - nearest_obs_l_end) / total_lane_width_obs, 0.0));
  occupiedRegion_leading.latRange.maximum = ParametricValue(std::min(
      (left_width_obs - nearest_obs_l_start) / total_lane_width_obs, 1.0));
  leadingObject.occupiedRegions.push_back(occupiedRegion_leading);

  ADEBUG << " occupiedRegion_leading.lonRange.minimum = "
         << static_cast<double>(occupiedRegion_leading.lonRange.minimum)
         << " occupiedRegion_leading.lonRange.maximum = "
         << static_cast<double>(occupiedRegion_leading.lonRange.maximum)
         << " occupiedRegion_leading.latRange.minimum = "
         << static_cast<double>(occupiedRegion_leading.latRange.minimum)
         << " occupiedRegion_leading.latRange.maximum = "
         << static_cast<double>(occupiedRegion_leading.latRange.maximum);

  RssDecider::rss_create_ego_object(&followingObject, adc_velocity, 0.0);

  ad_rss::world::OccupiedRegion occupiedRegion_following;
  occupiedRegion_following.segmentId = 0;
  occupiedRegion_following.lonRange.minimum =
      ParametricValue(ego_v_s_start / total_lane_length);
  occupiedRegion_following.lonRange.maximum =
      ParametricValue(ego_v_s_end / total_lane_length);
  occupiedRegion_following.latRange.minimum =
      ParametricValue((left_width_ego - ego_v_l_end) / total_lane_width_ego);
  occupiedRegion_following.latRange.maximum =
      ParametricValue((left_width_ego - ego_v_l_start) / total_lane_width_ego);
  followingObject.occupiedRegions.push_back(occupiedRegion_following);

  ADEBUG << " adc_velocity = " << adc_velocity
         << " occupiedRegion_following.lonRange.minimum = "
         << static_cast<double>(occupiedRegion_following.lonRange.minimum)
         << " occupiedRegion_following.lonRange.maximum = "
         << static_cast<double>(occupiedRegion_following.lonRange.maximum)
         << " occupiedRegion_following.latRange.minimum = "
         << static_cast<double>(occupiedRegion_following.latRange.minimum)
         << " occupiedRegion_following.latRange.maximum = "
         << static_cast<double>(occupiedRegion_following.latRange.maximum);

  ad_rss::world::RoadSegment roadSegment;
  ad_rss::world::LaneSegment laneSegment;

  laneSegment.id = 0;
  laneSegment.length.minimum = Distance(total_lane_length);
  laneSegment.length.maximum = Distance(total_lane_length);
  laneSegment.width.minimum =
      Distance(std::min(total_lane_width_ego, total_lane_width_obs));
  laneSegment.width.maximum =
      Distance(std::max(total_lane_width_ego, total_lane_width_obs));
  roadSegment.push_back(laneSegment);

  roadArea.push_back(roadSegment);

  ADEBUG << " laneSegment.length.min = "
         << static_cast<double>(laneSegment.length.minimum)
         << " laneSegment.length.maximum = "
         << static_cast<double>(laneSegment.length.maximum)
         << " laneSegment.width.minimum = "
         << static_cast<double>(laneSegment.width.minimum)
         << " laneSegment.width.maximum = "
         << static_cast<double>(laneSegment.width.maximum);

  ad_rss::world::WorldModel worldModel;
  worldModel.egoVehicle = followingObject;
  scene.object = leadingObject;
  scene.egoVehicleRoad = roadArea;
  worldModel.scenes.push_back(scene);
  worldModel.timeIndex = frame->SequenceNum();

  ad_rss::situation::SituationVector situationVector;
  bool rss_result = ::ad_rss::core::RssSituationExtraction::extractSituations(
      worldModel, situationVector);
  if (!rss_result) {
    return Status(ErrorCode::PLANNING_ERROR, "ad_rss::extractSituation failed.");
  }

  if (situationVector.size() == 0) {
    return Status(ErrorCode::PLANNING_ERROR, "situationVector unexpected empty.");
  }

  ::ad_rss::state::ResponseStateVector responseStateVector;
  ::ad_rss::core::RssSituationChecking RssCheck;
  rss_result = RssCheck.checkSituations(situationVector, responseStateVector);

  if (!rss_result) {
    return Status(ErrorCode::PLANNING_ERROR, "ad_rss::checkSituation failed.");
  }

  if (responseStateVector.size() == 0) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "responseStateVector unexpected empty.");
  }

  ::ad_rss::state::ResponseState properResponse;
  ::ad_rss::core::RssResponseResolving RssResponse;
  rss_result =
      RssResponse.provideProperResponse(responseStateVector, properResponse);

  if (!rss_result) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "ad_rss::provideProperResponse failed.");
  }

  ::ad_rss::world::AccelerationRestriction accelerationRestriction;
  rss_result = ad_rss::core::RssResponseTransformation::transformProperResponse(
      worldModel, properResponse, accelerationRestriction);

  if (!rss_result) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "ad_rss::transformProperResponse failed.");
  }

  Distance const currentLonDistance =
      situationVector[0].relativePosition.longitudinalDistance;
  Distance safeLonDistance;
  VehicleState const &leadingVehicleState =
      situationVector[0].otherVehicleState;
  VehicleState const &followingVehicleState =
      situationVector[0].egoVehicleState;

  rss_result =
      ::ad_rss::situation::calculateSafeLongitudinalDistanceSameDirection(
          leadingVehicleState, followingVehicleState, safeLonDistance);
  if (!rss_result) {
    return Status(ErrorCode::PLANNING_ERROR,
        "ad_rss::calculateSafeLongitudinalDistanceSameDirection failed");
  }

  if (responseStateVector[0].longitudinalState.isSafe) {
    AERROR << "Task " << Name() << " Distance is RSS-Safe";
    reference_line_info->mutable_rss_info()->set_is_rss_safe(true);
  } else {
    AERROR << "Task " << Name() << " Distance is not RSS-Safe";
    reference_line_info->mutable_rss_info()->set_is_rss_safe(false);
    if (FLAGS_enable_rss_fallback) {
      reference_line_info->mutable_speed_data()->clear();
    }
  }

  reference_line_info->mutable_rss_info()->set_cur_dist_lon(
      static_cast<double>(currentLonDistance));
  reference_line_info->mutable_rss_info()->set_rss_safe_dist_lon(
      static_cast<double>(safeLonDistance));
  reference_line_info->mutable_rss_info()->set_acc_lon_range_minimum(
      static_cast<double>(accelerationRestriction.longitudinalRange.minimum));
  reference_line_info->mutable_rss_info()->set_acc_lon_range_maximum(
      static_cast<double>(accelerationRestriction.longitudinalRange.maximum));
  reference_line_info->mutable_rss_info()->set_acc_lat_left_range_minimum(
      static_cast<double>(accelerationRestriction.lateralLeftRange.minimum));
  reference_line_info->mutable_rss_info()->set_acc_lat_left_range_maximum(
      static_cast<double>(accelerationRestriction.lateralLeftRange.maximum));
  reference_line_info->mutable_rss_info()->set_acc_lat_right_range_minimum(
      static_cast<double>(accelerationRestriction.lateralRightRange.minimum));
  reference_line_info->mutable_rss_info()->set_acc_lat_right_range_maximum(
      static_cast<double>(accelerationRestriction.lateralRightRange.maximum));

  ADEBUG << " longitudinalState.isSafe: "
         << responseStateVector[0].longitudinalState.isSafe;
  ADEBUG << " lateralStateLeft.isSafe: "
         << responseStateVector[0].lateralStateLeft.isSafe;
  ADEBUG << " lateralStateRight.isSafe: "
         << responseStateVector[0].lateralStateRight.isSafe;
  ADEBUG << " TrajectoryLength(): " << reference_line_info->TrajectoryLength();
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

void RssDecider::rss_config_default_dynamics(
    ::ad_rss::world::Dynamics *dynamics) {
  dynamics->alphaLon.accelMax = ::ad_rss::physics::Acceleration(3.5);
  dynamics->alphaLon.brakeMax = ::ad_rss::physics::Acceleration(8);
  dynamics->alphaLon.brakeMin = ::ad_rss::physics::Acceleration(4.);
  dynamics->alphaLon.brakeMinCorrect = ::ad_rss::physics::Acceleration(3.);
  dynamics->alphaLat.accelMax = ::ad_rss::physics::Acceleration(0.2);
  dynamics->alphaLat.brakeMin = ::ad_rss::physics::Acceleration(0.8);
}

void RssDecider::rss_create_ego_object(::ad_rss::world::Object *ego,
                                       double vel_lon, double vel_lat) {
  ego->objectId = 1;
  ego->objectType = ::ad_rss::world::ObjectType::EgoVehicle;
  ego->velocity.speedLon = Speed(vel_lon);
  ego->velocity.speedLat = Speed(vel_lat);
  rss_config_default_dynamics(&(ego->dynamics));
  ego->responseTime = ::ad_rss::physics::Duration(1.);
}

void RssDecider::rss_create_other_object(::ad_rss::world::Object *other,
                                         double vel_lon, double vel_lat) {
  other->objectId = 0;
  other->objectType = ::ad_rss::world::ObjectType::OtherVehicle;
  other->velocity.speedLon = Speed(vel_lon);
  other->velocity.speedLat = Speed(vel_lat);
  rss_config_default_dynamics(&(other->dynamics));
  other->responseTime = ::ad_rss::physics::Duration(2.);
}

}  //  namespace planning
}  //  namespace apollo
