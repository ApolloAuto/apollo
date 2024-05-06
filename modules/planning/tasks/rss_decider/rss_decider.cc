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

#include "modules/planning/tasks/rss_decider/rss_decider.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

using ad_rss::physics::Distance;
using ad_rss::physics::ParametricValue;
using ad_rss::physics::Speed;
using ad_rss::situation::VehicleState;
using ad_rss::world::Object;
using ad_rss::world::RoadArea;
using ad_rss::world::Scene;

bool RssDecider::Init(const std::string &config_dir, const std::string &name,
                      const std::shared_ptr<DependencyInjector> &injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<RssDeciderConfig>(&config_);
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
    const std::string msg = "Empty path or speed data";
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

  double front_obstacle_distance = config_.rss_max_front_obstacle_distance();
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
  if (front_obstacle_distance >= config_.rss_max_front_obstacle_distance()) {
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

#if RSS_FAKE_INPUT
  nearest_obs_l_start = -4.2608;
  nearest_obs_l_end = -1.42591;
  ego_v_l_start = -1.05554;
  ego_v_l_end = 1.08416;
#endif
  double lane_leftmost = std::max(ego_v_l_end, nearest_obs_l_end);
  double lane_rightmost = std::min(ego_v_l_start, nearest_obs_l_start);
  double lane_width = std::abs(lane_leftmost - lane_rightmost);
  double lane_length = std::max(nearest_obs_s_end, ego_v_s_end);

  rss_world_info.front_obs_dist = front_obstacle_distance;
  rss_world_info.obs_s_start = nearest_obs_s_start;
  rss_world_info.obs_s_end = nearest_obs_s_end;
  rss_world_info.obs_l_start = nearest_obs_l_start;
  rss_world_info.obs_l_end = nearest_obs_l_end;
  rss_world_info.obs_speed = nearest_obs_speed;
  rss_world_info.ego_v_s_start = ego_v_s_start;
  rss_world_info.ego_v_s_end = ego_v_s_end;
  rss_world_info.ego_v_l_start = ego_v_l_start;
  rss_world_info.ego_v_l_end = ego_v_l_end;
  rss_world_info.lane_leftmost = lane_leftmost;
  rss_world_info.lane_rightmost = lane_rightmost;
  rss_world_info.lane_width = lane_width;
  rss_world_info.lane_length = lane_length;

  Object followingObject;
  Object leadingObject;
  RoadArea roadArea;
  Scene scene;

  scene.situationType = ad_rss::situation::SituationType::SameDirection;

  rss_create_other_object(&leadingObject, nearest_obs_speed, 0);

  ad_rss::world::OccupiedRegion occupiedRegion_leading;
  occupiedRegion_leading.segmentId = 0;
  occupiedRegion_leading.lonRange.minimum =
      ParametricValue(nearest_obs_s_start / lane_length);
  occupiedRegion_leading.lonRange.maximum =
      ParametricValue(nearest_obs_s_end / lane_length);
  occupiedRegion_leading.latRange.minimum =
      ParametricValue(std::abs(ego_v_l_end - lane_leftmost) / lane_width);
  occupiedRegion_leading.latRange.maximum =
      ParametricValue(std::abs(ego_v_l_start - lane_leftmost) / lane_width);

  leadingObject.occupiedRegions.push_back(occupiedRegion_leading);

  rss_world_info.OR_front_lon_min =
      static_cast<double>(occupiedRegion_leading.lonRange.minimum);
  rss_world_info.OR_front_lon_max =
      static_cast<double>(occupiedRegion_leading.lonRange.maximum);
  rss_world_info.OR_front_lat_min =
      static_cast<double>(occupiedRegion_leading.latRange.minimum);
  rss_world_info.OR_front_lat_max =
      static_cast<double>(occupiedRegion_leading.latRange.maximum);

  RssDecider::rss_create_ego_object(&followingObject, adc_velocity, 0.0);

  ad_rss::world::OccupiedRegion occupiedRegion_following;
  occupiedRegion_following.segmentId = 0;
  occupiedRegion_following.lonRange.minimum =
      ParametricValue(ego_v_s_start / lane_length);
  occupiedRegion_following.lonRange.maximum =
      ParametricValue(ego_v_s_end / lane_length);
  occupiedRegion_following.latRange.minimum =
      ParametricValue(std::abs(nearest_obs_l_end - lane_leftmost) / lane_width);
  occupiedRegion_following.latRange.maximum = ParametricValue(
      std::abs(nearest_obs_l_start - lane_leftmost) / lane_width);

  followingObject.occupiedRegions.push_back(occupiedRegion_following);

  rss_world_info.adc_vel = adc_velocity;
  rss_world_info.OR_rear_lon_min =
      static_cast<double>(occupiedRegion_following.lonRange.minimum);
  rss_world_info.OR_rear_lon_max =
      static_cast<double>(occupiedRegion_following.lonRange.maximum);
  rss_world_info.OR_rear_lat_min =
      static_cast<double>(occupiedRegion_following.latRange.minimum);
  rss_world_info.OR_rear_lat_max =
      static_cast<double>(occupiedRegion_following.latRange.maximum);

  ad_rss::world::RoadSegment roadSegment;
  ad_rss::world::LaneSegment laneSegment;

  laneSegment.id = 0;
  laneSegment.length.minimum = Distance(lane_length);
  laneSegment.length.maximum = Distance(lane_length);
  laneSegment.width.minimum = Distance(lane_width);
  laneSegment.width.maximum = Distance(lane_width);

  roadSegment.push_back(laneSegment);
  roadArea.push_back(roadSegment);

  rss_world_info.laneSeg_len_min =
      static_cast<double>(laneSegment.length.minimum);
  rss_world_info.laneSeg_len_max =
      static_cast<double>(laneSegment.length.maximum);
  rss_world_info.laneSeg_width_min =
      static_cast<double>(laneSegment.width.minimum);
  rss_world_info.laneSeg_width_max =
      static_cast<double>(laneSegment.width.maximum);

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
    rss_world_info.err_code = "ad_rss::extractSituation failed";
    rss_dump_world_info(rss_world_info);
    return Status(ErrorCode::PLANNING_ERROR, rss_world_info.err_code);
  }

  if (situationVector.empty()) {
    rss_world_info.err_code = "situationVector unexpected empty";
    rss_dump_world_info(rss_world_info);
    return Status(ErrorCode::PLANNING_ERROR, rss_world_info.err_code);
  }

  ::ad_rss::state::ResponseStateVector responseStateVector;
  ::ad_rss::core::RssSituationChecking RssCheck;
  rss_result = RssCheck.checkSituations(situationVector, responseStateVector);

  if (!rss_result) {
    rss_world_info.err_code = "ad_rss::checkSituation failed";
    rss_dump_world_info(rss_world_info);
    return Status(ErrorCode::PLANNING_ERROR, rss_world_info.err_code);
  }

  if (responseStateVector.empty()) {
    rss_world_info.err_code = "responseStateVector unexpected empty";
    rss_dump_world_info(rss_world_info);
    return Status(ErrorCode::PLANNING_ERROR, rss_world_info.err_code);
  }

  ::ad_rss::state::ResponseState properResponse;
  ::ad_rss::core::RssResponseResolving RssResponse;
  rss_result =
      RssResponse.provideProperResponse(responseStateVector, properResponse);

  if (!rss_result) {
    rss_world_info.err_code = "ad_rss::provideProperResponse failed";
    rss_dump_world_info(rss_world_info);
    return Status(ErrorCode::PLANNING_ERROR, rss_world_info.err_code);
  }

  ::ad_rss::world::AccelerationRestriction accelerationRestriction;
  rss_result = ad_rss::core::RssResponseTransformation::transformProperResponse(
      worldModel, properResponse, accelerationRestriction);

  if (!rss_result) {
    rss_world_info.err_code = "ad_rss::transformProperResponse failed";
    rss_dump_world_info(rss_world_info);
    return Status(ErrorCode::PLANNING_ERROR, rss_world_info.err_code);
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
    rss_world_info.err_code =
        ("ad_rss::calculateSafeLongitudinalDistanceSameDirection failed");
    rss_dump_world_info(rss_world_info);
    return Status(ErrorCode::PLANNING_ERROR, rss_world_info.err_code);
  }

  if (responseStateVector[0].longitudinalState.isSafe) {
    ADEBUG << "Task " << Name() << " Distance is RSS-Safe";
    reference_line_info->mutable_rss_info()->set_is_rss_safe(true);
  } else {
    ADEBUG << "Task " << Name() << " Distance is not RSS-Safe";
    reference_line_info->mutable_rss_info()->set_is_rss_safe(false);
    if (config_.enable_rss_fallback()) {
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
  ADEBUG << " is_rss_safe : " << reference_line_info->rss_info().is_rss_safe();
  ADEBUG << " cur_dist_lon: " << reference_line_info->rss_info().cur_dist_lon();
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

void RssDecider::rss_dump_world_info(
    const struct rss_world_model_struct &rss_info) {
  AERROR << " RSS_INFO :"
         << " front_obs_dist: " << rss_info.front_obs_dist
         << " obs_s_start: " << rss_info.obs_s_start
         << " obs_s_end: " << rss_info.obs_s_end
         << " obs_l_start: " << rss_info.obs_l_start
         << " obs_l_end: " << rss_info.obs_l_end
         << " obs_speed: " << rss_info.obs_speed
         << " ego_v_s_start: " << rss_info.ego_v_s_start
         << " ego_v_s_end: " << rss_info.ego_v_s_end
         << " ego_v_l_start: " << rss_info.ego_v_l_start
         << " ego_v_l_end: " << rss_info.ego_v_l_end
         << " lane_leftmost: " << rss_info.lane_leftmost
         << " lane_rightmost: " << rss_info.lane_rightmost
         << " lane_width: " << rss_info.lane_width
         << " lane_length: " << rss_info.lane_length
         << " OR_front_lon_min: " << rss_info.OR_front_lon_min
         << " OR_front_lon_max: " << rss_info.OR_front_lon_max
         << " OR_front_lat_min: " << rss_info.OR_front_lat_min
         << " OR_front_lat_max: " << rss_info.OR_front_lat_max
         << " OR_rear_lon_min: " << rss_info.OR_rear_lon_min
         << " OR_rear_lon_max: " << rss_info.OR_rear_lon_max
         << " OR_rear_lat_min: " << rss_info.OR_rear_lat_min
         << " OR_rear_lat_max: " << rss_info.OR_rear_lat_max
         << " adc_vel: " << rss_info.adc_vel
         << " laneSeg_len_min: " << rss_info.laneSeg_len_min
         << " laneSeg_len_max: " << rss_info.laneSeg_len_max
         << " laneSeg_width_min: " << rss_info.laneSeg_width_min
         << " laneSeg_width_max: " << rss_info.laneSeg_width_max;
}

}  //  namespace planning
}  //  namespace apollo
