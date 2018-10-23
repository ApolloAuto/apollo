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

#include "modules/prediction/scenario/feature_extractor/feature_extractor.h"

#include <string>

#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/math/vec2d.h"
#include "modules/map/hdmap/hdmap_util.h"

using apollo::common::adapter::AdapterConfig;
using apollo::common::math::Vec2d;
using apollo::planning::ADCTrajectory;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfo;
using apollo::hdmap::JunctionInfo;
using apollo::perception::PerceptionObstacle;
using JunctionInfoPtr = std::shared_ptr<const JunctionInfo>;
using LaneInfoPtr = std::shared_ptr<const LaneInfo>;

namespace apollo {
namespace prediction {

EnvironmentFeatures FeatureExtractor::ExtractEnvironmentFeatures() {
  EnvironmentFeatures environment_features;

  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::Instance()->GetContainer(
          AdapterConfig::LOCALIZATION));

  if (pose_container == nullptr) {
    AERROR << "Null pose container found.";
    return environment_features;
  }
  ExtractEgoVehicleFeatures(&environment_features);

  const PerceptionObstacle* pose_ptr = pose_container->ToPerceptionObstacle();
  if (pose_ptr == nullptr) {
    AERROR << "Null pose pointer, skip extracting environment features.";
    return environment_features;
  }

  auto ego_trajectory_point = pose_ptr->position();
  if (!ego_trajectory_point.has_x() ||
      !ego_trajectory_point.has_y()) {
    AERROR << "Fail to get ego vehicle position";
    return environment_features;
  }
  Vec2d ego_position(ego_trajectory_point.x(), ego_trajectory_point.y());

  auto ptr_ego_lane = GetEgoLane(ego_position);

  ExtractEgoLaneFeatures(&environment_features,
      ptr_ego_lane, ego_position);

  ExtractNeighborLaneFeatures(
      &environment_features, ptr_ego_lane, ego_position);

  ExtractFrontJunctionFeatures(&environment_features);

  ExtractObstacleFeatures(&environment_features);

  // TODO(all): add other features

  return environment_features;
}

void FeatureExtractor::ExtractEgoVehicleFeatures(
    EnvironmentFeatures* ptr_environment_features) {
  PoseContainer* pose_container = dynamic_cast<PoseContainer*>(
      ContainerManager::Instance()->GetContainer(
          AdapterConfig::LOCALIZATION));
  const PerceptionObstacle* pose_ptr = pose_container->ToPerceptionObstacle();
  if (pose_ptr == nullptr) {
    AERROR << "Null pose pointer";
    return;
  }
  // TODO(all): change this to ego_speed and ego_heading
  double pose_speed = std::hypot(pose_ptr->velocity().x(),
                                 pose_ptr->velocity().y());
  ptr_environment_features->set_ego_speed(pose_speed);
  ptr_environment_features->set_ego_heading(pose_ptr->theta());
  // TODO(all): add acceleration if needed
}

void FeatureExtractor::ExtractEgoLaneFeatures(
    EnvironmentFeatures* ptr_environment_features,
    const LaneInfoPtr& ptr_ego_lane,
    const common::math::Vec2d& ego_position) {

  if (ptr_ego_lane == nullptr) {
    AERROR << "Ego vehicle is not on any lane.";
    return;
  }
  ADEBUG << "Ego vehicle is on lane [" << ptr_ego_lane->id().id() << "]";
  double curr_lane_s = 0.0;
  double curr_lane_l = 0.0;
  ptr_ego_lane->GetProjection(ego_position, &curr_lane_s, &curr_lane_l);
  ptr_environment_features->SetEgoLane(ptr_ego_lane->id().id(), curr_lane_s);

  double threshold = 1.0;
  auto ptr_left_neighbor_lane = PredictionMap::GetLeftNeighborLane(
      ptr_ego_lane, {ego_position.x(), ego_position.y()}, threshold);

  if (ptr_left_neighbor_lane == nullptr &&
      ptr_ego_lane->lane().has_left_boundary() &&
      ptr_ego_lane->lane().left_boundary().boundary_type_size() != 0 &&
      ptr_ego_lane->lane().left_boundary().boundary_type(0).types_size() != 0 &&
      ptr_ego_lane->lane().left_boundary().boundary_type(0).types(0) !=
      hdmap::LaneBoundaryType::CURB) {
    const auto& reverse_lanes =
        ptr_ego_lane->lane().left_neighbor_reverse_lane_id();
    std::for_each(reverse_lanes.begin(), reverse_lanes.end(),
        [&ptr_environment_features](decltype(*reverse_lanes.begin())& t) {
      ptr_environment_features->AddNonneglectableReverseLanes(t.id());
    });
  }
}

void FeatureExtractor::ExtractNeighborLaneFeatures(
    EnvironmentFeatures* ptr_environment_features,
    const LaneInfoPtr& ptr_ego_lane, const Vec2d& ego_position) {

  if (ptr_ego_lane == nullptr) {
    AERROR << "Ego vehicle is not on any lane.";
    return;
  }

  // TODO(all): make this a gflag
  double threshold = 3.0;

  auto ptr_left_neighbor_lane = PredictionMap::GetLeftNeighborLane(
      ptr_ego_lane, {ego_position.x(), ego_position.y()}, threshold);

  if (ptr_left_neighbor_lane != nullptr) {
    double left_neighbor_lane_s = 0.0;
    double left_neighbor_lane_l = 0.0;
    ptr_left_neighbor_lane->GetProjection(ego_position,
        &left_neighbor_lane_s, &left_neighbor_lane_l);
    ptr_environment_features->SetLeftNeighborLane(
        ptr_left_neighbor_lane->id().id(), left_neighbor_lane_s);
  }

  auto ptr_right_neighbor_lane = PredictionMap::GetRightNeighborLane(
      ptr_ego_lane, {ego_position.x(), ego_position.y()}, threshold);
  if (ptr_right_neighbor_lane != nullptr) {
    double right_neighbor_lane_s = 0.0;
    double right_neighbor_lane_l = 0.0;
    ptr_right_neighbor_lane->GetProjection(ego_position,
        &right_neighbor_lane_s, &right_neighbor_lane_l);
    ptr_environment_features->SetRightNeighborLane(
        ptr_right_neighbor_lane->id().id(), right_neighbor_lane_s);
  }
}

void FeatureExtractor::ExtractFrontJunctionFeatures(
    EnvironmentFeatures* ptr_environment_features) {
  ADCTrajectoryContainer* ego_trajectory_container =
      dynamic_cast<ADCTrajectoryContainer*>(
          ContainerManager::Instance()->GetContainer(
              AdapterConfig::PLANNING_TRAJECTORY));
  if (ego_trajectory_container == nullptr) {
    AERROR << "Null ego trajectory container";
    return;
  }
  JunctionInfoPtr junction = ego_trajectory_container->ADCJunction();
  if (junction != nullptr) {
    ptr_environment_features->SetFrontJunction(junction->id().id(),
        ego_trajectory_container->ADCDistanceToJunction());
  }
}

void FeatureExtractor::ExtractObstacleFeatures(
    EnvironmentFeatures* ptr_environment_features) {
}

LaneInfoPtr FeatureExtractor::GetEgoLane(const Vec2d& ego_position) {
  ADCTrajectoryContainer* ego_trajectory_container =
      dynamic_cast<ADCTrajectoryContainer*>(
          ContainerManager::Instance()->GetContainer(
              AdapterConfig::PLANNING_TRAJECTORY));
  const auto& trajectory =
      ego_trajectory_container->adc_trajectory();
  for (const auto& lane_id : trajectory.lane_id()) {
    LaneInfoPtr lane_info =
        HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(lane_id.id()));

    if (lane_info == nullptr) {
      continue;
    }

    if (lane_info->IsOnLane(ego_position)) {
      return lane_info;
    }
  }
  return nullptr;
}

}  // namespace prediction
}  // namespace apollo
