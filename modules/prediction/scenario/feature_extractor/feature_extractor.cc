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

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using common::adapter::AdapterConfig;
using common::math::Vec2d;
using hdmap::JunctionInfo;
using hdmap::LaneInfo;
using LaneInfoPtr = std::shared_ptr<const LaneInfo>;
using JunctionInfoPtr = std::shared_ptr<const JunctionInfo>;

EnvironmentFeatures FeatureExtractor::ExtractEnvironmentFeatures(
    ContainerManager* container_manager) {
  EnvironmentFeatures environment_features;

  auto ego_state_container = container_manager->GetContainer<PoseContainer>(
      AdapterConfig::LOCALIZATION);

  if (ego_state_container == nullptr ||
      ego_state_container->ToPerceptionObstacle() == nullptr) {
    AERROR << "Null ego state container found or "
              "the container pointer is nullptr";
    return environment_features;
  }

  auto ptr_ego_state = ego_state_container->ToPerceptionObstacle();
  if (!ptr_ego_state->has_position() || !ptr_ego_state->position().has_x() ||
      !ptr_ego_state->position().has_y() || !ptr_ego_state->has_theta() ||
      !ptr_ego_state->has_velocity() || !ptr_ego_state->velocity().has_x() ||
      !ptr_ego_state->velocity().has_y()) {
    AERROR << "Incomplete ego pose information.";
    return environment_features;
  }

  if (std::isnan(ptr_ego_state->position().x()) ||
      std::isnan(ptr_ego_state->position().y()) ||
      std::isnan(ptr_ego_state->theta()) ||
      std::isnan(ptr_ego_state->velocity().x()) ||
      std::isnan(ptr_ego_state->velocity().y())) {
    AERROR << "nan found in ego state";
    return environment_features;
  }

  Vec2d ego_position(ptr_ego_state->position().x(),
                     ptr_ego_state->position().y());

  Vec2d ego_velocity(ptr_ego_state->velocity().x(),
                     ptr_ego_state->velocity().y());

  environment_features.set_ego_speed(ego_velocity.Length());
  environment_features.set_ego_heading(ptr_ego_state->theta());

  auto ptr_ego_lane =
      GetEgoLane(ptr_ego_state->position(), ptr_ego_state->theta());

  ExtractEgoLaneFeatures(&environment_features, ptr_ego_lane, ego_position);

  ExtractNeighborLaneFeatures(&environment_features, ptr_ego_lane,
                              ego_position);

  ExtractFrontJunctionFeatures(&environment_features, container_manager);

  return environment_features;
}

void FeatureExtractor::ExtractEgoLaneFeatures(
    EnvironmentFeatures* ptr_environment_features,
    const LaneInfoPtr& ptr_ego_lane, const common::math::Vec2d& ego_position) {
  if (ptr_ego_lane == nullptr) {
    ADEBUG << "Ego vehicle is not on any lane.";
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
    std::for_each(
        reverse_lanes.begin(), reverse_lanes.end(),
        [&ptr_environment_features](decltype(*reverse_lanes.begin())& t) {
          ptr_environment_features->AddNonneglectableReverseLanes(t.id());
        });
  }
}

void FeatureExtractor::ExtractNeighborLaneFeatures(
    EnvironmentFeatures* ptr_environment_features,
    const LaneInfoPtr& ptr_ego_lane, const Vec2d& ego_position) {
  if (ptr_ego_lane == nullptr) {
    ADEBUG << "Ego vehicle is not on any lane.";
    return;
  }

  auto ptr_left_neighbor_lane = PredictionMap::GetLeftNeighborLane(
      ptr_ego_lane, {ego_position.x(), ego_position.y()},
      FLAGS_lane_distance_threshold);

  if (ptr_left_neighbor_lane != nullptr) {
    double left_neighbor_lane_s = 0.0;
    double left_neighbor_lane_l = 0.0;
    ptr_left_neighbor_lane->GetProjection(ego_position, &left_neighbor_lane_s,
                                          &left_neighbor_lane_l);
    ptr_environment_features->SetLeftNeighborLane(
        ptr_left_neighbor_lane->id().id(), left_neighbor_lane_s);
  }

  auto ptr_right_neighbor_lane = PredictionMap::GetRightNeighborLane(
      ptr_ego_lane, {ego_position.x(), ego_position.y()},
      FLAGS_lane_distance_threshold);

  if (ptr_right_neighbor_lane != nullptr) {
    double right_neighbor_lane_s = 0.0;
    double right_neighbor_lane_l = 0.0;
    ptr_right_neighbor_lane->GetProjection(ego_position, &right_neighbor_lane_s,
                                           &right_neighbor_lane_l);
    ptr_environment_features->SetRightNeighborLane(
        ptr_right_neighbor_lane->id().id(), right_neighbor_lane_s);
  }
}

void FeatureExtractor::ExtractFrontJunctionFeatures(
    EnvironmentFeatures* ptr_environment_features,
    ContainerManager* container_manager) {
  auto ego_trajectory_container =
      container_manager->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  if (ego_trajectory_container == nullptr) {
    AERROR << "Null ego trajectory container";
    return;
  }
  JunctionInfoPtr junction = ego_trajectory_container->ADCJunction();
  if (junction == nullptr) {
    return;
  }
  // Only consider junction have overlap with signal or stop_sign
  bool need_consider = FLAGS_enable_all_junction;
  for (const auto& overlap_id : junction->junction().overlap_id()) {
    if (PredictionMap::OverlapById(overlap_id.id()) != nullptr) {
      for (const auto& object :
           PredictionMap::OverlapById(overlap_id.id())->overlap().object()) {
        if (object.has_signal_overlap_info() ||
            object.has_stop_sign_overlap_info()) {
          need_consider = true;
        }
      }
    }
  }
  if (need_consider) {
    ptr_environment_features->SetFrontJunction(
        junction->id().id(), ego_trajectory_container->ADCDistanceToJunction());
  }
}

LaneInfoPtr FeatureExtractor::GetEgoLane(const common::Point3D& position,
                                         const double heading) {
  common::PointENU position_enu;
  position_enu.set_x(position.x());
  position_enu.set_y(position.y());
  position_enu.set_z(position.z());

  return PredictionMap::GetMostLikelyCurrentLane(
      position_enu, FLAGS_lane_distance_threshold, heading,
      FLAGS_lane_angle_difference_threshold);
}

}  // namespace prediction
}  // namespace apollo
