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
using JunctionInfoPtr = std::shared_ptr<const JunctionInfo>;
using LaneInfoPtr = std::shared_ptr<const LaneInfo>;

namespace apollo {
namespace prediction {

FeatureExtractor::FeatureExtractor() {
  ego_trajectory_containter_ = dynamic_cast<ADCTrajectoryContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PLANNING_TRAJECTORY));

  pose_container_ = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::LOCALIZATION));
}

void FeatureExtractor::ExtractFeatures() {
  if (pose_container_ == nullptr) {
    AERROR << "Null pose container found.";
    return;
  }
  ExtractEgoVehicleFeatures();

  auto ego_trajectory_point = pose_container_->GetPosition();
  if (!ego_trajectory_point.has_x() ||
      !ego_trajectory_point.has_y()) {
    AERROR << "Fail to get ego vehicle position";
    return;
  }
  Vec2d ego_position(ego_trajectory_point.x(), ego_trajectory_point.y());

  auto ptr_ego_lane = GetEgoLane(ego_position);
  ExtractEgoLaneFeatures(ptr_ego_lane, ego_position);

  ExtractNeighborLaneFeatures(ptr_ego_lane, ego_position);

  ExtractFrontJunctionFeatures();

  ExtractObstacleFeatures();

  // TODO(all): add other features
}

const ScenarioFeature& FeatureExtractor::GetScenarioFeatures() const {
  return scenario_feature_;
}

void FeatureExtractor::ExtractEgoVehicleFeatures() {
  // TODO(all): change this to ego_speed and ego_heading
  scenario_feature_.set_speed(pose_container_->GetSpeed());
  scenario_feature_.set_heading(pose_container_->GetTheta());
  // TODO(all): add acceleration if needed
}

void FeatureExtractor::ExtractEgoLaneFeatures(const LaneInfoPtr& ptr_ego_lane,
    const common::math::Vec2d& ego_position) {

  if (ptr_ego_lane == nullptr) {
    AERROR << "Ego vehicle is not on any lane.";
    return;
  }
  scenario_feature_.set_curr_lane_id(ptr_ego_lane->id().id());
  double curr_lane_s = 0.0;
  double curr_lane_l = 0.0;
  ptr_ego_lane->GetProjection(ego_position, &curr_lane_s, &curr_lane_l);
  scenario_feature_.set_curr_lane_s(curr_lane_s);
}

void FeatureExtractor::ExtractNeighborLaneFeatures(
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
    scenario_feature_.set_left_neighbor_lane_id(
        ptr_left_neighbor_lane->id().id());
  }

  auto ptr_right_neighbor_lane = PredictionMap::GetRightNeighborLane(
      ptr_ego_lane, {ego_position.x(), ego_position.y()}, threshold);

  if (ptr_right_neighbor_lane != nullptr) {
    scenario_feature_.set_right_neighbor_lane_id(
        ptr_right_neighbor_lane->id().id());
  }
}

void FeatureExtractor::ExtractFrontJunctionFeatures() {
  if (ego_trajectory_containter_ == nullptr) {
    AERROR << "Null ego trajectory container";
    return;
  }
  JunctionInfoPtr junction = ego_trajectory_containter_->ADCJunction();
  if (junction != nullptr) {
    scenario_feature_.set_junction_id(junction->id().id());
    scenario_feature_.set_dist_to_junction(
        ego_trajectory_containter_->ADCDistanceToJunction());
  }
}

void FeatureExtractor::ExtractObstacleFeatures() {
}

LaneInfoPtr FeatureExtractor::GetEgoLane(const Vec2d& ego_position) const {
  const auto& trajectory =
      ego_trajectory_containter_->adc_trajectory();
  for (const auto& lane_id : trajectory.lane_id()) {
    LaneInfoPtr lane_info =
        HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(lane_id.id()));
    if (lane_info->IsOnLane(ego_position)) {
      return lane_info;
    }
  }
  return nullptr;
}

}  // namespace prediction
}  // namespace apollo
