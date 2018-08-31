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

FeatureExtractor::~FeatureExtractor() {
}

void FeatureExtractor::ExtractFeatures() {
  ExtractEgoVehicleFeature();

  ExtractLaneFeature();

  ExtractFrontJunctionFeature();
  // TODO(all) other processes
}

const ScenarioFeature& FeatureExtractor::scenario_feature() const {
  return scenario_feature_;
}

void FeatureExtractor::ExtractEgoVehicleFeature() {
  // TODO(all): change this to ego_speed and ego_heading
  scenario_feature_.set_speed(pose_container_->GetSpeed());
  scenario_feature_.set_heading(pose_container_->GetTheta());
  // TODO(all) adc acceleration if needed
}

void FeatureExtractor::ExtractLaneFeature() {
  LaneInfoPtr curr_lane_info = GetCurrentLane();
  if (curr_lane_info == nullptr) {
    AERROR << "ADC is not on any lane.";
    return;
  }
  auto position = pose_container_->GetPosition();
  scenario_feature_.set_curr_lane_id(curr_lane_info->id().id());
  double curr_lane_s = 0.0;
  double curr_lane_l = 0.0;
  curr_lane_info->GetProjection(Vec2d{position.x(), position.y()},
                                &curr_lane_s, &curr_lane_l);
  scenario_feature_.set_curr_lane_s(curr_lane_s);

  // TODO(all) implement neighbor lane features
}

void FeatureExtractor::ExtractFrontJunctionFeature() {
  JunctionInfoPtr junction = ego_trajectory_containter_->ADCJunction();
  if (junction != nullptr) {
    scenario_feature_.set_junction_id(junction->id().id());
    scenario_feature_.set_dist_to_junction(
        ego_trajectory_containter_->ADCDistanceToJunction());
  }
}

std::shared_ptr<const hdmap::LaneInfo>
FeatureExtractor::GetCurrentLane() const {
  auto position = pose_container_->GetPosition();
  const auto& adc_trajectory =
      ego_trajectory_containter_->adc_trajectory();
  for (const auto& lane_id : adc_trajectory.lane_id()) {
    LaneInfoPtr lane_info =
        HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(lane_id.id()));
    if (lane_info->IsOnLane({position.x(), position.y()})) {
      return lane_info;
    }
  }
  return nullptr;
}

}  // namespace prediction
}  // namespace apollo
