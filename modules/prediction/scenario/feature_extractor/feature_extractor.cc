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

#include <memory>

#include "modules/common/adapters/proto/adapter_config.pb.h"

using apollo::common::adapter::AdapterConfig;
using JunctionInfoPtr = std::shared_ptr<const apollo::hdmap::JunctionInfo>;

namespace apollo {
namespace prediction {

FeatureExtractor::FeatureExtractor() {
  adc_trajectory_container_ = dynamic_cast<ADCTrajectoryContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PLANNING_TRAJECTORY));
  pose_container_ = dynamic_cast<PoseContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::LOCALIZATION));
}

FeatureExtractor::~FeatureExtractor() {
}

void FeatureExtractor::ExtractFeatures() {
  SetADCFeature();
  SetLaneFeature();
  SetJunctionFeature();
  // TODO(all) other processes
}

const ScenarioFeature& FeatureExtractor::scenario_feature() const {
  return scenario_feature_;
}

void FeatureExtractor::SetADCFeature() {
  scenario_feature_.set_speed(pose_container_->GetSpeed());
  scenario_feature_.set_heading(pose_container_->GetTheta());
  // TODO(all) adc acceleration if needed
}

void FeatureExtractor::SetLaneFeature() {
  // TODO(all) implement lane features of ADC
}

void FeatureExtractor::SetJunctionFeature() {
  JunctionInfoPtr junction = adc_trajectory_container_->ADCJunction();
  if (junction != nullptr) {
    scenario_feature_.set_junction_id(junction->id().id());
    scenario_feature_.set_dist_to_junction(
        adc_trajectory_container_->ADCDistanceToJunction());
  }
}

}  // namespace prediction
}  // namespace apollo
