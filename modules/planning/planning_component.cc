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
#include "modules/planning/planning_component.h"

#include "modules/common/util/message_util.h"

namespace apollo {
namespace planning {

using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::perception::TrafficLightDetection;
using apollo::routing::RoutingResponse;

bool PlanningComponent::Init() {
  planning_base_ = std::unique_ptr<PlanningBase>(new StdPlanning());
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      "/apollo/routing_response",
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        ADEBUG << "Received routing data: run routing callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      "/apollo/perception/traffic_light",
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received chassis data: run chassis callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  routing::RoutingResponse routing_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    routing_copy = routing_;
  }
  perception::TrafficLightDetection traffic_light_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    traffic_light_copy = traffic_light_;
  }

  ADCTrajectory adc_trajectory_pb;
  // TODO(Liangliang) implement here

  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);
  writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));
  return true;
}

}  // namespace planning
}  // namespace apollo
