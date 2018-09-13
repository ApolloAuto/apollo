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

#include "modules/common/adapters/adapter_gflags.h"

#include "modules/common/util/message_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/std_planning.h"

namespace apollo {
namespace planning {

using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;
using apollo::perception::TrafficLightDetection;
using apollo::routing::RoutingResponse;

bool PlanningComponent::Init() {
  AINFO << "Loading gflag from file: " << ConfigFilePath();
  google::SetCommandLineOption("flagfile", ConfigFilePath().c_str());

  planning_base_ = std::unique_ptr<PlanningBase>(new StdPlanning());
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      FLAGS_routing_response_topic,
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        ADEBUG << "Received routing data: run routing callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      FLAGS_traffic_light_detection_topic,
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received chassis data: run chassis callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  if (FLAGS_use_navigation_mode) {
    pad_message_reader_ = node_->CreateReader<PadMessage>(
        FLAGS_planning_pad_topic,
        [this](const std::shared_ptr<PadMessage>& pad_message) {
          ADEBUG << "Received chassis data: run chassis callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          pad_message_.CopyFrom(*pad_message);
        });
  }

  planning_writer_ =
      node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);

  rerouting_writer_ = node_->CreateWriter<routing::RoutingRequest>(
      "/apollo/routing/routing_request");

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  // check and process possible rerouting request
  Rerouting();

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.routing = std::make_shared<routing::RoutingResponse>(routing_);
  }
  perception::TrafficLightDetection traffic_light_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
  }

  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);

  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);
  planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));
  return true;
}

void PlanningComponent::Rerouting() {
  auto* rerouting = PlanningContext::Instance()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(
      std::make_shared<routing::RoutingRequest>(rerouting->routing_request()));
}

}  // namespace planning
}  // namespace apollo
