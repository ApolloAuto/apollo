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

#include "modules/map/relative_map/relative_map_component.h"

#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace relative_map {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;

bool RelativeMapComponent::Init() {
  InitReaders();
  return relative_map_.Init().ok() && relative_map_.Start().ok();
}

bool RelativeMapComponent::Proc() {
  auto map_msg = std::make_shared<MapMsg>();
  if (!relative_map_.Process(map_msg.get())) {
    return false;
  }
  common::util::FillHeader(node_->Name(), map_msg.get());
  relative_map_writer_->Write(map_msg);
  return true;
}

bool RelativeMapComponent::InitReaders() {
  perception_reader_ = node_->CreateReader<PerceptionObstacles>(
      FLAGS_perception_obstacle_topic,
      [this](const std::shared_ptr<PerceptionObstacles>& perception_obstacles) {
        ADEBUG << "Received perception data: run perception callback.";
        relative_map_.OnPerception(*perception_obstacles.get());
      });

  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis>& chassis) {
        ADEBUG << "Received chassis data: run chassis callback.";
        relative_map_.OnChassis(*chassis.get());
      });

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<LocalizationEstimate>& localization) {
        ADEBUG << "Received chassis data: run chassis callback.";
        relative_map_.OnLocalization(*localization.get());
      });

  navigation_reader_ = node_->CreateReader<NavigationInfo>(
      FLAGS_navigation_topic,
      [this](const std::shared_ptr<NavigationInfo>& navigation_info) {
        ADEBUG << "Received chassis data: run chassis callback.";
        relative_map_.OnNavigationInfo(*navigation_info.get());
      });

  relative_map_writer_ = node_->CreateWriter<MapMsg>(FLAGS_relative_map_topic);
  return true;
}

}  // namespace relative_map
}  // namespace apollo
