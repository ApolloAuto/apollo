/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/perception.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/perception/common/perception_gflags.h"
#include "third_party/ros/include/ros/ros.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;

std::string Perception::Name() const {
  return "perception";
}

Status Perception::Init() {
  AdapterManager::Init();
  return Status::OK();
}

Status Perception::Start() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  spinner.stop();
  ros::Rate loop_rate(FLAGS_perception_loop_rate);
  while (ros::ok()) {
    AdapterManager::Observe();
    PerceptionObstacles perceptionObstacles;
    AdapterManager::FillPerceptionObstaclesHeader(
        Name(), perceptionObstacles.mutable_header());
    AdapterManager::PublishPerceptionObstacles(perceptionObstacles);

    TrafficLightDetection trafficLightDetection;
    AdapterManager::FillTrafficLightDetectionHeader(
        Name(), trafficLightDetection.mutable_header());
    AdapterManager::PublishTrafficLightDetection(trafficLightDetection);
  }
  return Status::OK();
}

void Perception::Stop() {}

}  // namespace perception
}  // namespace apollo
