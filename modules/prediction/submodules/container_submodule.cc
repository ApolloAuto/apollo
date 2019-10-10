/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/submodules/container_submodule.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/prediction/common/message_process.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;

ContainerSubmodule::~ContainerSubmodule() {}

std::string ContainerSubmodule::Name() const {
  return FLAGS_container_submodule_name;
}

bool ContainerSubmodule::Init() {
  if (!MessageProcess::InitContainers()) {
    return false;
  }

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic, nullptr);

  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          FLAGS_localization_topic, nullptr);

  // TODO(kechxu) init the cyber writer

  return true;
}

bool ContainerSubmodule::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_message) {
  MessageProcess::ContainerProcess(*perception_message);
  // TODO(kechxu): implement the writer
  return true;
}

}  // namespace prediction
}  // namespace apollo
