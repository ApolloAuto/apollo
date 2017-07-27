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

#include "modules/prediction/container/container_manager.h"

#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"
#include "modules/common/log.h"

namespace apollo {
namespace prediction {

ContainerManager::ContainerManager() {
  RegisterContainers();
}

ContainerManager::~ContainerManager() {
  containers_.clear();
}

void ContainerManager::RegisterContainers() {
  RegisterContainer("PerceptionObstacles");
  RegisterContainer("Pose");
  CHECK_NOTNULL(containers_["PerceptionObstacles"].get());
}

Container* ContainerManager::mutable_container(const std::string& name) {
  if (containers_.find(name) != containers_.end()) {
    return containers_[name].get();
  } else {
    return nullptr;
  }
}

std::unique_ptr<Container> ContainerManager::CreateContainer(
    const std::string& name) {
  std::unique_ptr<Container> container_ptr(nullptr);
  if (name == "PerceptionObstacles") {
    container_ptr.reset(new ObstaclesContainer());
  } else if (name == "Pose") {
    container_ptr.reset(new PoseContainer());
  }
  return container_ptr;
}

void ContainerManager::RegisterContainer(const std::string& name) {
  containers_[name] = CreateContainer(name);
  ADEBUG << "Container [" << name << "] is registered.";
}

}  // namespace prediction
}  // namespace apollo
