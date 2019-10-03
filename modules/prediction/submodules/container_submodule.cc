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

#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacles;

ContainerSubmodule::~ContainerSubmodule() {}

std::string ContainerSubmodule::Name() const {
  return FLAGS_container_submodule_name;
}

bool ContainerSubmodule::Init() {
  // TODO(kechxu): implement
  return true;
}

bool ContainerSubmodule::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_message) {
  // TODO(kechxu): implement
  return true;
}

}  // namespace prediction
}  // namespace apollo
