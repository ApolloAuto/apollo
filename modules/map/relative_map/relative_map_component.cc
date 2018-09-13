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

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/map/relative_map/relative_map_component.h"

namespace apollo {
namespace relative_map {

using apollo::perception::PerceptionObstacles;
using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;

bool RelativeMapComponent::Init() {
  AINFO << "Loading gflag from file: " << ConfigFilePath();
  google::SetCommandLineOption("flagfile", ConfigFilePath().c_str());

  relative_map_writer_ =
      node_->CreateWriter<MapMsg>(FLAGS_relative_map_topic);

  return true;
}

bool RelativeMapComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& prediction_obstacles,
    const std::shared_ptr<Chassis>& chassis,
    const std::shared_ptr<LocalizationEstimate>& localization_estimate) {
  //TODO(yifei) migrate implementation
  return true;
}

}  // namespace routing
}  // namespace apollo

