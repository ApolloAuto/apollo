/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/third_party_perception/third_party_perception_base.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace third_party_perception {

using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacles;

ThirdPartyPerception::ThirdPartyPerception(
    apollo::cyber::Node* const node) : node_(node) {

  localization_reader_ =
      node_->CreateReader<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic,
          [this](
              const std::shared_ptr<apollo::localization::LocalizationEstimate>
                  &localization) {
            OnLocalization(*localization.get());
          });

  chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
      FLAGS_chassis_topic,
      [this](const std::shared_ptr<apollo::canbus::Chassis> &chassis) {
          OnChassis(*chassis.get());
      });
}

std::string ThirdPartyPerception::Name() const {
  return FLAGS_third_party_perception_node_name;
}

Status ThirdPartyPerception::Init() {
  return Status::OK();
}

Status ThirdPartyPerception::Start() { return Status::OK(); }

void ThirdPartyPerception::Stop() {}

void ThirdPartyPerception::OnChassis(const Chassis& message) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  chassis_.CopyFrom(message);
}

void ThirdPartyPerception::OnLocalization(const LocalizationEstimate& message) {
  ADEBUG << "Received localization data: run localization callback.";
  std::lock_guard<std::mutex> lock(third_party_perception_mutex_);
  localization_.CopyFrom(message);
}

bool ThirdPartyPerception::Process(PerceptionObstacles* const response) {
  return true;
}

}  // namespace third_party_perception
}  // namespace apollo
