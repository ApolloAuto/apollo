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

#include "modules/third_party_perception/third_party_perception_component.h"

#include "boost/algorithm/string.hpp"
#include "boost/format.hpp"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/third_party_perception/proto/third_party_perception_component.pb.h"

DECLARE_string(flagfile);

namespace apollo {
namespace third_party_perception {

bool ThirdPartyPerceptionComponent::Init() {
  apollo::third_party_perception::ThirdPartyPerceptionDevice
          third_party_perception_param;
  if (!GetProtoConfig(&third_party_perception_param)) {
    AINFO << "load third party perception param failed";
    return false;
  }

  std::string device_names_str = third_party_perception_param.device_names();
  std::vector<std::string> device_names;

  boost::algorithm::split(device_names, device_names_str,
                          boost::algorithm::is_any_of(","));
  if (device_names.size() != 1) {
    AERROR << "Now third_party_perception only support one camera device";
    return false;
  } else {
    if (device_names.at(0) == "ThirdPartyPerceptionSmartereye") {
      perception_ = std::make_shared<ThirdPartyPerceptionSmartereye>(
                    node_.get());
    } else if (device_names.at(0) == "ThirdPartyPerceptionMobileye") {
      perception_ = std::make_shared<ThirdPartyPerceptionMobileye>(node_.get());
    } else {
      perception_ = std::make_shared<ThirdPartyPerception>(node_.get());
    }
  }

  if (!perception_->Init().ok()) {
    return false;
  }

  writer_ = node_->CreateWriter<apollo::perception::PerceptionObstacles>(
      FLAGS_perception_obstacle_topic);

  return perception_->Start().ok();
}

bool ThirdPartyPerceptionComponent::Proc() {
  auto response = std::make_shared<apollo::perception::PerceptionObstacles>();
  if (!perception_->Process(response.get())) {
    return false;
  }
  writer_->Write(response);
  return true;
}

}  // namespace third_party_perception
}  // namespace apollo
