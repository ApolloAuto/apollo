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
#pragma once

#include <memory>
#include <string>
#include <boost/circular_buffer.hpp>

#include "cyber/cyber.h"
#include "modules/perception/common/camera/common/pose.h"
#include "modules/perception/common/camera/common/trafficlight_frame.h"
#include "modules/perception/common/onboard/inner_component_messages/inner_component_messages.h"

namespace apollo {
namespace perception {
namespace onboard {

class TrafficDetectMessage {
 public:
  TrafficDetectMessage() : traffic_light_frame_(nullptr) {
    type_name_ = "TrafficDetectMessage";
  }

  ~TrafficDetectMessage() = default;

  std::string GetTypeName() const { return type_name_; }

  TrafficDetectMessage* New() const { return new TrafficDetectMessage; }

 public:
  double timestamp_ = 0.0;
  std::string type_name_;
  apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;
  ::google::protobuf::RepeatedPtrField<apollo::hdmap::Curve> stoplines_;
  std::shared_ptr<camera::CarPose> carpose_;
  std::shared_ptr<camera::TrafficLightFrame> traffic_light_frame_;
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
