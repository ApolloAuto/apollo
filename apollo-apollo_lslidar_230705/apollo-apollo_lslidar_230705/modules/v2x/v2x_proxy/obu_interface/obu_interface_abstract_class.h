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

/**
 * @file obu_interface_abstract_class.h
 * @brief define v2x proxy module and onboard unit interface base class
 */

#pragma once

#include <chrono>
#include <memory>

#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/v2x/proto/v2x_car_status.pb.h"
#include "modules/v2x/proto/v2x_obstacles.pb.h"
#include "modules/v2x/proto/v2x_obu_rsi.pb.h"
#include "modules/common_msgs/v2x_msgs/v2x_traffic_light.pb.h"

#include "cyber/cyber.h"

namespace apollo {
namespace v2x {

class ObuInterFaceBase {
 public:
  ObuInterFaceBase() {}

  virtual ~ObuInterFaceBase() {}

 public:
  virtual bool InitialServer() = 0;

  virtual bool InitialClient() = 0;

  virtual void GetV2xTrafficLightFromObu(
      std::shared_ptr<::apollo::v2x::obu::ObuTrafficLight> *msg) {}

  virtual void SendCarStatusToObu(
      const std::shared_ptr<::apollo::v2x::CarStatus> &msg) {}

  virtual void GetV2xRsiFromObu(
      std::shared_ptr<::apollo::v2x::obu::ObuRsi> *msg) {}
  virtual void GetV2xObstaclesFromObu(
      std::shared_ptr<::apollo::v2x::V2XObstacles> *msg) {}
};

}  // namespace v2x
}  // namespace apollo
