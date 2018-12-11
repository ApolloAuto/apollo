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
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/v2x/proto/v2x_carstatus.pb.h"
#include "modules/v2x/proto/v2x_traffic_light.pb.h"

namespace apollo {
namespace v2x {

class ObuInterFaceBase {
 public:
  ObuInterFaceBase() {}
  virtual ~ObuInterFaceBase() {}

 public:
  virtual bool InitialServer() = 0;
  virtual bool InitialClient() = 0;
  virtual void GetV2xObstaclesFromObu(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg) {}
  virtual void GetV2xTrafficLightFromObu(
      const std::shared_ptr<IntersectionTrafficLightData> &msg) {}
  virtual void SendCarStatusToObu(const std::shared_ptr<CarStatus> &msg) {}
  virtual void SendObstaclesToObu(
      const std::shared_ptr<apollo::perception::PerceptionObstacles> &msg) {}
};

}  // namespace v2x
}  // namespace apollo
