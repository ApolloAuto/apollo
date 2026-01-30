/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar_cpdet_detection/proto/lidar_cpdet_detection_component_config.pb.h"

#include "cyber/component/component.h"
#include "modules/perception/common/lidar/common/object_builder.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/lidar_cpdet_detection/interface/base_cpdetector.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;

class LidarCPDetectionComponent : public cyber::Component<LidarFrameMessage> {
 public:
  /**
   * @brief Construct a new Lidar Detection Component object
   * 
   */
  LidarCPDetectionComponent() = default;

  /**
   * @brief Destroy the Lidar Detection Component object
   * 
   */
  virtual ~LidarCPDetectionComponent() = default;

  /**
   * @brief Init the Lidar Detection Component object
   * 
   * @return true 
   * @return false 
   */
  bool Init() override;

  /**
   * @brief Detect foreground obejcts using lidar detction models
   * 
   * @param message lidar frame mssage
   * @return true 
   * @return false 
   */
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  /**
   * @brief InternalProc, main function is defined here.
   * @param LidarFrameMessage, input/output message
   * @return true if success
   */
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& in_message);

 private:
  std::string sensor_name_;
  bool use_object_builder_ = true;
  std::shared_ptr<BaseCPDetector> detector_;
  ObjectBuilder builder_;

  std::string output_channel_name_;
  std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;
};

CYBER_REGISTER_COMPONENT(LidarCPDetectionComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
