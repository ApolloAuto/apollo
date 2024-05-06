/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar_tracking/proto/lidar_tracking_component_config.pb.h"

#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/lidar_tracking/classifier/fused_classifier/fused_classifier.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_engine.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;
using onboard::SensorFrameMessage;

class LidarTrackingComponent : public cyber::Component<LidarFrameMessage> {
 public:
  /**
   * @brief Constructor
   */
  LidarTrackingComponent() = default;
  /**
   * @brief Destructor
   */
  virtual ~LidarTrackingComponent() = default;

  /**
   * @brief Initialize the node
   *
   * @return true if initialized
   * @return false
   */
  bool Init() override;
  /**
   * @brief Data callback upon receiving a LidarFrameMessage.
   *
   * @param message LidarFrameMessage
   * @return true if success
   * @return false
   */
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  /**
   * @brief InternalProc, main function is defined here.
   * @param LidarFrameMessage, input message
   * @param SensorFrameMessage, output message
   * @return true if success
   */
  bool InternalProc(const std::shared_ptr<const LidarFrameMessage>& in_message,
                    const std::shared_ptr<SensorFrameMessage>& out_message);

 private:
  BaseMultiTargetTracker* multi_target_tracker_;
  BaseClassifier* fusion_classifier_;

  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> writer_;
};

CYBER_REGISTER_COMPONENT(LidarTrackingComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
