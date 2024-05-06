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

#include "modules/perception/pointcloud_ground_detection/proto/pointcloud_ground_detection_component_config.pb.h"

#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/pointcloud_ground_detection/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;

class PointCloudGroundDetectComponent
    : public cyber::Component<LidarFrameMessage> {
 public:
  PointCloudGroundDetectComponent() = default;
  virtual ~PointCloudGroundDetectComponent() = default;
  /**
   * @brief Init pointcloud ground detection
   *
   * @return true
   * @return false
   */
  bool Init() override;
  /**
   * @brief Process of pointcloud ground detection
   *
   * @param message lidar frame message
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& message);

 private:
  std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;
  std::string output_channel_name_;
  BaseGroundDetector* ground_detector_;
};

CYBER_REGISTER_COMPONENT(PointCloudGroundDetectComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
