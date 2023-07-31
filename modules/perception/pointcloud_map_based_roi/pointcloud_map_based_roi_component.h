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
#include <numeric>
#include <string>

#include "modules/perception/pointcloud_map_based_roi/proto/pointcloud_map_based_roi_component_config.pb.h"

#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "modules/perception/common/lidar/scene_manager/scene_manager.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/pointcloud_map_based_roi/interface/base_roi_filter.h"
#include "modules/perception/pointcloud_map_based_roi/map_manager/map_manager.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;

class PointCloudMapROIComponent final
    : public cyber::Component<LidarFrameMessage> {
 public:
  /**
   * @brief Construct a new Point Cloud Map ROI Component object
   * 
   */
  PointCloudMapROIComponent() = default;

  /**
   * @brief Destroy the Point Cloud Map ROI Component object
   * 
   */
  virtual ~PointCloudMapROIComponent() = default;

  /**
   * @brief Init of Point Cloud Map ROI Component object
   * 
   * @return true 
   * @return false 
   */
  bool Init() override;

  /**
   * @brief Process of Point Cloud Map ROI Component object
   * 
   * @param message lidar frame message
   * @return true 
   * @return false 
   */
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;

 private:
  bool InternalProc(const std::shared_ptr<LidarFrameMessage>& message);

 private:
  std::shared_ptr<cyber::Writer<LidarFrameMessage>> writer_;
  std::string output_channel_name_;

  bool use_map_manager_;
  MapManager map_manager_;
  std::shared_ptr<BaseROIFilter> roi_filter_;
};

CYBER_REGISTER_COMPONENT(PointCloudMapROIComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
