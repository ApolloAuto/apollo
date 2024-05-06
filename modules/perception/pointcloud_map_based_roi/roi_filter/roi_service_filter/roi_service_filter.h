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

#include <string>

#include "modules/perception/common/lidar/scene_manager/roi_service/roi_service.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/pointcloud_map_based_roi/interface/base_roi_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class ROIServiceFilter : public BaseROIFilter {
 public:
  /**
   * @brief Construct a new ROIServiceFilter object
   * 
   */
  ROIServiceFilter() = default;

  /**
   * @brief Destroy the ROIServiceFilter object
   * 
   */
  ~ROIServiceFilter() = default;

  /**
   * @brief Init of ROIServiceFilter
   * 
   * @param options roi filter init options
   * @return true 
   * @return false 
   */
  bool Init(const ROIFilterInitOptions& options) override;

  /**
   * @brief filter point cloud outside of roi
   * 
   * @param options roi filter options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Filter(const ROIFilterOptions& options, LidarFrame* frame) override;

  /**
   * @brief Name of ROIServiceFilter
   * 
   * @return std::string name
   */
  std::string Name() const override { return "ROIServiceFilter"; }

 private:
  ROIServicePtr roi_service_ = nullptr;
  ROIServiceContent roi_service_content_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::perception::lidar::ROIServiceFilter, BaseROIFilter)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
