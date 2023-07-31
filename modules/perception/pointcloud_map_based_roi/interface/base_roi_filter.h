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

#include "cyber/common/macros.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct ROIFilterInitOptions : public BaseInitOptions {
};

struct ROIFilterOptions {};

class BaseROIFilter {
 public:
  /**
   * @brief Construct a new Base ROI Filter object
   * 
   */
  BaseROIFilter() = default;

  /**
   * @brief Destroy the Base ROI Filter object
   * 
   */
  virtual ~BaseROIFilter() = default;

  /**
   * @brief Init of Base ROI Filter
   * 
   * @param options ROI filter Init options
   * @return true 
   * @return false 
   */
  virtual bool Init(
      const ROIFilterInitOptions& options = ROIFilterInitOptions()) = 0;

  /**
   * @brief Filter roi points from point cloud
   * 
   * @param options ROI Filter options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  virtual bool Filter(const ROIFilterOptions& options, LidarFrame* frame) = 0;

  /**
   * @brief Name of Base ROI Filter
   * 
   * @return std::string 
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseROIFilter);
};  // class BaseROIFilter

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
