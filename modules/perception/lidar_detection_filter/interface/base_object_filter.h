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

struct ObjectFilterInitOptions : public BaseInitOptions{
  std::string sensor_name = "velodyne64";
};

struct ObjectFilterOptions {};

class BaseObjectFilter {
 public:
  /**
   * @brief Construct a new Base Object Filter object
   * 
   */
  BaseObjectFilter() = default;

  /**
   * @brief Destroy the Base Object Filter object
   * 
   */
  virtual ~BaseObjectFilter() = default;

  /**
   * @brief Init of the Base Object Filter object
   * 
   * @param options object filter options
   * @return true 
   * @return false 
   */
  virtual bool Init(
      const ObjectFilterInitOptions& options = ObjectFilterInitOptions()) = 0;

  /**
   * @brief Filter objects
   * 
   * @param options object filter options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  virtual bool Filter(const ObjectFilterOptions& options,
                      LidarFrame* frame) = 0;

  /**
   * @brief Name of Object Filter
   * 
   * @return std::string name
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseObjectFilter);
};  // class BaseObjectFilter

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
