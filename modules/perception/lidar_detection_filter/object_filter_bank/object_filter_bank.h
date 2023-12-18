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
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/lidar_detection_filter/interface/base_object_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class ObjectFilterBank {
 public:
  /**
   * @brief Construct a new Object Filter Bank object
   * 
   */
  ObjectFilterBank() = default;

  /**
   * @brief Destroy the Object Filter Bank object
   * 
   */
  ~ObjectFilterBank() = default;

  /**
   * @brief Init of Object Filter Bank
   * 
   * @param options obejct filter inits options
   * @return true 
   * @return false 
   */
  bool Init(const ObjectFilterInitOptions& options = ObjectFilterInitOptions());

  /**
   * @brief Filter the foreground objects and background objects
   * 
   * @param options object filter opitons
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  bool Filter(const ObjectFilterOptions& options, LidarFrame* frame);

  /**
   * @brief Name of Object Filter Bank
   * 
   * @return std::string name
   */
  std::string Name() const { return "ObjectFilterBank"; }

  /**
   * @brief Number of filter banks
   * 
   * @return std::size_t 
   */
  std::size_t Size() const { return filter_bank_.size(); }

 private:
  std::vector<std::shared_ptr<BaseObjectFilter>> filter_bank_;

  DISALLOW_COPY_AND_ASSIGN(ObjectFilterBank);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
