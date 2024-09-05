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

#include <string>

#include "cyber/common/macros.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct DownSampleInitOptions : public BaseInitOptions {};

struct DownSampleOptions {};

class BaseDownSample {
 public:
  /**
   * @brief Construct a new Base Down Sample object
   *
   */
  BaseDownSample() = default;

  /**
   * @brief Destroy the Base Down Sample object
   *
   */
  virtual ~BaseDownSample() = default;

  /**
   * @brief Init of the Base Down Sample object
   *
   * @param options Down Sample options
   * @return true
   * @return false
   */
  virtual bool Init(
      const DownSampleInitOptions& options = DownSampleInitOptions()) = 0;

  /**
   * @brief Down sample pointcloud
   *
   * @param options Down Sample options
   * @param cloud_ptr point cloud to process
   * @return true
   * @return false
   */
  virtual bool Process(const DownSampleOptions& options,
                       base::PointFCloudPtr& cloud_ptr) = 0;

  /**
   * @brief Name of Down Sample
   *
   * @return std::string name
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseDownSample);
};  // class BaseDownSample

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
