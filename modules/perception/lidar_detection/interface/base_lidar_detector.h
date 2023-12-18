/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct LidarDetectorInitOptions : public BaseInitOptions {
  std::string sensor_name = "velodyne64";
};

struct LidarDetectorOptions {};

class BaseLidarDetector {
 public:
  /**
   * @brief Construct a new Base Lidar Detector object
   * 
   */
  BaseLidarDetector() = default;

  /**
   * @brief Destroy the Base Lidar Detector object
   * 
   */
  virtual ~BaseLidarDetector() = default;

  /**
   * @brief Init the Base Lidar Detector object
   * 
   * @param options lidar detector init options
   * @return true 
   * @return false 
   */
  virtual bool Init(
      const LidarDetectorInitOptions& options = LidarDetectorInitOptions()) = 0;

  /**
   * @brief Detect foreground objects
   * 
   * @param options lidar detector options
   * @param frame lidar frame
   * @return true 
   * @return false 
   */
  virtual bool Detect(const LidarDetectorOptions& options,
                      LidarFrame* frame) = 0;

  /**
   * @brief Name of Base Lidar Detector class
   * 
   * @return std::string 
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseLidarDetector);
};  // class BaseLidarDetector

PERCEPTION_REGISTER_REGISTERER(BaseLidarDetector);
#define PERCEPTION_REGISTER_LIDARDETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseLidarDetector, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
