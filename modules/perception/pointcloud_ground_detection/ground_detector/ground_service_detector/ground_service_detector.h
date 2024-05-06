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

#include "modules/perception/common/lidar/scene_manager/ground_service/ground_service.h"
#include "modules/perception/common/lidar/scene_manager/scene_manager.h"
#include "modules/perception/pointcloud_ground_detection/interface/base_ground_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

class GroundServiceDetector : public BaseGroundDetector {
 public:
  GroundServiceDetector() = default;
  ~GroundServiceDetector() = default;
  /**
   * @brief Init ground service detector
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const GroundDetectorInitOptions& options =
                GroundDetectorInitOptions()) override;
  /**
   * @brief Ground service detect
   *
   * @param options
   * @param frame loacation to save point height
   * @return true
   * @return false
   */
  bool Detect(const GroundDetectorOptions& options, LidarFrame* frame) override;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const override { return "GroundServiceDetector"; }

 private:
  GroundServicePtr ground_service_ = nullptr;
  GroundServiceContent ground_service_content_;
  double ground_threshold_ = 0.25;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
