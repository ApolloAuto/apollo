/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/radar_detection/interface/base_radar_obstacle_perception.h"

namespace apollo {
namespace perception {
namespace radar {

class RadarObstaclePerception : public BaseRadarObstaclePerception {
 public:
  /**
   * @brief Radar object detection processor, which include 3 steps:
   * object detection, ROI filtering and tracking.
   */
  RadarObstaclePerception() = default;
  virtual ~RadarObstaclePerception() = default;

  /**
   * @brief Init the radar object detection processor
   *  includes config, create classes, etc.
   *
   * @param options init options
   * @return true
   * @return false
   */
  bool Init(const PerceptionInitOptions &options) override;

  /**
   * @brief Process the radar raw data obtained from radar driver, and get
   * targets after tracking.
   *
   * @param corrected_obstacles raw data obtained from radar driver
   * @param options processing options
   * @param objects targets after tracking
   * @return true
   * @return false
   */
  bool Perceive(const drivers::ContiRadar& corrected_obstacles,
                const RadarPerceptionOptions& options,
                std::vector<base::ObjectPtr>* objects) override;

  /**
   * @brief The name of the radar object detection processor
   *
   * @return std::string
   */
  std::string Name() const override;

 private:
  std::shared_ptr<BaseDetector> detector_;
  std::shared_ptr<BaseRoiFilter> roi_filter_;
  std::shared_ptr<BaseTracker> tracker_;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
