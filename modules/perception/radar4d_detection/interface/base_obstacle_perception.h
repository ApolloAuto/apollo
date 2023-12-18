/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <string>
#include <vector>
#include "modules/perception/common/base/radar_point_cloud.h"
#include "modules/perception/common/radar/common/radar_frame.h"

#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/radar4d_detection/interface/base_detector.h"
#include "modules/perception/radar4d_detection/interface/base_roi_filter.h"

namespace apollo {
namespace perception {
namespace radar4d {

struct PerceptionInitOptions : public BaseInitOptions {
  // reserved
};

struct RadarPerceptionOptions {
  DetectorOptions detector_options;
  RoiFilterOptions roi_filter_options;
  std::string sensor_name;
};

class BaseRadarObstaclePerception {
 public:
  /**
   * @brief Construct a new Base Radar Obstacle Perception object
   *
   */
  BaseRadarObstaclePerception() = default;
  virtual ~BaseRadarObstaclePerception() = default;

  /**
   * @brief Init radar object detection processor
   *
   * @param options init options
   * @return true
   * @return false
   */
  virtual bool Init(const PerceptionInitOptions& options) = 0;

  /**
   * @brief Process radar frame with preprocessed point cloud, and get
   * targets after detection and tracking.
   *
   * @param frame radar frame with preprocessed point cloud
   * @param options processing options
   * @param objects targets after detection and tracking
   * @return true
   * @return false
   */
  virtual bool Perceive(RadarFrame* frame,
                        const RadarPerceptionOptions& options,
                        std::vector<base::ObjectPtr>* objects) = 0;

  /**
   * @brief The name of the radar object detection processor
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;
};

PERCEPTION_REGISTER_REGISTERER(BaseRadarObstaclePerception);
#define PERCEPTION_REGISTER_RADAR_OBSTACLE_PERCEPTION(name) \
  PERCEPTION_REGISTER_CLASS(BaseRadarObstaclePerception, name)

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
