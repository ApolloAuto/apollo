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

#include <memory>
#include <string>
#include <vector>
#include "modules/perception/common/base/radar_point_cloud.h"

#include "modules/perception/radar4d_detection/interface/base_obstacle_perception.h"
#include "modules/perception/radar4d_detection/lib/object_builder/object_builder.h"
#include "modules/perception/radar4d_detection/lib/classifier/fused_classifier.h"
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_engine.h"

namespace apollo {
namespace perception {
namespace radar4d {

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
   * @brief Process radar frame with preprocessed point cloud, and get
   * targets after detection and tracking.
   *
   * @param frame radar frame with preprocessed point cloud
   * @param options processing options
   * @param objects targets after detection and tracking
   * @return true
   * @return false
   */
  bool Perceive(RadarFrame* frame,
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
  ObjectBuilder builder_;
  std::shared_ptr<BaseRoiFilter> roi_filter_;
  BaseMultiTargetTracker* multi_target_tracker_;
  BaseClassifier* fusion_classifier_;
  bool enable_roi_filter_ = false;
};

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
