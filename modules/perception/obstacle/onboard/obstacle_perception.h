/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBSTACLE_PERCEPTION_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBSTACLE_PERCEPTION_H_

#include <memory>
#include <vector>

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/fusion/interface/base_fusion.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/obstacle/onboard/lidar_process.h"
#include "modules/perception/obstacle/onboard/sensor_raw_frame.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"

namespace apollo {
namespace perception {

enum ObstacleShowType {
  SHOW_LIDAR = 0,
  SHOW_RADAR = 1,
  SHOW_FUSED = 2,
  MAX_SHOW_TYPE
};

class ObstaclePerception {
 public:
  /**
   * @brief Construct
   */
  ObstaclePerception();

  /**
   * @brief Destruct
   */
  ~ObstaclePerception();

  /**
   * @brief Initialize configuration
   * @return True if initialize successfully, false otherwise
   */
  bool Init();

  /**
   * @brief The main process to detect, recognize and track objects
   * based on different kinds of sensor data.
   * @param frame Sensor data of one single frame
   * @param out_objects The obstacle perception results
   * @return True if process successfully, false otherwise
   */
  bool Process(SensorRawFrame* frame,
               std::vector<std::shared_ptr<Object>>* out_objects);

 private:
  /**
   * @brief Regist all algorithms for each module
   */
  void RegistAllAlgorithm();

  /// obstacle detector
  std::unique_ptr<LidarProcess> lidar_perception_;
  std::unique_ptr<BaseRadarDetector> radar_detector_;
  std::unique_ptr<BaseFusion> fusion_;

  /// visualization
  std::unique_ptr<OpenglVisualizer> frame_visualizer_ = nullptr;
  ObstacleShowType obstacle_show_type_;
  FrameContent frame_content_;
  bool lidar_pose_inited_;

  DISALLOW_COPY_AND_ASSIGN(ObstaclePerception);
};  // class ObstaclePerception

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBSTACLE_PERCEPTION_H_
