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

#ifndef MODEULES_PERCEPTION_OBSTACLE_ONBOARD_OBSTACLE_PERCEPTION_H_
#define MODEULES_PERCEPTION_OBSTACLE_ONBOARD_OBSTACLE_PERCEPTION_H_

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

class ObstaclePerception {
 public:
  ObstaclePerception();
  ~ObstaclePerception();

  bool Init();

  bool Process(SensorRawFrame* frame, std::vector<ObjectPtr>& out_objects);

  void SetGlobalOffset(const Eigen::Vector3d& global_offset);

 private:
  std::unique_ptr<LidarProcess> lidar_perception_;
  std::unique_ptr<BaseRadarDetector> radar_detector_;
  std::unique_ptr<BaseFusion> fusion_;
  std::unique_ptr<OpenglVisualizer> frame_visualizer_ = nullptr;
  FrameContent frame_content_;
  bool initialized_;
  Eigen::Vector3d global_offset_;
  DISALLOW_COPY_AND_ASSIGN(ObstaclePerception);
};  // class ObstaclePerception

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_OBSTACLE_ONBOARD_OBSTACLE_PERCEPTION_H_