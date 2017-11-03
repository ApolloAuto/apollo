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

#ifndef MODEULES_PERCEPTION_OBSTACLE_ONBOARD_RADAR_PROCESS_H_
#define MODEULES_PERCEPTION_OBSTACLE_ONBOARD_RADAR_PROCESS_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"
#include "modules/perception/obstacle/radar/detector/modest/modest_radar_detector.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/drivers/proto/sensor_radar.pb.h"

namespace apollo {
namespace perception {

class RadarProcess {
 public:
  RadarProcess() = default;
  ~RadarProcess() = default;

  bool Init();
  bool IsInit() { return inited_; }
  bool Process(const RadarObsArray& radar_obs_proto);

  bool GeneratePbMsg(PerceptionObstacles* obstacles);

  std::vector<ObjectPtr> GetObjects() { return objects_; }

 private:
  void RegistAllAlgorithm();
  bool InitFrameDependence();
  bool InitAlgorithmPlugin();

  bool GetRadarTrans(const double query_time, Eigen::Matrix4d* trans);

  bool inited_ = false;
  double timestamp_;
  common::ErrorCode error_code_ = common::OK;
  std::vector<ObjectPtr> objects_;
  HDMapInput* hdmap_input_ = NULL;
  std::unique_ptr<BaseRadarDetector> radar_detector_;

  std::unique_ptr<OpenglVisualizer> visualizer_ = nullptr;

  FRIEND_TEST(RadarProcessTest, test_Init);
  FRIEND_TEST(RadarProcessTest, test_Process);
  FRIEND_TEST(RadarProcessTest, test_GeneratePbMsg);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_OBSTACLE_ONBOARD_RADAR_PROCESS_H_
