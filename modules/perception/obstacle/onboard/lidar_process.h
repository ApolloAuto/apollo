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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PROCESS_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PROCESS_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "gtest/gtest_prod.h"
#include "sensor_msgs/PointCloud2.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/sequence_type_fuser/base_type_fuser.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/interface/base_object_builder.h"
#include "modules/perception/obstacle/lidar/interface/base_roi_filter.h"
#include "modules/perception/obstacle/lidar/interface/base_segmentation.h"
#include "modules/perception/obstacle/lidar/interface/base_tracker.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"
#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/opengl_visualizer.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"

namespace apollo {
namespace perception {

class LidarProcess {
 public:
  LidarProcess() = default;
  ~LidarProcess() = default;

  bool Init();
  bool IsInit() { return inited_; }
  bool Process(const sensor_msgs::PointCloud2& message);

  bool Process(const double timestamp, pcl_util::PointCloudPtr cloud,
               std::shared_ptr<Eigen::Matrix4d> velodyne_trans);

  void GeneratePbMsg(PerceptionObstacles* obstacles);

  std::vector<std::shared_ptr<Object>> GetObjects() { return objects_; }

  pcl_util::PointIndicesPtr GetROIIndices() { return roi_indices_; }

 private:
  void RegistAllAlgorithm();
  bool InitFrameDependence();
  bool InitAlgorithmPlugin();

  void TransPointCloudToPCL(const sensor_msgs::PointCloud2& in_msg,
                            pcl_util::PointCloudPtr* out_cloud);
  bool GetVelodyneTrans(const double query_time, Eigen::Matrix4d* trans);

  bool inited_ = false;
  double timestamp_ = 0.0;
  common::ErrorCode error_code_ = common::OK;
  std::vector<std::shared_ptr<Object>> objects_;
  HDMapInput* hdmap_input_ = nullptr;
  std::unique_ptr<BaseROIFilter> roi_filter_;
  std::unique_ptr<BaseSegmentation> segmentor_;
  std::unique_ptr<BaseObjectBuilder> object_builder_;
  std::unique_ptr<BaseTracker> tracker_;
  std::unique_ptr<BaseTypeFuser> type_fuser_;
  pcl_util::PointIndicesPtr roi_indices_;

  std::unique_ptr<OpenglVisualizer> visualizer_;

  FRIEND_TEST(LidarProcessTest, test_Init);
  FRIEND_TEST(LidarProcessTest, test_Process);
  FRIEND_TEST(LidarProcessTest, test_GeneratePbMsg);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_LIDAR_PROCESS_H_
