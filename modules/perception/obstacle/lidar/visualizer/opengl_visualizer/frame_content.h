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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_FRAME_CONTENT_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_FRAME_CONTENT_H_

#include <deque>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <thread>

#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

class FrameContent {
 public:
  FrameContent();
  ~FrameContent();

  void SetLidarPose(const Eigen::Matrix4d &pose);
  Eigen::Matrix4d GetPoseV2w();

  void SetLidarCloud(pcl_util::PointCloudPtr cloud);
  void SetLidarRoiCloud(pcl_util::PointCloudPtr cloud);
  pcl_util::PointCloudPtr GetCloud();
  pcl_util::PointCloudPtr GetRoiCloud();

  bool HasCloud();

  void SetTrackedObjectsLidar(const std::vector<ObjectPtr> &objects);
  void SetTrackedObjectsRadar(const std::vector<ObjectPtr> &objects);
  void SetTrackedObjectsFused(const std::vector<ObjectPtr> &objects);
  std::vector<ObjectPtr> GetTrackedObjects();

 protected:
  // coordinate transform utilities
  void OffsetPointcloud(pcl_util::PointCloud* cloud,
                        const Eigen::Vector3d &offset);
  void OffsetPointcloud(pcl_util::PointDCloud* cloud,
                        const Eigen::Vector3d &offset);
  void OffsetObject(ObjectPtr object, const Eigen::Vector3d &offset);

 private:
  // input
  // 1.lidar
  Eigen::Matrix4d pose_v2w_;
  pcl_util::PointCloudPtr cloud_;
  pcl_util::PointCloudPtr roi_cloud_;

  Eigen::Vector3d global_offset_;
  bool global_offset_initialized_;
  std::vector<ObjectPtr> tracked_objects_lidar_;  // after tracking
  std::vector<ObjectPtr> tracked_objects_radar_;  // after tracking
  std::vector<ObjectPtr> tracked_objects_fused_;  // after tracking
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_FRAME_CONTENT_H_
