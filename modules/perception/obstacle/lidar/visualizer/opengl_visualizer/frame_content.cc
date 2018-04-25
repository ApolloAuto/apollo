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

#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/frame_content.h"

#include "modules/common/log.h"

namespace apollo {
namespace perception {

FrameContent::FrameContent()
    : pose_v2w_(Eigen::Matrix4d::Identity()),
      cloud_(new pcl_util::PointCloud),
      roi_cloud_(new pcl_util::PointCloud),
      global_offset_initialized_(false) {}

FrameContent::~FrameContent() {}

void FrameContent::SetLidarPose(const Eigen::Matrix4d &pose) {
  if (!global_offset_initialized_) {
    global_offset_[0] = -pose(0, 3);
    global_offset_[1] = -pose(1, 3);
    global_offset_[2] = -pose(2, 3);
    global_offset_initialized_ = true;
    AINFO << "initial pose " << pose;
    AINFO << "offset = " << global_offset_[0] << "  " << global_offset_[1]
          << "  " << global_offset_[2] << "\n";
  }
  pose_v2w_ = pose;
  pose_v2w_(0, 3) += global_offset_[0];
  pose_v2w_(1, 3) += global_offset_[1];
  pose_v2w_(2, 3) += global_offset_[2];
}

Eigen::Matrix4d FrameContent::GetPoseV2w() { return pose_v2w_; }

void FrameContent::SetLidarCloud(pcl_util::PointCloudPtr cloud) {
  pcl::transformPointCloud(*cloud, *(cloud_), pose_v2w_);
}

void FrameContent::SetLidarRoiCloud(pcl_util::PointCloudPtr cloud) {
  pcl::transformPointCloud(*cloud, *(roi_cloud_), pose_v2w_);
}

pcl_util::PointCloudPtr FrameContent::GetCloud() { return cloud_; }

pcl_util::PointCloudPtr FrameContent::GetRoiCloud() { return roi_cloud_; }

bool FrameContent::HasCloud() {
  if ((cloud_ == nullptr || cloud_->size() == 0)) {
    return false;
  }
  return true;
}

void FrameContent::OffsetPointcloud(pcl_util::PointCloud *cloud,
                                    const Eigen::Vector3d &offset) {
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud->points[i].x += offset[0];
    cloud->points[i].y += offset[1];
    cloud->points[i].z += offset[2];
  }
}

void FrameContent::OffsetPointcloud(pcl_util::PointDCloud *cloud,
                                    const Eigen::Vector3d &offset) {
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud->points[i].x += offset[0];
    cloud->points[i].y += offset[1];
    cloud->points[i].z += offset[2];
  }
}

void FrameContent::OffsetObject(std::shared_ptr<Object> object,
                                const Eigen::Vector3d &offset) {
  OffsetPointcloud(object->cloud.get(), offset);
  OffsetPointcloud(&(object->polygon), offset);

  object->center[0] += offset[0];
  object->center[1] += offset[1];
  object->center[2] += offset[2];
}

void FrameContent::SetTrackedObjects(
    const std::vector<std::shared_ptr<Object>> &objects) {
  tracked_objects_.resize(objects.size());
  for (size_t i = 0; i < objects.size(); ++i) {
    tracked_objects_[i].reset(new Object);
    tracked_objects_[i]->clone(*objects[i]);
    OffsetObject(tracked_objects_[i], global_offset_);
  }
}

std::vector<std::shared_ptr<Object>> FrameContent::GetTrackedObjects() {
  return tracked_objects_;
}

}  // namespace perception
}  // namespace apollo
