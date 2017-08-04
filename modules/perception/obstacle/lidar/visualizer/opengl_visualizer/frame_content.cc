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

#include "frame_content.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

FrameContent::FrameContent() :
        _pose_v2w(Eigen::Matrix4d::Identity()),
        _cloud(new pcl_util::PointCloud),
        _roi_cloud(new pcl_util::PointCloud),
        _global_offset_initialized(false) {
}

FrameContent::~FrameContent() {
}

void FrameContent::set_lidar_pose(const Eigen::Matrix4d& pose) {
    if (!_global_offset_initialized) {
        _global_offset[0] = -pose(0, 3);
        _global_offset[1] = -pose(1, 3);
        _global_offset[2] = -pose(2, 3);
        _global_offset_initialized = true;
        AINFO << "initial pose " << pose ;
        AINFO << "offset = " << _global_offset[0] << "  "
            << _global_offset[1] << "  " << _global_offset[2] << "\n";
    }
    _pose_v2w = pose;
    _pose_v2w(0, 3) += _global_offset[0];
    _pose_v2w(1, 3) += _global_offset[1];
    _pose_v2w(2, 3) += _global_offset[2];
}

Eigen::Matrix4d FrameContent::get_pose_v2w() {
    return _pose_v2w;
}

void FrameContent::set_lidar_cloud(pcl_util::PointCloudPtr cloud) {
    pcl::transformPointCloud(*cloud, *(_cloud), _pose_v2w);
}


pcl_util::PointCloudPtr FrameContent::get_cloud() {
    return _cloud;
}

bool FrameContent::has_cloud() {
    if ((_cloud == nullptr || _cloud->size() == 0)) {
        return false;
    }
    return true;
}

void FrameContent::offset_pointcloud(pcl_util::PointCloud& cloud,
    const Eigen::Vector3d& offset) {
    for (size_t i = 0; i < cloud.size(); ++i) {
        cloud.points[i].x += offset[0];
        cloud.points[i].y += offset[1];
        cloud.points[i].z += offset[2];
    }
}

void FrameContent::offset_pointcloud(pcl_util::PointDCloud& cloud,
    const Eigen::Vector3d& offset) {
    for (size_t i = 0; i < cloud.size(); ++i) {
        cloud.points[i].x += offset[0];
        cloud.points[i].y += offset[1];
        cloud.points[i].z += offset[2];
    }
}

void FrameContent::offset_object(ObjectPtr object, const Eigen::Vector3d& offset) {
    offset_pointcloud(*(object->cloud), offset);
    offset_pointcloud(object->polygon, offset);

    object->center[0] += offset[0];
    object->center[1] += offset[1];
    object->center[2] += offset[2];
}

void FrameContent::set_tracked_objects(const std::vector<ObjectPtr>& objects) {
    _tracked_objects_lidar.resize(objects.size());
    for (size_t i = 0; i < objects.size(); ++i) {
        _tracked_objects_lidar[i].reset(new Object);
        _tracked_objects_lidar[i]->clone(*objects[i]);
        offset_object(_tracked_objects_lidar[i], _global_offset);
    }
}

std::vector<ObjectPtr> FrameContent::get_tracked_objects() {
    return _tracked_objects_lidar;
}

} // namespace perception
} // namespace apollo
