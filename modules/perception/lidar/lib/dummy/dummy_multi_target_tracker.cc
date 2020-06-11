/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/dummy/dummy_multi_target_tracker.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/point_cloud_processing/common.h"

namespace apollo {
namespace perception {
namespace lidar {

bool DummyMultiTargetTracker::Init(
    const MultiTargetTrackerInitOptions& options) {
  return true;
}

bool DummyMultiTargetTracker::Track(const MultiTargetTrackerOptions& options,
                                    LidarFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  // transform objects
  frame->tracked_objects.clear();
  base::ObjectPool::Instance().BatchGet(frame->segmented_objects.size(),
                                        &frame->tracked_objects);
  for (size_t i = 0; i < frame->segmented_objects.size(); i++) {
    *(frame->tracked_objects[i]) = *(frame->segmented_objects[i]);
    frame->tracked_objects[i]->track_id = id_++;
    auto& obj = frame->tracked_objects[i];
    Eigen::Vector3d direction = obj->direction.cast<double>();
    direction = frame->lidar2world_pose.linear() * direction;
    obj->direction = direction.cast<float>();
    obj->center = frame->lidar2world_pose * obj->center;
    obj->anchor_point = obj->center;

    obj->lidar_supplement.cloud_world.resize(
        obj->lidar_supplement.cloud.size());

    for (size_t j = 0; j < obj->lidar_supplement.cloud.size(); j++) {
      obj->lidar_supplement.cloud_world[j].x = obj->lidar_supplement.cloud[j].x;
      obj->lidar_supplement.cloud_world[j].y = obj->lidar_supplement.cloud[j].y;
      obj->lidar_supplement.cloud_world[j].z = obj->lidar_supplement.cloud[j].z;
    }
    const Eigen::Affine3d pose = frame->lidar2world_pose;
    common::TransformPointCloud(pose, &obj->lidar_supplement.cloud_world);
    common::TransformPointCloud(
        pose, (base::AttributePointCloud<base::PointD>*)&obj->polygon);
    obj->theta =
        static_cast<float>(atan2(obj->direction[1], obj->direction[0]));
  }
  return true;
}

PERCEPTION_REGISTER_MULTITARGET_TRACKER(DummyMultiTargetTracker);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
