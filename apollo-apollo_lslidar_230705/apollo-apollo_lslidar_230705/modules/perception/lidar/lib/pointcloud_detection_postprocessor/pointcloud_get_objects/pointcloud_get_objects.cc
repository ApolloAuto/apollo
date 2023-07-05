/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_get_objects/pointcloud_get_objects.h"


#include "modules/perception/base/object_types.h"
#include "modules/perception/base/point.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lidar {

PointCloudGetObjects::PointCloudGetObjects(const PluginConfig& plugin_config) {
  Init(plugin_config);
}

bool PointCloudGetObjects::Init(const PluginConfig& plugin_config) {
  ACHECK(plugin_config.has_pointcloud_get_objects_config());
  return true;
}

bool PointCloudGetObjects::Process(const std::vector<float>& detections,
                                   const std::vector<int>& labels,
                                   DataFrame* data_frame) {
  if (nullptr == data_frame) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  auto lidar_frame = data_frame->lidar_frame;
  GetObjects(lidar_frame->lidar2world_pose, detections, labels,
             &lidar_frame->segmented_objects);
  return true;
}

void PointCloudGetObjects::GetObjects(
    const Eigen::Affine3d& pose, const std::vector<float>& detections,
    const std::vector<int>& labels,
    std::vector<std::shared_ptr<base::Object>>* objects) {
  int num_objects = detections.size() / FLAGS_num_output_box_feature;

  objects->clear();
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    float x = detections.at(i * FLAGS_num_output_box_feature + 0);
    float y = detections.at(i * FLAGS_num_output_box_feature + 1);
    float z = detections.at(i * FLAGS_num_output_box_feature + 2);
    float dx = detections.at(i * FLAGS_num_output_box_feature + 4);
    float dy = detections.at(i * FLAGS_num_output_box_feature + 3);
    float dz = detections.at(i * FLAGS_num_output_box_feature + 5);
    float yaw = detections.at(i * FLAGS_num_output_box_feature + 6);
    yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    float roll = 0, pitch = 0;
    Eigen::Quaternionf quater =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();
    for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
      for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
        for (float vz : std::vector<float>{0, dz}) {
          Eigen::Vector3f v3f(vx, vy, vz);
          v3f = affine3f * v3f;
          base::PointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->lidar_supplement.cloud.push_back(point);

          Eigen::Vector3d trans_point(point.x, point.y, point.z);
          trans_point = pose * trans_point;
          base::PointD world_point;
          world_point.x = trans_point(0);
          world_point.y = trans_point(1);
          world_point.z = trans_point(2);
          object->lidar_supplement.cloud_world.push_back(world_point);
        }
      }
    }

    // classification
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    object->sub_type = GetObjectsubType(labels.at(i));
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] =
        1.0f;
    // copy to type
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),
                              object->lidar_supplement.raw_probs.back().end());
  }
}

base::ObjectSubType PointCloudGetObjects::GetObjectsubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::CAR;
    case 1:
      return base::ObjectSubType::PEDESTRIAN;
    case 2:  // construction vehicle
      return base::ObjectSubType::CYCLIST;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
