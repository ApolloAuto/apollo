/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/perception/tool/benchmark/lidar/util/object_supplement.h"
#include "modules/perception/tool/benchmark/lidar/util/types.h"

namespace apollo {
namespace perception {
namespace benchmark {

enum ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};

ObjectType translate_string_to_type(const std::string& str);

unsigned int translate_type_to_index(const ObjectType& type);

std::string translate_type_index_to_string(unsigned int index);

std::string translate_type_to_string(ObjectType type);

enum InternalObjectType {
  INT_BACKGROUND = 0,
  INT_SMALLMOT = 1,
  INT_PEDESTRIAN = 2,
  INT_NONMOT = 3,
  INT_BIGMOT = 4,
  INT_UNKNOWN = 5,
  INT_MAX_OBJECT_TYPE = 6,
};

enum SensorType {
  VELODYNE_64 = 0,
  VELODYNE_16 = 1,
  RADAR = 2,
  CAMERA = 3,
  UNKNOWN_SENSOR_TYPE = 4,
};

SensorType translate_string_to_sensor_type(const std::string& str);

std::string translate_sensor_type_to_string(const SensorType& type);

struct alignas(16) Object {
  Object();
  // shallow copy for copy constructor and assignment
  Object(const Object& rhs);
  Object& operator=(const Object& rhs);
  // deep copy
  void clone(const Object& rhs);
  std::string to_string() const;

  // object id per frame
  int id = 0;
  // point cloud of the object
  PointCloudPtr cloud;
  // point cloud indices of the object
  PointIndicesPtr indices;
  // polygon of the object
  PointCloud polygon;
  // confidence of the object
  float confidence = 1.f;

  // oriented boundingbox information
  // main direction
  Eigen::Vector3d direction;
  // the yaw angle, direction with x-axis (1, 0, 0)
  double yaw = 0.0;
  // the roll angle, direction with
  double roll = 0.0;
  // the pitch angle, direction with
  double pitch = 0.0;
  // ground center of the object (cx, cy, z_min)
  Eigen::Vector3d center;
  // size of the oriented bbox, length is the size in the main direction
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  // truncated
  double truncated = 0.0;
  double occluded = 0.0;

  // Object classification type.
  ObjectType type = UNKNOWN;
  // Probability of each type, used for track type.
  std::vector<float> type_probs;

  // Internal object classification type.
  InternalObjectType internal_type;
  // Internal probability of each type, used for track type.
  std::vector<float> internal_type_probs;

  // fg/bg flag
  bool is_background = false;

  // tracking information
  int track_id = 0;

  Eigen::Vector3d velocity;
  // age of the tracked object
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;

  // roi flag
  bool is_in_roi = false;

  // lane flag
  bool is_in_main_lanes = false;

  // visible related
  float visible_ratio = 1.f;
  bool visible = true;

  // sensor type
  SensorType sensor_type = UNKNOWN_SENSOR_TYPE;

  // reserve
  std::string reserve;

  // sensor particular suplplements, default nullptr
  LidarSupplementPtr lidar_supplement = nullptr;
  RadarSupplementPtr radar_supplement = nullptr;
  CameraSupplementPtr camera_supplement = nullptr;

  // jaccard index with ground truth when benchmark evaluation
  double ji = 0.0;
};

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<const Object> ObjectConstPtr;

using SeqId = uint32_t;

// Sensor single frame objects.
struct SensorObjects {
  SensorObjects() { sensor2world_pose = Eigen::Matrix4d::Zero(); }

  std::string to_string() const;

  SensorType type = UNKNOWN_SENSOR_TYPE;
  std::string name;
  double timestamp = 0.0;
  SeqId seq_num = 0;
  std::vector<ObjectPtr> objects;
  std::vector<std::vector<Eigen::Vector3d>> objects_box_vertices;
  std::vector<ObjectPtr> gt_objects;
  std::vector<std::vector<Eigen::Vector3d>> gt_objects_box_vertices;
  Eigen::Matrix4d sensor2world_pose;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
