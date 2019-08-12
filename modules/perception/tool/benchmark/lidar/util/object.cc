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

#include "modules/perception/tool/benchmark/lidar/util/object.h"
#include <sstream>
#include "modules/perception/tool/benchmark/lidar/util/macros.h"

namespace apollo {
namespace perception {
namespace benchmark {

Object::Object() {
  direction = Eigen::Vector3d(1, 0, 0);
  center = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  cloud.reset(new PointCloud);
  indices.reset(new PointIndices);
  type_probs.resize(MAX_OBJECT_TYPE, 0);
  internal_type_probs.resize(INT_MAX_OBJECT_TYPE, 0);
  confidence = 1.f;
}

Object::Object(const Object& rhs) {
  id = rhs.id;
  cloud = rhs.cloud;
  indices = rhs.indices;
  confidence = rhs.confidence;
  direction = rhs.direction;
  yaw = rhs.yaw;
  roll = rhs.roll;
  pitch = rhs.pitch;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  truncated = rhs.truncated;
  occluded = rhs.occluded;
  type = rhs.type;
  type_probs = rhs.type_probs;
  internal_type = rhs.internal_type;
  internal_type_probs = rhs.internal_type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
  is_in_roi = rhs.is_in_roi;
  is_in_main_lanes = rhs.is_in_main_lanes;
  sensor_type = rhs.sensor_type;
  reserve = rhs.reserve;
  lidar_supplement = rhs.lidar_supplement;
  radar_supplement = rhs.radar_supplement;
  camera_supplement = rhs.camera_supplement;
}

Object& Object::operator=(const Object& rhs) {
  id = rhs.id;
  cloud = rhs.cloud;
  indices = rhs.indices;
  confidence = rhs.confidence;
  direction = rhs.direction;
  yaw = rhs.yaw;
  roll = rhs.roll;
  pitch = rhs.pitch;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  truncated = rhs.truncated;
  occluded = rhs.occluded;
  type = rhs.type;
  type_probs = rhs.type_probs;
  internal_type = rhs.internal_type;
  internal_type_probs = rhs.internal_type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
  is_in_roi = rhs.is_in_roi;
  is_in_main_lanes = rhs.is_in_main_lanes;
  sensor_type = rhs.sensor_type;
  reserve = rhs.reserve;
  lidar_supplement = rhs.lidar_supplement;
  radar_supplement = rhs.radar_supplement;
  camera_supplement = rhs.camera_supplement;
  return (*this);
}

void Object::clone(const Object& rhs) {
  id = rhs.id;
  pcl::copyPointCloud<Point, Point>(*(rhs.cloud), *cloud);
  indices->indices = rhs.indices->indices;
  confidence = rhs.confidence;
  direction = rhs.direction;
  yaw = rhs.yaw;
  roll = rhs.roll;
  pitch = rhs.pitch;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  truncated = rhs.truncated;
  occluded = rhs.occluded;
  type = rhs.type;
  type_probs = rhs.type_probs;
  internal_type = rhs.internal_type;
  internal_type_probs = rhs.internal_type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
  is_in_roi = rhs.is_in_roi;
  is_in_main_lanes = rhs.is_in_main_lanes;
  sensor_type = rhs.sensor_type;
  reserve = rhs.reserve;
  lidar_supplement = nullptr;
  if (rhs.lidar_supplement != nullptr) {
    lidar_supplement.reset(new LidarSupplement());
    lidar_supplement->clone(*(rhs.lidar_supplement));
  }

  radar_supplement = nullptr;
  if (rhs.radar_supplement != nullptr) {
    radar_supplement.reset(new RadarSupplement());
    radar_supplement->clone(*(rhs.radar_supplement));
  }

  camera_supplement = nullptr;
  if (rhs.camera_supplement != nullptr) {
    camera_supplement.reset(new CameraSupplement());
    camera_supplement->clone(*(rhs.camera_supplement));
  }
}

std::string Object::to_string() const {
  std::ostringstream oss;
  oss << "Object[id: " << id << ", track_id: " << track_id
      << ", cloud_size: " << cloud->size()
      << ", indices_size: " << indices->indices.size()
      << ", direction: " << direction.transpose() << ", yaw: " << yaw
      << ", center: " << center.transpose()
      << " velocity: " << velocity.transpose() << ", width: " << width
      << ", length: " << length << ", height: " << height
      << ", confidence: " << confidence << ", type: " << type
      << ", is_background: " << is_background << ", is_in_roi: " << is_in_roi
      << ", is_in_main_lanes: " << is_in_main_lanes
      << ", tracking_time: " << GLOG_TIMESTAMP(tracking_time)
      << ", latest_tracked_time: " << GLOG_TIMESTAMP(latest_tracked_time)
      << "]";

  return oss.str();
}

std::string get_object_name(ObjectType obj_type) {
  std::string obj_name;
  switch (obj_type) {
    case UNKNOWN:
      obj_name = "unknown";
      break;
    case UNKNOWN_MOVABLE:
      obj_name = "unknown_movable";
      break;
    case UNKNOWN_UNMOVABLE:
      obj_name = "unknown_unmovable";
      break;
    case PEDESTRIAN:
      obj_name = "pedestrian";
      break;
    case BICYCLE:
      obj_name = "bicycle";
      break;
    case VEHICLE:
      obj_name = "vehicle";
      break;
    default:
      obj_name = "error";
      break;
  }
  return obj_name;
}

std::string get_sensor_name(SensorType sensor_type) {
  std::string sensor_name;
  switch (sensor_type) {
    case VELODYNE_64:
      sensor_name = "velodyne_64";
      break;
    case VELODYNE_16:
      sensor_name = "velodyne_16";
      break;
    case RADAR:
      sensor_name = "radar";
      break;
    case CAMERA:
      sensor_name = "camera";
      break;
    case UNKNOWN_SENSOR_TYPE:
      sensor_name = "unknown_sensor_type";
      break;
  }
  return sensor_name;
}

std::string SensorObjects::to_string() const {
  std::ostringstream oss;
  oss << "SensorObjects[sensor_type: " << get_sensor_name(type)
      << ", name: " << name << ", timestamp:" << GLOG_TIMESTAMP(timestamp)
      << ", sensor2world_pose:\n";
  oss << sensor2world_pose << "\n, objects: " << objects.size() << " < ";
  for (auto obj : objects) {
    oss << "\n" << obj->to_string();
  }
  oss << " >]";
  return oss.str();
}

ObjectType translate_string_to_type(const std::string& str) {
  if (str == "bigMot" || str == "smallMot" || str == "vehicle" ||
      str == "midMot" || str == "5") {
    return VEHICLE;
  } else if (str == "pedestrian" || str == "3") {
    return PEDESTRIAN;
  } else if (str == "nonMot" || str == "cyclist" || str == "motorcyclist" ||
             str == "bicyclist" || str == "4") {
    return BICYCLE;
  } else {
    return UNKNOWN;
  }
}

unsigned int translate_type_to_index(const ObjectType& type) {
  if (type <= UNKNOWN) {
    return 0;
  } else if (type < MAX_OBJECT_TYPE) {
    return static_cast<unsigned int>(type) - 2;
  } else {
    return 0;
  }
}

std::string translate_type_index_to_string(unsigned int index) {
  switch (index) {
    case 0:
      return "others";
    case 1:
      return "pedestrian";
    case 2:
      return "cyclist";
    case 3:
      return "vehicle";
    default:
      return "others";
  }
}

SensorType translate_string_to_sensor_type(const std::string& str) {
  if (str == "velodyne_64") {
    return VELODYNE_64;
  } else if (str == "velodyne_16") {
    return VELODYNE_16;
  } else if (str == "radar") {
    return RADAR;
  } else if (str == "camera") {
    return CAMERA;
  } else {
    return UNKNOWN_SENSOR_TYPE;
  }
}

std::string translate_type_to_string(ObjectType type) {
  switch (type) {
    case UNKNOWN:
    case UNKNOWN_MOVABLE:
    case UNKNOWN_UNMOVABLE:
      return "others";
    case PEDESTRIAN:
      return "pedestrian";
    case BICYCLE:
      return "cyclist";
    case VEHICLE:
      return "vehicle";
    case MAX_OBJECT_TYPE:
    default:
      return "others";
  }
}

std::string translate_sensor_type_to_string(const SensorType& type) {
  switch (type) {
    case VELODYNE_64:
      return "velodyne_64";
    case VELODYNE_16:
      return "velodyne_16";
    case RADAR:
      return "radar";
    case CAMERA:
      return "camera";
    default:
      return "unknown_sensor_type";
  }
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
