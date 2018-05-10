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

#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

std::string GetObjectName(const ObjectType& obj_type) {
  std::string obj_name;
  switch (obj_type) {
    case ObjectType::UNKNOWN:
      obj_name = "unknown";
      break;
    case ObjectType::UNKNOWN_MOVABLE:
      obj_name = "unknown_movable";
      break;
    case ObjectType::UNKNOWN_UNMOVABLE:
      obj_name = "unknown_unmovable";
      break;
    case ObjectType::PEDESTRIAN:
      obj_name = "pedestrian";
      break;
    case ObjectType::BICYCLE:
      obj_name = "bicycle";
      break;
    case ObjectType::VEHICLE:
      obj_name = "vehicle";
      break;
    default:
      obj_name = "error";
      break;
  }
  return obj_name;
}

std::string GetSensorType(SensorType sensor_type) {
  switch (sensor_type) {
    case SensorType::VELODYNE_64:
      return "velodyne_64";
    case SensorType::VELODYNE_16:
      return "velodyne_16";
    case SensorType::RADAR:
      return "radar";
    case SensorType::CAMERA:
      return "camera";
    case SensorType::UNKNOWN_SENSOR_TYPE:
      return "unknown_sensor_type";
  }
  return "";
}

bool is_lidar(SensorType sensor_type) {
  return (sensor_type == SensorType::VELODYNE_64);
}

bool is_radar(SensorType sensor_type) {
  return (sensor_type == SensorType::RADAR);
}

bool is_camera(SensorType sensor_type) {
  return (sensor_type == SensorType::CAMERA);
}

}  // namespace perception
}  // namespace apollo
