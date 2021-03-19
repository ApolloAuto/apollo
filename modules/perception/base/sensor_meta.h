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
#pragma once

#include <string>

namespace apollo {
namespace perception {
namespace base {

/**
 * @brief Sensor types are set in the order of lidar, radar, camera, ultrasonic
 *        Please make sure SensorType has same id with SensorMeta,
 *        which defined in the proto of sensor_manager
 */
enum class SensorType {
  UNKNOWN_SENSOR_TYPE = -1,
  VELODYNE_64 = 0,
  VELODYNE_32 = 1,
  VELODYNE_16 = 2,
  LDLIDAR_4 = 3,
  LDLIDAR_1 = 4,
  SHORT_RANGE_RADAR = 5,
  LONG_RANGE_RADAR = 6,
  MONOCULAR_CAMERA = 7,
  STEREO_CAMERA = 8,
  ULTRASONIC = 9,
  VELODYNE_128 = 10,
  SENSOR_TYPE_NUM
};

enum class SensorOrientation {
  FRONT = 0,
  LEFT_FORWARD = 1,
  LEFT = 2,
  LEFT_BACKWARD = 3,
  REAR = 4,
  RIGHT_BACKWARD = 5,
  RIGHT = 6,
  RIGHT_FORWARD = 7,
  PANORAMIC = 8
};

struct SensorInfo {
  std::string name = "UNKNONW_SENSOR";
  SensorType type = SensorType::UNKNOWN_SENSOR_TYPE;
  SensorOrientation orientation = SensorOrientation::FRONT;
  std::string frame_id = "UNKNOWN_FRAME_ID";
  void Reset() {
    name = "UNKNONW_SENSOR";
    type = SensorType::UNKNOWN_SENSOR_TYPE;
    orientation = SensorOrientation::FRONT;
    frame_id = "UNKNOWN_FRAME_ID";
  }
};

// TODO(All): remove
// typedef std::shared_ptr<SensorInfo> SensorInfoPtr;

}  // namespace base
}  // namespace perception
}  // namespace apollo
