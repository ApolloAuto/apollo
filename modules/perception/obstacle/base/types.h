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

#ifndef MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_
#define MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_

#include <string>

#include "modules/perception/lib/pcl_util/pcl_types.h"

namespace apollo {
namespace perception {

enum ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};

enum SensorType {
  VELODYNE_64 = 0,
  VELODYNE_16 = 1,
  RADAR = 2,
  CAMERA = 3,
  UNKNOWN_SENSOR_TYPE = 10,
};

typedef pcl_util::PointCloud PolygonType;
typedef pcl_util::PointDCloud PolygonDType;

using SeqId = uint32_t;

std::string GetSensorType(SensorType sensor_type);

bool is_lidar(SensorType sensor_type);
bool is_radar(SensorType sensor_type);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_
