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

#ifndef MODUELS_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_OBJECT_H_  // NOLINT
#define MODUELS_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_OBJECT_H_  // NOLINT

#include <memory>
#include <string>
#include <vector>
#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

struct PbfSensorObject {
  PbfSensorObject();
  explicit PbfSensorObject(std::shared_ptr<Object> obj3d, SensorType type,
                           double time);
  ~PbfSensorObject();
  PbfSensorObject(const PbfSensorObject &rhs);
  PbfSensorObject &operator=(const PbfSensorObject &rhs);
  void clone(const PbfSensorObject &rhs);

  SensorType sensor_type;
  std::string sensor_id;
  double timestamp;
  std::shared_ptr<Object> object;
  double invisible_period;
};

struct PbfSensorFrame {
  PbfSensorFrame() { sensor2world_pose = Eigen::Matrix4d::Identity(); }
  SensorType sensor_type = SensorType::UNKNOWN_SENSOR_TYPE;
  std::string sensor_id = "unknown_sensor_type";
  /**@brief capturing timestamp*/
  double timestamp = 0.0;
  int seq_num = 0;
  Eigen::Matrix4d sensor2world_pose;
  std::vector<std::shared_ptr<PbfSensorObject>> objects;
};

typedef std::shared_ptr<PbfSensorFrame> PbfSensorFramePtr;

}  // namespace perception
}  // namespace apollo

// clang-format off
#endif  // MODUELS_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_SENSOR_OBJECT_H_ // NOLINT
// clang-format on
