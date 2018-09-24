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
#include "modules/perception/fusion/base/sensor_frame.h"

#include <assert.h>

#include "modules/perception/fusion/base/sensor.h"

namespace apollo {
namespace perception {
namespace fusion {

void SensorFrame::Initialize(const base::FrameConstPtr& frame_ptr,
                             const SensorPtr& sensor_ptr) {
  timestamp_ = frame_ptr->timestamp;
  sensor2world_pose_ = frame_ptr->sensor2world_pose;
  lidar_frame_supplement_ = frame_ptr->lidar_frame_supplement;
  radar_frame_supplement_ = frame_ptr->radar_frame_supplement;
  camera_frame_supplement_ = frame_ptr->camera_frame_supplement;
  sensor_ptr_ = sensor_ptr;

  const auto& base_objects = frame_ptr->objects;
  foreground_objects_.reserve(base_objects.size());

  SensorFramePtr sensor_frame_ptr = this->GetPtr();
  for (size_t i = 0; i < base_objects.size(); ++i) {
    SensorObjectPtr obj(new SensorObject(base_objects[i], sensor_frame_ptr));
    if (base_objects[i]->lidar_supplement.is_background) {
      background_objects_.emplace_back(obj);
    } else {
      foreground_objects_.emplace_back(obj);
    }
  }
}

std::string SensorFrame::GetSensorId() const {
  if (!this->CheckSensorExist()) {
    return std::string("");
  }

  SensorConstPtr sp = sensor_ptr_.lock();
  return sp->GetSensorId();
}

base::SensorType SensorFrame::GetSensorType() const {
  if (!this->CheckSensorExist()) {
    return base::SensorType::UNKNOWN_SENSOR_TYPE;
  }

  SensorConstPtr sp = sensor_ptr_.lock();
  return sp->GetSensorType();
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
