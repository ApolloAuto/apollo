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

namespace apollo {
namespace perception {
namespace fusion {

SensorFrame::SensorFrame() { header_.reset(new SensorFrameHeader()); }

SensorFrame::SensorFrame(const base::FrameConstPtr& base_frame_ptr) {
  Initialize(base_frame_ptr);
}

void SensorFrame::Initialize(const base::FrameConstPtr& base_frame_ptr) {
  header_.reset(new SensorFrameHeader(base_frame_ptr->sensor_info,
                                      base_frame_ptr->timestamp,
                                      base_frame_ptr->sensor2world_pose));

  lidar_frame_supplement_ = base_frame_ptr->lidar_frame_supplement;
  radar_frame_supplement_ = base_frame_ptr->radar_frame_supplement;
  camera_frame_supplement_ = base_frame_ptr->camera_frame_supplement;

  const auto& base_objects = base_frame_ptr->objects;
  foreground_objects_.reserve(base_objects.size());

  for (const auto& base_obj : base_objects) {
    SensorObjectPtr obj = std::make_shared<SensorObject>(base_obj, header_);
    if (base_obj->lidar_supplement.is_background) {
      background_objects_.emplace_back(obj);
    } else {
      foreground_objects_.emplace_back(obj);
    }
  }
}

void SensorFrame::Initialize(const base::FrameConstPtr& base_frame_ptr,
                             const SensorPtr& sensor) {
  Initialize(base_frame_ptr);
}

std::string SensorFrame::GetSensorId() const {
  if (header_ == nullptr) {
    return std::string("");
  }

  return header_->sensor_info.name;
}

base::SensorType SensorFrame::GetSensorType() const {
  if (header_ == nullptr) {
    return base::SensorType::UNKNOWN_SENSOR_TYPE;
  }

  return header_->sensor_info.type;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
