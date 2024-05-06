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
#include "modules/perception/multi_sensor_fusion/base/sensor_object.h"

#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/base/object_pool_types.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

// SensorObject implementations
SensorObject::SensorObject(
    const std::shared_ptr<const base::Object>& object_ptr)
    : object_(object_ptr), frame_header_(nullptr) {}

SensorObject::SensorObject(
    const std::shared_ptr<const base::Object>& object_ptr,
    const std::shared_ptr<const SensorFrameHeader>& frame_header)
    : object_(object_ptr), frame_header_(frame_header) {}

SensorObject::SensorObject(
    const std::shared_ptr<const base::Object>& object_ptr,
    const std::shared_ptr<SensorFrame>& frame_ptr)
    : object_(object_ptr),
      frame_header_((frame_ptr == nullptr) ? nullptr : frame_ptr->GetHeader()) {
}

double SensorObject::GetTimestamp() const {
  if (frame_header_ == nullptr) {
    return 0.0;
  }

  return frame_header_->timestamp;
}

bool SensorObject::GetRelatedFramePose(Eigen::Affine3d* pose) const {
  if (pose == nullptr) {
    AERROR << "pose is not available";
    return false;
  }
  if (frame_header_ == nullptr) {
    return false;
  }

  *pose = frame_header_->sensor2world_pose;
  return true;
}

std::string SensorObject::GetSensorId() const {
  if (frame_header_ == nullptr) {
    return std::string("");
  }

  return frame_header_->sensor_info.name;
}

base::SensorType SensorObject::GetSensorType() const {
  if (frame_header_ == nullptr) {
    return base::SensorType::UNKNOWN_SENSOR_TYPE;
  }

  return frame_header_->sensor_info.type;
}

// FusedObject implementations
FusedObject::FusedObject() {
  base::ObjectPool& object_pool = base::ObjectPool::Instance();
  object_ = object_pool.Get();
}

bool IsLidar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return algorithm::SensorManager::Instance()->IsLidar(type);
}

bool IsRadar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return algorithm::SensorManager::Instance()->IsRadar(type);
}

bool IsCamera(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  return algorithm::SensorManager::Instance()->IsCamera(type);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
