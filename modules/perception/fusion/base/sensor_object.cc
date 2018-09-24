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
#include "modules/perception/fusion/base/sensor_object.h"

#include <assert.h>

#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

// SensorObject implementations
SensorObject::SensorObject(const base::ObjectConstPtr& object_ptr,
                           const SensorFramePtr& frame_ptr) {
  object_ = object_ptr;
  frame_ptr_ = frame_ptr;
}

double SensorObject::GetTimestamp() const {
  if (!this->CheckFrameExist()) {
    return 0.0;
  }

  SensorFrameConstPtr sp = frame_ptr_.lock();
  return sp->GetTimestamp();
}

bool SensorObject::GetRelatedFramePose(Eigen::Affine3d* pose) const {
  assert(pose != nullptr);
  if (!this->CheckFrameExist()) {
    return false;
  }

  SensorFrameConstPtr sp = frame_ptr_.lock();
  return sp->GetPose(pose);
}

std::string SensorObject::GetSensorId() const {
  if (!this->CheckFrameExist()) {
    return std::string("");
  }

  SensorFrameConstPtr sp = frame_ptr_.lock();
  return sp->GetSensorId();
}

base::SensorType SensorObject::GetSensorType() const {
  if (!this->CheckFrameExist()) {
    return base::SensorType::UNKNOWN_SENSOR_TYPE;
  }

  SensorFrameConstPtr sp = frame_ptr_.lock();
  return sp->GetSensorType();
}

// FusedObject implementations
FusedObject::FusedObject() {
  base::ObjectPool& object_pool = base::ObjectPool::Instance();
  object_ = object_pool.Get();
}

bool IsLidar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  common::SensorManager* sensor_manager =
      lib::Singleton<common::SensorManager>::get_instance();
  return sensor_manager->IsLidar(type);
}

bool IsRadar(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  common::SensorManager* sensor_manager =
      lib::Singleton<common::SensorManager>::get_instance();
  return sensor_manager->IsRadar(type);
}

bool IsCamera(const SensorObjectConstPtr& obj) {
  base::SensorType type = obj->GetSensorType();
  common::SensorManager* sensor_manager =
      lib::Singleton<common::SensorManager>::get_instance();
  return sensor_manager->IsCamera(type);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
