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
#include<limits>
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;

bool MotionService::InitInternal() {
  CHECK(AdapterManager::GetLocalization()) << "Localiztion is not initialized.";
  AdapterManager::AddLocalizationCallback(&MotionService::OnLocalization, this);
  AINFO << "start to init MotionService.";
  vehicle_planemotion_ = new PlaneMotion(motion_buffer_size_, false,
                                         1.0f / motion_sensor_frequency_);

  CHECK(shared_data_manager_ != NULL);
  camera_shared_data_ = dynamic_cast<CameraSharedData*>(
    shared_data_manager_->GetSharedData("CameraSharedData"));
    if (camera_shared_data_ == nullptr) {
      AERROR << "Failed to get CameraSharedData.";
      return false;
    }
  AINFO << "init MotionService success.";
  return true;
}

void MotionService::OnLocalization(
    const localization::LocalizationEstimate& localization) {
  const auto& velocity = localization.pose().linear_velocity();
  // Get VehicleStatus
  VehicleStatus vehicle_status;
  double velx = velocity.x();
  double vely = velocity.y();
  double velz = velocity.z();
  vehicle_status.velocity = sqrt(velx * velx + vely * vely + velz * velz);

  double timestamp_diff = 0;
  if (!start_flag_) {
    start_flag_ = true;
    vehicle_status.yaw_rate = 0;
    timestamp_diff = 0;
  } else {
    vehicle_status.yaw_rate = localization.pose().angular_velocity_vrf().z();
    timestamp_diff = localization.measurement_time() - pre_timestamp;
  }

  VehicleInformation vehicle_information;
  vehicle_information.timestamp = localization.measurement_time();
  vehicle_information.velocity = vehicle_status.velocity;
  vehicle_information.yaw_rate = vehicle_status.yaw_rate;
  vehicle_information.time_diff = timestamp_diff;

  {
    MutexLock lock(&mutex_);
    vehicle_information_buffer_.push_back(vehicle_information);
  }
  pre_timestamp = localization.measurement_time();

  // add motion to buffer
  double camera_timestamp =
    camera_shared_data_->GetLatestTimestamp();
  if (std::abs(pre_timestamp-camera_timestamp) <
        std::numeric_limits<double>::epsilon()) {
      // exactly same timestamp
      vehicle_planemotion_->add_new_motion(&vehicle_status,
       timestamp_diff, PlaneMotion::ACCUM_PUSH_MOTION);
  } else if (pre_timestamp < camera_timestamp) {
      vehicle_planemotion_->add_new_motion(&vehicle_status,
       timestamp_diff, PlaneMotion::ACCUM_MOTION);
  } else {
      vehicle_planemotion_->add_new_motion(&vehicle_status,
       timestamp_diff, PlaneMotion::PUSH_ACCUM_MOTION);
  }
}

void MotionService::GetVehicleInformation(
    float timestamp, VehicleInformation* vehicle_information) {
  MutexLock lock(&mutex_);
  if (vehicle_information_buffer_.empty()) {
    return;
  }
  std::list<VehicleInformation>::iterator iter =
      vehicle_information_buffer_.begin();
  std::list<VehicleInformation>::iterator vehicle_information_iter1 = iter;
  ++iter;
  if (vehicle_information_iter1->timestamp >= timestamp) {
    *vehicle_information = *vehicle_information_iter1;
    return;
  }
  std::list<VehicleInformation>::iterator vehicle_information_iter2;
  while (iter != vehicle_information_buffer_.end()) {
    vehicle_information_iter2 = iter;
    ++iter;
    if (vehicle_information_iter1->timestamp <= timestamp &&
        vehicle_information_iter2->timestamp >= timestamp) {
      *vehicle_information =
          fabs(vehicle_information_iter1->timestamp - timestamp) <
                  fabs(vehicle_information_iter2->timestamp - timestamp)
              ? *vehicle_information_iter1
              : *vehicle_information_iter2;
      break;
    }
    vehicle_information_buffer_.erase(vehicle_information_iter1);
    vehicle_information_iter1 = vehicle_information_iter2;
  }
}

void MotionService::GetMotionBuffer(MotionBufferPtr motion_buffer) {
  motion_buffer = vehicle_planemotion_->get_buffer();
  return;
}

REGISTER_SUBNODE(MotionService);

}  // namespace perception
}  // namespace apollo
