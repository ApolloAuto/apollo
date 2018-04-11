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
#include "modules/perception/obstacle/onboard/motion_service.h"

#include <limits>
#include <string>
#include <unordered_map>

#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;

bool MotionService::InitInternal() {
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);
  auto citer = fields.find("device_id");
  if (citer == fields.end()) {
    AERROR << "Failed to find field device_id, reserve: " << reserve_;
    return false;
  }
  device_id_ = citer->second;

  CHECK(AdapterManager::GetLocalization()) << "Localiztion is not initialized.";
  AdapterManager::AddLocalizationCallback(&MotionService::OnLocalization, this);
  AINFO << "start to init MotionService.";
  vehicle_planemotion_ = new PlaneMotion(motion_buffer_size_);

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
    vehicle_status.time_d = 0;
    vehicle_status.time_ts = 0;

  } else {
    vehicle_status.yaw_rate = localization.pose().angular_velocity_vrf().z();
    timestamp_diff = localization.measurement_time() - pre_timestamp_;
    vehicle_status.time_d = timestamp_diff;
    vehicle_status.time_ts = localization.measurement_time();
    //    timestamp_diff = localization.header().timestamp_sec() -
    //    pre_timestamp_;
  }

  VehicleInformation vehicle_information;
  vehicle_information.timestamp = localization.measurement_time();
  //  vehicle_information.timestamp = localization.header().timestamp_sec();

  vehicle_information.velocity = vehicle_status.velocity;
  vehicle_information.yaw_rate = vehicle_status.yaw_rate;
  vehicle_information.time_diff = timestamp_diff;

  {
    MutexLock lock(&mutex_);
    vehicle_information_buffer_.push_back(vehicle_information);
  }
  pre_timestamp_ = localization.measurement_time();
  //  pre_timestamp_ = localization.header().timestamp_sec();

  // add motion to buffer
  double camera_timestamp = camera_shared_data_->GetLatestTimestamp();
  if (start_flag_) {
    if (std::abs(camera_timestamp - pre_camera_timestamp_) <
        std::numeric_limits<double>::epsilon()) {
      ADEBUG << "Motion_status: accum";
      vehicle_planemotion_->add_new_motion(
          &vehicle_status, pre_camera_timestamp_, camera_timestamp,
          PlaneMotion::ACCUM_MOTION);
    } else if (camera_timestamp > pre_camera_timestamp_) {
      ADEBUG << "Motion_status: accum_push";
      vehicle_planemotion_->add_new_motion(
          &vehicle_status, pre_camera_timestamp_, camera_timestamp,
          PlaneMotion::ACCUM_PUSH_MOTION);
      PublishEvent(camera_timestamp);
    } else {
      AERROR << "camera timestamp should arrive in order";
      return;
    }
  }
  pre_camera_timestamp_ = camera_timestamp;

  //  AINFO << "pre_timestamp_:" <<std::to_string(pre_timestamp_);
  //  AINFO << "cam_timestamp_:" <<std::to_string(camera_timestamp);

  //   if (std::abs(pre_timestamp - camera_timestamp) <
  //       std::numeric_limits<double>::epsilon()) {
  //     // exactly same timestamp
  //     vehicle_planemotion_->add_new_motion(&vehicle_status, timestamp_diff,
  //                                          PlaneMotion::ACCUM_PUSH_MOTION);
  //     AINFO << "Motion_status: accum_push";
  //   } else if (pre_timestamp < camera_timestamp) {
  //     vehicle_planemotion_->add_new_motion(&vehicle_status, timestamp_diff,
  //                                          PlaneMotion::ACCUM_MOTION);
  //     AINFO << "Motion_status: acuum";
  //   } else {
  //     vehicle_planemotion_->add_new_motion(&vehicle_status, timestamp_diff,
  //                                          PlaneMotion::PUSH_ACCUM_MOTION);
  //     AINFO << "Motion_status: push_accum";
  //   }

  //  AINFO << "Motion Matrix: ";
  //  auto motion_buffer_ptr = vehicle_planemotion_->get_buffer();
  //  int motion_size = motion_buffer_ptr->size();
  //  AINFO << (*motion_buffer_ptr)[motion_size-1].motion;
  //  AINFO << "Motion Matrix end";
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
void MotionService::PublishEvent(const double timestamp) {
  // pub events
  //  AINFO << "MotionService: pub size " << pub_meta_events_.size();
  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta& event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
    //    AINFO << "MotionService: event_id " << event.event_id
    //      << " timestamp " << timestamp <<", device_id "
    //      << device_id_ ;
  }
}
MotionBufferPtr MotionService::GetMotionBuffer() {
  return vehicle_planemotion_->get_buffer();
}

}  // namespace perception
}  // namespace apollo
