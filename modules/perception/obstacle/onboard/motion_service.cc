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
#include "modules/common/time/time_util.h"

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

  AdapterManager::AddImageFrontCallback(&MotionService::ImageCallback, this);

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
void MotionService::ImageCallback(const sensor_msgs::Image &message) {
  double curr_timestamp = message.header.stamp.toSec();
  ADEBUG << "motion received image : " << GLOG_TIMESTAMP(curr_timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());

  if (FLAGS_skip_camera_frame && camera_timestamp_ > 0.0) {
    if ((curr_timestamp - camera_timestamp_) < (1.0 / FLAGS_camera_hz) &&
        curr_timestamp > camera_timestamp_) {
      ADEBUG << "MotionService Skip frame";
      return;
    }
  }

  MutexLock lock(&image_mutex_);
  camera_timestamp_ = curr_timestamp;
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
  pre_timestamp_ = localization.measurement_time();

  // add motion to buffer
  // double camera_timestamp = camera_shared_data_->GetLatestTimestamp();
  double camera_timestamp = 0;
  {
     MutexLock lock(&image_mutex_);
     camera_timestamp = camera_timestamp_;
  }
  AINFO << "motion timestamp: " << std::to_string(camera_timestamp);

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
      ADEBUG << "Motion_status: pop";
      vehicle_planemotion_->add_new_motion(
          &vehicle_status, pre_camera_timestamp_, camera_timestamp,
          PlaneMotion::RESET);
    }
  }

  {
    MutexLock lock(&image_mutex_);
    pre_camera_timestamp_ = camera_timestamp;
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

double MotionService::GetLatestTimestamp() {
  MutexLock lock(&image_mutex_);
  double rst = pre_camera_timestamp_;
  return rst;
}

bool MotionService::GetMotionInformation(double timestamp, VehicleStatus *vs) {
  return vehicle_planemotion_->find_motion_with_timestamp(timestamp, vs);
}
}  // namespace perception
}  // namespace apollo
