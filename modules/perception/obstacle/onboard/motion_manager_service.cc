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

#include "modules/perception/obstacle/onboard/motion_manager_service.h"
#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"

namespace apollo {
namespace perception {

using apollo::common::adapter::AdapterManager;

bool MotionManagerService::InitInternal() {
  CHECK(AdapterManager::GetLocalization()) << "Localiztion is not initialized.";
  AdapterManager::AddLocalizationCallback(&MotionManagerService::OnLocalization,
                                          this);
  AINFO << "start to init MotionManagerService.";
  _vehicle_planemotion = new PlaneMotion(_motion_buffer_size, false,
                                         1.0f / _motion_sensor_frequency);

  AINFO << "init MotionManagerService success.";
  return true;
}

void MotionManagerService::OnLocalization(
    const apollo::localization::LocalizationEstimate& localization) {
  const auto& velocity = localization.pose().linear_velocity();
  // Get VehicleStatus
  VehicleStatus vehicle_status;
  double velx = velocity.x();
  double vely = velocity.y();
  double velz = velocity.z();
  vehicle_status.velocity = sqrt(velx * velx + vely * vely + velz * velz);

  double timestamp_diff = 0;
  if (!_start_flag) {
    _start_flag = true;
    vehicle_status.yaw_rate = 0;
    timestamp_diff = 0;
  } else {
    vehicle_status.yaw_rate = localization.pose().angular_velocity_vrf().z();
    timestamp_diff = localization.measurement_time() - pre_timestamp;
  }

  pre_timestamp = localization.measurement_time();

  // add motion to buffer
  _vehicle_planemotion->add_new_motion(&vehicle_status, timestamp_diff, false);
}

REGISTER_SUBNODE(MotionManagerService);

}  // namespace perception
}  // namespace apollo
