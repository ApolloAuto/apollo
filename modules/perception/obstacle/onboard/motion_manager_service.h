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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_MOTION_MANAGER_SERVICE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_MOTION_MANAGER_SERVICE_H_

#include <Eigen/Core>
#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/obstacle/camera/motionmanager/vehicleplanemotion.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"

namespace apollo {
namespace perception {

struct VehicleInformation {
  double timestamp;
  double velocity;
  double yaw_rate;
  double time_diff;
};

class MotionManagerService : public Subnode {
 public:
  MotionManagerService() = default;
  virtual ~MotionManagerService() {
    delete _vehicle_planemotion;
  }

  apollo::common::Status ProcEvents() override {
    return apollo::common::Status::OK();
  }

 protected:
  bool InitInternal() override;

 private:
  void OnLocalization(
      const apollo::localization::LocalizationEstimate &localization);
  PlaneMotion *_vehicle_planemotion = nullptr;
  double pre_azimuth = 0;  // a invalid value
  double pre_timestamp = 0;
  bool _start_flag = false;
  const int _motion_buffer_size = 6000;
  const int _motion_sensor_frequency = 100;
  Mutex _mutex;

  DISALLOW_COPY_AND_ASSIGN(MotionManagerService);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_MOTION_MANAGER_SERVICE_H
