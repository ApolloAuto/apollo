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

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_MOTION_SERVICE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_MOTION_SERVICE_H_

#include <Eigen/Core>
#include <list>
#include <string>
#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/base/mutex.h"
#include "modules/perception/obstacle/camera/motion/plane_motion.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
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

class MotionService : public Subnode {
 public:
  MotionService() = default;
  virtual ~MotionService() { delete vehicle_planemotion_; }

  common::Status ProcEvents() override { return common::Status::OK(); }

  void GetVehicleInformation(float timestamp,
                             VehicleInformation *vehicle_information);
  bool GetMotionInformation(double timestamp, VehicleStatus *vs);
  MotionBufferPtr GetMotionBuffer();
  double GetLatestTimestamp();

 protected:
  bool InitInternal() override;

 private:
  void OnLocalization(const localization::LocalizationEstimate &localization);
  void ImageCallback(const sensor_msgs::Image &message);
  void PublishEvent(double timestamp);
  PlaneMotion *vehicle_planemotion_ = nullptr;
  std::string device_id_;
  double pre_azimuth = 0;  // a invalid value
  double pre_timestamp_ = 0;
  double pre_camera_timestamp_ = 0;
  double camera_timestamp_ = 0;
  bool start_flag_ = false;
  const int motion_buffer_size_ = 60;
  Mutex mutex_;
  Mutex image_mutex_;
  std::list<VehicleInformation> vehicle_information_buffer_;
  CameraSharedData *camera_shared_data_ = nullptr;
  DISALLOW_COPY_AND_ASSIGN(MotionService);
};

REGISTER_SUBNODE(MotionService);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_MOTION_SERVICE_H_
