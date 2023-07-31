/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/perception/common/proto/motion_service.pb.h"
#include "modules/perception/motion_service/proto/motion_service_component.pb.h"

#include "cyber/common/macros.h"
#include "cyber/component/component.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/motion_service/motion/plane_motion.h"

namespace apollo {
namespace perception {
namespace camera {

typedef std::shared_ptr<apollo::drivers::Image> ImageMsgType;
typedef std::shared_ptr<localization::LocalizationEstimate> LocalizationMsgType;

class MotionServiceComponent final : public apollo::cyber::Component<> {
 public:
  MotionServiceComponent() = default;
  virtual ~MotionServiceComponent() { delete vehicle_planemotion_; }

  bool Init() override;
  bool GetMotionInformation(double timestamp, base::VehicleStatus *vs);
  base::MotionBuffer GetMotionBuffer();
  double GetLatestTimestamp();

 private:
  void OnLocalization(const LocalizationMsgType &localization);
  void OnReceiveImage(const ImageMsgType &message,
                      const std::string &camera_name);
  void PublishEvent(const double timestamp);
  void ConvertVehicleMotionToMsgOut(
      base::VehicleStatus vs, apollo::perception::VehicleStatus *v_status_msg);

  PlaneMotion *vehicle_planemotion_ = nullptr;
  std::string device_id_;
  double pre_azimuth = 0;  // a invalid value
  double pre_timestamp_ = 0;
  double pre_camera_timestamp_ = 0;
  double camera_timestamp_ = 0;
  bool start_flag_ = false;
  const int motion_buffer_size_ = 100;
  double timestamp_offset_ = 0.0;

  std::vector<std::string> camera_names_;  // camera sensor names
  std::vector<std::string> input_camera_channel_names_;
  std::mutex mutex_;
  //   std::mutex image_mutex_;
  std::mutex motion_mutex_;
  std::shared_ptr<apollo::cyber::Writer<apollo::perception::MotionService>>
      writer_;
  DISALLOW_COPY_AND_ASSIGN(MotionServiceComponent);
};

CYBER_REGISTER_COMPONENT(MotionServiceComponent);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
