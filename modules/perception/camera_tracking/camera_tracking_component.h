/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "modules/perception/camera_tracking/proto/omt.pb.h"
#include "modules/perception/camera_tracking/tracking/omt_obstacle_tracker.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"
#include "modules/perception/common/onboard/inner_component_messages/inner_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

class CameraTrackingComponent
    : public apollo::cyber::Component<onboard::CameraFrame> {
 public:
  CameraTrackingComponent() = default;
  ~CameraTrackingComponent() = default;

  bool Init() override;
  bool Proc(
      const std::shared_ptr<onboard::CameraFrame>& message) override;

 private:
  bool InternalProc(
      const std::shared_ptr<const onboard::CameraFrame>& in_message,
      std::shared_ptr<onboard::SensorFrameMessage> out_message);

  std::string output_channel_name_;
  std::shared_ptr<BaseObstacleTracker> camera_obstacle_tracker_;
  std::shared_ptr<apollo::cyber::Writer<onboard::SensorFrameMessage>> writer_;
  algorithm::SensorManager* sensor_manager_ = nullptr;
  OmtParam camera_tracker_config_;
};

CYBER_REGISTER_COMPONENT(CameraTrackingComponent);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
