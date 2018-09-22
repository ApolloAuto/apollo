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

#ifndef MODULES_DRIVERS_USB_CAM_USB_CAM_COMPONENT_H
#define MODULES_DRIVERS_USB_CAM_USB_CAM_COMPONENT_H

#include <memory>

#include "cybertron/cybertron.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/usb_cam/proto/config.pb.h"

#include "modules/drivers/usb_cam/usb_cam.h"

namespace apollo {
namespace drivers {
namespace usb_cam {

using apollo::cybertron::Component;
using apollo::cybertron::Reader;
using apollo::cybertron::Writer;
using apollo::drivers::Image;
using apollo::drivers::usb_cam::config::Config;

class UsbCamComponent : public Component<> {
 public:
  bool Init() override;

 private:
  void run();

  std::shared_ptr<Writer<Image>> writer_ = nullptr;
  std::unique_ptr<UsbCam> camera_device_;
  std::shared_ptr<Config> camera_config_;
  CameraImagePtr raw_image_ = nullptr;
  std::shared_ptr<Image> pb_image_ = nullptr;
  float spin_rate_ = 0.005;
  float device_wait_ = 2.0;
};

CYBERTRON_REGISTER_COMPONENT(UsbCamComponent)
}  // namespace usb_cam
}  // namespace drivers
}  // namespace apollo

#endif  // ONBOARD_DRIVERS_USB_CAMERA_INCLUDE_USB_CAMERA_COMPONENT_H
