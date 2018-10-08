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

#pragma once


#include <memory>

#include "cybertron/cybertron.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/camera/proto/config.pb.h"

#include "modules/drivers/camera/usb_cam.h"

namespace apollo {
namespace drivers {
namespace camera {

using apollo::cybertron::Component;
using apollo::cybertron::Reader;
using apollo::cybertron::Writer;
using apollo::drivers::Image;
using apollo::drivers::camera::config::Config;

class CameraComponent : public Component<> {
 public:
  bool Init() override;

 private:
  void run();

  std::shared_ptr<Writer<Image>> writer_ = nullptr;
  std::unique_ptr<UsbCam> camera_device_;
  std::shared_ptr<Config> camera_config_;
  CameraImagePtr raw_image_ = nullptr;
  std::shared_ptr<Image> pb_image_ = nullptr;
  uint32_t spin_rate_ = 200;
  uint32_t device_wait_ = 2000;
};

CYBERTRON_REGISTER_COMPONENT(CameraComponent)
}  // namespace camera
}  // namespace drivers
}  // namespace apollo


