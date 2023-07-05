/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/smartereye/smartereye_device.h"

#include <time.h>

#include <cmath>
#include <string>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace apollo {
namespace drivers {
namespace smartereye {

SmartereyeDevice::SmartereyeDevice() {}

SmartereyeDevice::~SmartereyeDevice() { uninit(); }

bool SmartereyeDevice::init(const std::shared_ptr<Config>& camera_config) {
  pcamera_ = StereoCamera::connect("192.168.1.251");
  pcameraHandler_ = new SmartereyeHandler("camera A");
  pcamera_->enableTasks(TaskId::ObstacleTask | TaskId::DisplayTask);
  inited_ = true;

  return true;
}

bool SmartereyeDevice::SetCallback(CallbackFunc ptr) {
  pcameraHandler_->SetCallback(ptr);

  return true;
}

int SmartereyeDevice::poll() {
  pcamera_->requestFrame(pcameraHandler_, FrameId::Compound);
  is_capturing_ = true;

  return 1;
}

int SmartereyeDevice::uninit() {
  if (!inited_) {
    return 1;
  }

  pcamera_->disconnectFromServer();
  is_capturing_ = false;
  inited_ = false;

  return 1;
}

bool SmartereyeDevice::is_capturing() { return is_capturing_; }

bool SmartereyeDevice::wait_for_device() {
  if (is_capturing_) {
    ADEBUG << "is capturing";
    return true;
  }

  return true;
}

}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
