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

#pragma once

#include <math.h>
#include <stdio.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "cyber/cyber.h"
#include "modules/drivers/smartereye/proto/config.pb.h"
#include "modules/drivers/smartereye/smartereye_handler.h"
#include "third_party/camera_library/smartereye/include/LdwDataInterface.h"
#include "third_party/camera_library/smartereye/include/calibrationparams.h"
#include "third_party/camera_library/smartereye/include/camerahandler.h"
#include "third_party/camera_library/smartereye/include/disparityconvertor.h"
#include "third_party/camera_library/smartereye/include/frameid.h"
#include "third_party/camera_library/smartereye/include/obstacleData.h"
#include "third_party/camera_library/smartereye/include/rotationmatrix.h"
#include "third_party/camera_library/smartereye/include/satpext.h"
#include "third_party/camera_library/smartereye/include/stereocamera.h"
#include "third_party/camera_library/smartereye/include/taskiddef.h"

namespace apollo {
namespace drivers {
namespace smartereye {

using apollo::drivers::smartereye::config::Config;

class SmartereyeDevice {
 public:
  SmartereyeDevice();
  virtual ~SmartereyeDevice();
  virtual bool init(const std::shared_ptr<Config> &camera_config);
  bool is_capturing();
  bool wait_for_device();
  int uninit();
  bool SetCallback(CallbackFunc ptr);
  int poll();

 private:
  bool is_capturing_ = false;
  bool inited_ = false;
  std::shared_ptr<Config> config_;
  StereoCamera *pcamera_ = nullptr;
  SmartereyeHandler *pcameraHandler_ = nullptr;
};

}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
