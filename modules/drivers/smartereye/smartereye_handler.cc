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
#include "modules/drivers/smartereye/smartereye_handler.h"

#include <math.h>
#include <iostream>
#include <string>

#include "third_party/camera_library/smartereye/include/LdwDataInterface.h"
#include "third_party/camera_library/smartereye/include/disparityconvertor.h"
#include "third_party/camera_library/smartereye/include/frameid.h"
#include "third_party/camera_library/smartereye/include/obstacleData.h"
#include "third_party/camera_library/smartereye/include/satpext.h"

namespace apollo {
namespace drivers {
namespace smartereye {

SmartereyeHandler::SmartereyeHandler(std::string name)
    : mName(name) {
  pCallbackFunc = nullptr;
}

SmartereyeHandler::~SmartereyeHandler() {
  pCallbackFunc = nullptr;
}

bool SmartereyeHandler::SetCallback(CallbackFunc ptr) {
  pCallbackFunc = ptr;

  return true;
}

void SmartereyeHandler::handleRawFrame(const RawImageFrame *rawFrame) {
  pCallbackFunc(const_cast<RawImageFrame *>(rawFrame));
}

}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
