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

#include <functional>
#include <iostream>
#include <string>

#include "cyber/cyber.h"
#include "third_party/camera_library/smartereye/include/calibrationparams.h"
#include "third_party/camera_library/smartereye/include/camerahandler.h"
#include "third_party/camera_library/smartereye/include/rotationmatrix.h"

namespace apollo {
namespace drivers {
namespace smartereye {

typedef std::function<bool(RawImageFrame *rawFrame)> CallbackFunc;

class SmartereyeHandler : public CameraHandler {
 public:
  explicit SmartereyeHandler(std::string name);
  ~SmartereyeHandler();
  void handleRawFrame(const RawImageFrame *rawFrame);
  bool SetCallback(CallbackFunc ptr);
  void handleUpdateFinished(Result result) {}

 protected:
  void handleDisparityPointByPoint(unsigned char *image, int width, int height,
                                   int bitNum) {}
  void handleDisparityByLookupTable(unsigned char *image, int width, int height,
                                    int bitNum) {}

 private:
  std::string mName = nullptr;
  CallbackFunc pCallbackFunc = nullptr;
};

}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
