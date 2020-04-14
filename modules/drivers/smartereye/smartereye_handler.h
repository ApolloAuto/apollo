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

using namespace std;
using namespace std::placeholders;

namespace apollo {
namespace drivers {
namespace smartereye {

typedef std::function<bool(RawImageFrame *rawFrame)> CallbackFunc;

class SmartereyeHandler : public CameraHandler {
 public:
  SmartereyeHandler(std::string name);
  ~SmartereyeHandler();
  void handleRawFrame(const RawImageFrame *rawFrame);
  bool SetCallback(CallbackFunc ptr);
  void handleUpdateFinished(Result result);
  void setStereoCalibParams(StereoCalibrationParameters &params);
  void setRotationMatrix(RotationMatrix &rotationMatrix);

 protected:
  void processFrame(int frameId, char *image, uint32_t dataSize, int width,
                    int height, int frameFormat);
  void handleDisparityPointByPoint(unsigned char *image, int width, int height,
                                   int bitNum);
  void handleDisparityByLookupTable(unsigned char *image, int width, int height,
                                    int bitNum);

 private:
  std::string mName;
  bool mIsLookupTableGenerated;
  Result mUpgradeResult;
  StereoCalibrationParameters mStereoCalibrationParameters;
  bool mIsCalibParamReady;
  RotationMatrix mRotationMatrix;
  CallbackFunc pCallbackFunc;
};
}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
