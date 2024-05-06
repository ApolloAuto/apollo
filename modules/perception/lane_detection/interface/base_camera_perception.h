/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

struct CameraPerceptionInitOptions : public BaseInitOptions {
  // TODO(Xun): modified to be configurable
  std::string lane_calibration_working_sensor_name = "front_6mm";
  std::string calibrator_method = "LaneLineCalibrator";
};

struct CameraPerceptionOptions {};

class BaseCameraPerception {
 public:
  BaseCameraPerception() = default;
  virtual ~BaseCameraPerception() = default;

  virtual bool Init(const CameraPerceptionInitOptions &init_options) = 0;
  virtual bool Perception(const CameraPerceptionOptions &options,
                          CameraFrame *frame) = 0;

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseCameraPerception);
};

PERCEPTION_REGISTER_REGISTERER(BaseCameraPerception);
#define PERCEPTION_REGISTER_CAMERA_PERCEPTION(name) \
  PERCEPTION_REGISTER_CLASS(BaseCameraPerception, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
