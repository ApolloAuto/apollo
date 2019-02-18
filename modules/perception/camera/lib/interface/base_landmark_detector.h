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

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

#include "modules/perception/camera/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace camera {

struct LandmarkDetectorInitOptions : public BaseInitOptions {};

struct LandmarkDetectorOptions {};

class BaseLandmarkDetector {
 public:
  BaseLandmarkDetector() = default;

  virtual ~BaseLandmarkDetector() = default;

  virtual bool Init(const LandmarkDetectorInitOptions& options =
                        LandmarkDetectorInitOptions()) = 0;

  // @brief: detect landmark from image.
  // @param [in]: options
  // @param [in/out]: frame
  // landmark type and 2D bbox should be filled, required,
  virtual bool Detect(const LandmarkDetectorOptions& options,
                      CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  BaseLandmarkDetector(const BaseLandmarkDetector&) = delete;
  BaseLandmarkDetector& operator=(const BaseLandmarkDetector&) = delete;
};  // class BaseLandmarkDetector

PERCEPTION_REGISTER_REGISTERER(BaseLandmarkDetector);
#define REGISTER_LANDMARK_DETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseLandmarkDetector, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
