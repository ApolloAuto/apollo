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

#include <memory>
#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/base/camera.h"
#include "modules/perception/common/camera/common/camera_frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

struct LaneDetectorInitOptions : public BaseInitOptions {
  std::shared_ptr<base::BaseCameraModel> base_camera_model = nullptr;
  int gpu_id = 0;
};

struct LaneDetectorOptions {};

class BaseLaneDetector {
 public:
  BaseLaneDetector() = default;

  virtual ~BaseLaneDetector() = default;

  virtual bool Init(
      const LaneDetectorInitOptions& options = LaneDetectorInitOptions()) = 0;

  // @brief: detect lane from image.
  // @param [in]: options
  // @param [in/out]: frame
  // detected lanes should be filled, required,
  // 3D information of lane can be filled, optional.
  virtual bool Detect(const LaneDetectorOptions& options,
                      CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseLaneDetector);
};  // class BaseLaneDetector

PERCEPTION_REGISTER_REGISTERER(BaseLaneDetector);
#define REGISTER_LANE_DETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseLaneDetector, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
