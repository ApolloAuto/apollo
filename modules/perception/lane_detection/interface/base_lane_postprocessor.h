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

struct LanePostprocessorInitOptions : public BaseInitOptions {
  std::string detect_config_root;
  std::string detect_config_name;
};

struct LanePostprocessorOptions {};

class BaseLanePostprocessor {
 public:
  BaseLanePostprocessor() = default;

  virtual ~BaseLanePostprocessor() = default;

  virtual bool Init(const LanePostprocessorInitOptions& options =
                        LanePostprocessorInitOptions()) = 0;

  // @brief: refine 3D location of detected lanes.
  // @param [in]: options
  // @param [in/out]: frame
  // 3D information of lane should be filled, required.
  virtual bool Process2D(const LanePostprocessorOptions& options,
                         CameraFrame* frame) = 0;

  virtual bool Process3D(const LanePostprocessorOptions& options,
                         CameraFrame* frame) = 0;

  virtual void SetIm2CarHomography(const Eigen::Matrix3d& homography_im2car) {
    // do nothing
  }

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseLanePostprocessor);
};  // class BaseLanePostprocessor

PERCEPTION_REGISTER_REGISTERER(BaseLanePostprocessor);
#define REGISTER_LANE_POSTPROCESSOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseLanePostprocessor, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
