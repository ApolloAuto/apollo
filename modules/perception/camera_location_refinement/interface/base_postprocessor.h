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
#include "modules/perception/common/lib/interface/base_calibration_service.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

struct PostprocessorInitOptions : public BaseInitOptions {
  std::shared_ptr<BaseCalibrationService> calibration_service;
};

struct PostprocessorOptions {
  bool do_refinement_with_disp_map = false;
  bool do_refinement_with_calibration_service = true;
};

class BasePostprocessor {
 public:
  BasePostprocessor() = default;

  virtual ~BasePostprocessor() = default;
  /**
   * @brief Init postprocessor
   *
   * @param options
   * @return true
   * @return false
   */
  virtual bool Init(
      const PostprocessorInitOptions& options = PostprocessorInitOptions()) = 0;

  /**
   * @brief refine 3D location of detected obstacles.3D information of obstacle
   * should be filled, required.
   *
   * @param options
   * @param frame
   * @return true
   * @return false
   */
  virtual bool Process(const PostprocessorOptions& options,
                       onboard::CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BasePostprocessor);
};  // class BasePostprocessor

PERCEPTION_REGISTER_REGISTERER(BasePostprocessor);
#define REGISTER_OBSTACLE_POSTPROCESSOR(name) \
  PERCEPTION_REGISTER_CLASS(BasePostprocessor, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
