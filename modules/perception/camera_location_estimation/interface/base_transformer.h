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
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/onboard/inner_component_messages/camera_detection_component_messages.h"

namespace apollo {
namespace perception {
namespace camera {

struct TransformerInitOptions : public BaseInitOptions {};

class BaseTransformer {
 public:
  BaseTransformer() = default;

  virtual ~BaseTransformer() = default;
  /**
   * @brief Init transfomer interface
   *
   * @param options options for inference.
   * @return true
   * @return false
   */
  virtual bool Init(
      const TransformerInitOptions& options = TransformerInitOptions()) = 0;
  /**
   * @brief Transform 2d obstacle to 3D obstacle interface.
   *
   * @param frame camera frame
   * @return true
   * @return false
   */
  virtual bool Transform(onboard::CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  DISALLOW_COPY_AND_ASSIGN(BaseTransformer);
};  // class BaseTransformer

PERCEPTION_REGISTER_REGISTERER(BaseTransformer);
#define REGISTER_OBSTACLE_TRANSFORMER(name) \
  PERCEPTION_REGISTER_CLASS(BaseTransformer, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
