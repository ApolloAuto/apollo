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

struct InferenceEngineInitOptions : public BaseInitOptions {};

struct InferenceEngineOptions {};

class BaseInferenceEngine {
 public:
  BaseInferenceEngine() = default;

  virtual ~BaseInferenceEngine() = default;

  virtual bool Init(const InferenceEngineInitOptions& options =
                        InferenceEngineInitOptions()) = 0;

  // @brief: do network inference.
  // @param [in]: options
  // output blobs should be filled, required,
  virtual bool Infer(const InferenceEngineOptions& options,
                     CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  BaseInferenceEngine(const BaseInferenceEngine&) = delete;
  BaseInferenceEngine& operator=(const BaseInferenceEngine&) = delete;
};  // class BaseInferenceEngine

PERCEPTION_REGISTER_REGISTERER(BaseInferenceEngine);
#define REGISTER_INFERENCE_ENGINE(name) \
  PERCEPTION_REGISTER_CLASS(BaseInferenceEngine, name)

}  // namespace camera
}  // namespace perception
}  // namespace apollo
