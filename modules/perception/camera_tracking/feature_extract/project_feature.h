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

#include "modules/perception/camera_tracking/feature_extract/proto/model_param.pb.h"

#include "modules/perception/camera_tracking/interface/base_feature_extractor.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/utils/gemm.h"

namespace apollo {
namespace perception {
namespace camera {

class ProjectFeature : public BaseFeatureExtractor {
 public:
  ProjectFeature() = default;
  ~ProjectFeature() = default;

  bool Init(const FeatureExtractorInitOptions &init_options) override;
  bool Extract(const FeatureExtractorOptions &options,
               CameraTrackingFrame *frame) override;

 private:
  std::shared_ptr<inference::Inference> net_;
  int gpu_id_ = 0;
  tracking_feature::ModelParam model_param_;
  inference::GPUL2Norm norm_;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
