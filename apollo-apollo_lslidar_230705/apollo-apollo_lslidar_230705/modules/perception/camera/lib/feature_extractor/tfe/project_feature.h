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

#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/utils/gemm.h"
#include "modules/perception/pipeline/proto/stage/tracking_feature.pb.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace camera {

class ProjectFeature : public BaseFeatureExtractor {
 public:
  ProjectFeature() = default;
  ~ProjectFeature() = default;

  bool Init(const FeatureExtractorInitOptions &init_options) override;
  bool Extract(const FeatureExtractorOptions &options,
               CameraFrame *frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  std::shared_ptr<inference::Inference> inference_;
  int gpu_id_ = 0;
  tracking_feature::ExternalParam param_;
  inference::GPUL2Norm norm_;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
