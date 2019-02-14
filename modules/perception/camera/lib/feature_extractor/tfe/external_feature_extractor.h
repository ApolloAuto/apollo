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
#pragma once  // NOLINT

#include <memory>
#include <string>

#include "modules/perception/camera/lib/feature_extractor/tfe/tracking_feature.pb.h"
#include "modules/perception/camera/lib/interface/base_feature_extractor.h"
#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace camera {
class ExternalFeatureExtractor : public BaseFeatureExtractor {
 public:
  bool Init(const FeatureExtractorInitOptions &init_options) override;
  bool Extract(const FeatureExtractorOptions &options,
               CameraFrame *frame) override;
  std::string Name() const override;

 private:
  std::shared_ptr<base::Image8U> image_ = nullptr;
  std::shared_ptr<inference::Inference> inference_;
  std::shared_ptr<BaseFeatureExtractor> feature_extractor_;
  tracking_feature::ExternalParam param_;
  int height_;
  int width_;
  bool InitFeatureExtractor(const std::string &root_dir);
  int gpu_id_;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
