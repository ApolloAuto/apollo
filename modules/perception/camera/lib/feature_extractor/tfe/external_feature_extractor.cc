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
#include "modules/perception/camera/lib/feature_extractor/tfe/external_feature_extractor.h"

#include <map>
#include <vector>

#include "cyber/common/file.h"
#include "modules/perception/camera/common/global_config.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/utils/resize.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool ExternalFeatureExtractor::Init(
    const FeatureExtractorInitOptions &options) {
  std::string efx_config = GetAbsolutePath(options.root_dir, options.conf_file);
  CHECK(cyber::common::GetProtoFromFile(efx_config, &param_))
      << "Read config failed: " << efx_config;
  AINFO << "Load config Success: " << param_.ShortDebugString();
  std::string proto_file =
      GetAbsolutePath(options.root_dir, param_.proto_file());
  std::string weight_file =
      GetAbsolutePath(options.root_dir, param_.weight_file());
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  input_names.push_back(param_.input_blob());
  output_names.push_back(param_.feat_blob());
  height_ = param_.resize_height();
  width_ = param_.resize_width();
  const auto &model_type = param_.model_type();
  AINFO << "model_type=" << model_type;
  inference_.reset(inference::CreateInferenceByName(
      model_type, proto_file, weight_file, output_names, input_names,
      options.root_dir));
  CHECK(nullptr != inference_) << "Failed to init CNNAdapter";
  gpu_id_ = GlobalConfig::Instance()->track_feature_gpu_id;
  inference_->set_gpu_id(gpu_id_);
  std::vector<int> shape = {1, height_, width_, 3};
  std::map<std::string, std::vector<int>> shape_map{
      {param_.input_blob(), shape}};

  CHECK(inference_->Init(shape_map));
  inference_->Infer();
  InitFeatureExtractor(options.root_dir);
  image_.reset(new base::Image8U(height_, width_, base::Color::BGR));
  return true;
}
bool ExternalFeatureExtractor::InitFeatureExtractor(
    const std::string &root_dir) {
  FeatureExtractorInitOptions feat_options;
  feat_options.conf_file = param_.feature_file();
  feat_options.root_dir = root_dir;
  feat_options.gpu_id = gpu_id_;
  auto feat_blob_name = param_.feat_blob();
  feat_options.feat_blob = inference_->get_blob(feat_blob_name);
  feat_options.input_height = height_;
  feat_options.input_width = width_;
  feature_extractor_.reset(BaseFeatureExtractorRegisterer::GetInstanceByName(
      "TrackingFeatureExtractor"));
  feature_extractor_->Init(feat_options);
  return true;
}
bool ExternalFeatureExtractor::Extract(const FeatureExtractorOptions &options,
                                       CameraFrame *frame) {
  int raw_height = frame->data_provider->src_height();
  int raw_width = frame->data_provider->src_width();
  auto input_blob = inference_->get_blob(param_.input_blob());
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  auto offset_y_ = static_cast<int>(
      param_.offset_ratio() * static_cast<float>(raw_height) + 0.5f);
  image_options.crop_roi =
      base::RectI(0, offset_y_, raw_width, raw_height - offset_y_);
  image_options.do_crop = true;
  // Timer timer;
  frame->data_provider->GetImage(image_options, image_.get());
  inference::ResizeGPU(*image_, input_blob, raw_width, 0);
  inference_->Infer();
  FeatureExtractorOptions feat_options;
  feat_options.normalized = false;
  feature_extractor_->set_roi(
      image_options.crop_roi.x, image_options.crop_roi.y,
      image_options.crop_roi.width, image_options.crop_roi.height);
  feature_extractor_->Extract(feat_options, frame);
  AINFO << "Extract Done";
  return true;
}
std::string ExternalFeatureExtractor::Name() const {
  return "ExternalFeatureExtractor";
}
REGISTER_FEATURE_EXTRACTOR(ExternalFeatureExtractor);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
