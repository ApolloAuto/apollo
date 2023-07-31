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
#include "modules/perception/camera_tracking/feature_extract/external_feature_extractor.h"

#include <map>
#include <vector>

#include "cyber/common/file.h"
#include "modules/perception/common/camera/common/global_config.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/inference/utils/resize.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

bool ExternalFeatureExtractor::Init(
    const FeatureExtractorInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }

  AINFO << "Load config Success: " << model_param_.ShortDebugString();

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());
  std::string proto_file =
      GetModelFile(model_path, model_info.proto_file().file());
  std::string weight_file =
      GetModelFile(model_path, model_info.weight_file().file());

  // Network input and output names
  std::vector<std::string> input_names =
      inference::GetBlobNames(model_info.inputs());
  std::vector<std::string> output_names =
      inference::GetBlobNames(model_info.outputs());

  height_ = model_param_.resize().height();
  width_ = model_param_.resize().width();

  net_.reset(inference::CreateInferenceByName(
      model_info.framework(), proto_file, weight_file, output_names,
      input_names, model_path));
  ACHECK(nullptr != net_) << "Failed to init CNNAdapter";

  gpu_id_ = GlobalConfig::Instance()->track_feature_gpu_id;
  net_->set_gpu_id(gpu_id_);

  std::map<std::string, std::vector<int>> shape_map;
  inference::AddShape(&shape_map, model_info.inputs());
  inference::AddShape(&shape_map, model_info.outputs());
  if (!net_->Init(shape_map)) {
    AERROR << model_info.name() << "init failed!";
    return false;
  }

  net_->Infer();

  InitFeatureExtractor(options);
  image_.reset(new base::Image8U(height_, width_, base::Color::BGR));
  return true;
}

bool ExternalFeatureExtractor::InitFeatureExtractor(
    const FeatureExtractorInitOptions &options) {
  FeatureExtractorInitOptions feat_options;
  feat_options.config_path = options.feature_path;
  feat_options.config_file = options.feature_file;
  feat_options.gpu_id = gpu_id_;
  // feat_blob is the 0th of the output blobs
  auto feat_blob_name = model_param_.info().outputs(0).name();
  feat_options.feat_blob = net_->get_blob(feat_blob_name);
  feat_options.input_height = height_;
  feat_options.input_width = width_;
  feature_extractor_.reset(BaseFeatureExtractorRegisterer::GetInstanceByName(
      "TrackingFeatureExtractor"));
  feature_extractor_->Init(feat_options);
  return true;
}

bool ExternalFeatureExtractor::Extract(const FeatureExtractorOptions &options,
                                       CameraTrackingFrame *frame) {
  int raw_height = frame->data_provider->src_height();
  int raw_width = frame->data_provider->src_width();
  auto input_blob = net_->get_blob(model_param_.info().inputs(0).name());
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  auto offset_y_ = static_cast<int>(
      model_param_.offset_ratio() * static_cast<float>(raw_height) + 0.5f);
  image_options.crop_roi =
      base::RectI(0, offset_y_, raw_width, raw_height - offset_y_);
  image_options.do_crop = true;
  // Timer timer;
  frame->data_provider->GetImage(image_options, image_.get());
  inference::ResizeGPU(*image_, input_blob, raw_width, 0);
  net_->Infer();
  FeatureExtractorOptions feat_options;
  feat_options.normalized = false;
  feature_extractor_->set_roi(
      image_options.crop_roi.x, image_options.crop_roi.y,
      image_options.crop_roi.width, image_options.crop_roi.height);
  feature_extractor_->Extract(feat_options, frame);
  AINFO << "Extract Done";
  return true;
}

REGISTER_FEATURE_EXTRACTOR(ExternalFeatureExtractor);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
