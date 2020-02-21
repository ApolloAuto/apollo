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
#include "modules/perception/camera/lib/feature_extractor/tfe/project_feature.h"

#include <map>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/global_config.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/utils/gemm.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

bool ProjectFeature::Init(const FeatureExtractorInitOptions &options) {
  std::string efx_config = GetAbsolutePath(options.root_dir, options.conf_file);
  ACHECK(cyber::common::GetProtoFromFile(efx_config, &param_))
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
  const auto &model_type = param_.model_type();
  AINFO << "model_type=" << model_type;
  inference_.reset(inference::CreateInferenceByName(
      model_type, proto_file, weight_file, output_names, input_names,
      options.root_dir));
  ACHECK(nullptr != inference_) << "Failed to init CNNAdapter";
  gpu_id_ = GlobalConfig::Instance()->track_feature_gpu_id;
  inference_->set_gpu_id(gpu_id_);
  inference_->set_max_batch_size(100);
  std::vector<int> shape = {5, 64, 3, 3};
  std::map<std::string, std::vector<int>> shape_map{
      {param_.input_blob(), shape}};

  ACHECK(inference_->Init(shape_map));
  inference_->Infer();
  return true;
}

bool ProjectFeature::Extract(const FeatureExtractorOptions &options,
                             CameraFrame *frame) {
  auto input_blob = inference_->get_blob(param_.input_blob());
  auto output_blob = inference_->get_blob(param_.feat_blob());
  if (frame->detected_objects.empty()) {
    return true;
  }
  input_blob->Reshape(frame->track_feature_blob->shape());
  cudaMemcpy(
      input_blob->mutable_gpu_data(), frame->track_feature_blob->gpu_data(),
      frame->track_feature_blob->count() * sizeof(float), cudaMemcpyDefault);

  cudaDeviceSynchronize();
  inference_->Infer();
  cudaDeviceSynchronize();
  frame->track_feature_blob->Reshape(
      {static_cast<int>(frame->detected_objects.size()), output_blob->shape(1),
       output_blob->shape(2), output_blob->shape(3)});

  cudaMemcpy(
      frame->track_feature_blob->mutable_gpu_data(), output_blob->gpu_data(),
      frame->track_feature_blob->count() * sizeof(float), cudaMemcpyDefault);

  norm_.L2Norm(frame->track_feature_blob.get());
  return true;
}

std::string ProjectFeature::Name() const { return "ProjectFeature"; }

REGISTER_FEATURE_EXTRACTOR(ProjectFeature);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
