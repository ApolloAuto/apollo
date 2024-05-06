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
#include "modules/perception/camera_tracking/feature_extract/project_feature.h"

#include <map>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/camera/common/global_config.h"
#include "modules/perception/common/inference/inference_factory.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/inference/utils/gemm.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

bool ProjectFeature::Init(const FeatureExtractorInitOptions &options) {
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

  net_.reset(inference::CreateInferenceByName(
      model_info.framework(), proto_file, weight_file, output_names,
      input_names, model_path));
  ACHECK(nullptr != net_) << "Failed to init CNNAdapter";

  gpu_id_ = GlobalConfig::Instance()->track_feature_gpu_id;
  net_->set_gpu_id(gpu_id_);
  net_->set_max_batch_size(100);

  std::map<std::string, std::vector<int>> shape_map;
  inference::AddShape(&shape_map, model_info.inputs());
  inference::AddShape(&shape_map, model_info.outputs());

  if (!net_->Init(shape_map)) {
    AERROR << model_info.name() << "init failed!";
    return false;
  }

  net_->Infer();
  return true;
}

bool ProjectFeature::Extract(const FeatureExtractorOptions &options,
                             CameraTrackingFrame *frame) {
  auto input_blob = net_->get_blob(model_param_.info().inputs(0).name());
  auto output_blob = net_->get_blob(model_param_.info().outputs(0).name());
  if (frame->detected_objects.empty()) {
    return true;
  }
  input_blob->Reshape(frame->track_feature_blob->shape());
  cudaMemcpy(
      input_blob->mutable_gpu_data(), frame->track_feature_blob->gpu_data(),
      frame->track_feature_blob->count() * sizeof(float), cudaMemcpyDefault);

  cudaDeviceSynchronize();
  net_->Infer();
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

REGISTER_FEATURE_EXTRACTOR(ProjectFeature);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
