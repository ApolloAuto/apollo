/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "paddle_inference.h"

#include <numeric>



namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

//构造函数
// combined model
PaddleInference::PaddleInference(const std::string &model_file,
                                 const std::string &para_file,
                                 const std::vector<std::string> &inputs,
                                 const std::vector<std::string> &outputs)
    : model_file_(model_file),
      para_file_(para_file),
      input_names_(inputs),
      output_names_(outputs) {}

// uncombined model
PaddleInference::PaddleInference(const std::string &model_dir,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs)
    : model_dir_(model_dir), input_names_(inputs), output_names_(outputs){};

// set all kinds of config
bool PaddleInference::Init(
    const std::map<std::string, std::vector<int>> &shapes) {
  // deal with blob

  for (const auto &name : input_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }

  for (const auto &name : output_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }

  // deal with config
  if (model_dir_ != "") {
    config_.SetModel(model_dir_);
  } else {
    config_.SetModel(model_file_, para_file_);
  }
  if (gpu_id_ >= 0) {
    //(memorySize， deviceId)
    // todo 确认memorySize的大小
    config_.EnableUseGpu(100, 0);
  }
  if (enable_mkldnn_) config_.EnableMKLDNN();
  if (enable_mem_opt_) config_.EnableMemoryOptim();
  // todo: 配置tensorRT
  paddle_infer::CreatePredictor(config_);
  return true;
}


std::shared_ptr<Blob<float>> PaddleInference::get_blob(
    const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}


void PaddleInference::Infer() {
  //  根据blob数据创建tensor、进行推理、获取结果
  //  input[0]  input tensor
  //  input[1]  input tensor shape
  //  output[0] output tensor

  // todo 做更详细的分析和数据验证
  
  // name
  auto input_data = predictor_->GetInputTensor(input_names_[0]);
  // size
  input_data->Reshape(blobs_[input_names_[0]]->shape());
  // data
  auto blob = blobs_[input_names_[0]];
  //input_data->copy_to_cpu((blobs_[input_names_[0]])->data()->mutable_gpu_data());
  //mutable_cpu_data返回的是 void * 数据，   而copy_to_cpu返回的 T * 数据，故需要进行类型转换
  input_data->copy_to_cpu(static_cast<float *>(blob->data()->mutable_cpu_data()));
  //input_data->copy_to_cpu(output_shape_.data());
                         

  // name
  auto input_size = predictor_->GetInputTensor(input_names_[1]);
  // size
  input_size->Reshape(blobs_[input_names_[1]]->shape());
  // data
  input_size->copy_from_cpu(static_cast<float *>(blobs_[input_names_[1]]->data()->mutable_cpu_data()));
   
  predictor_->ZeroCopyRun();
 

  auto output_t = predictor_->GetOutputTensor(output_names_[0]);
  this->output_shape_ = output_t->shape();
  int out_num = std::accumulate(output_shape_.begin(), output_shape_.end(), 1,
                               std::multiplies<int>());
  out_data_.resize(out_num);
  output_t->copy_to_cpu(static_cast<float *>(blobs_[output_names_[0]]->data()->mutable_cpu_data())); 
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo