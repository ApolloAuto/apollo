/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "paddle/include/paddle_inference_api.h"

#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

using BlobPtr = std::shared_ptr<apollo::perception::base::Blob<float>>;

class PaddleInference : public Inference {
 public:
  // combined model
  PaddleInference(const std::string &model_file, const std::string &para_file,
                  const std::vector<std::string> &outputs,
                  const std::vector<std::string> &inputs);

  // uncombined model
  PaddleInference(const std::string &model_dir,
                  const std::vector<std::string> &outputs,
                  const std::vector<std::string> &inputs);

  virtual ~PaddleInference() {}

  //必须override的函数
  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
  void Infer() override;
  BlobPtr get_blob(const std::string &name) override;

 protected:
  bool shape(const std::string &name, std::vector<int> *res);

 private:
  std::string model_file_;
  std::string para_file_;
  std::string model_dir_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<float> out_data_;
  std::unique_ptr<paddle::PaddlePredictor> predictor_;
  std::vector<int> output_shape_;
  //todo 确认init函数被调用的频率，确认是否可以将config的定义放到init函数里
  paddle_infer::Config config_; 
  BlobMap blobs_;

  //use gpu or not
  bool use_gpu_ ;
  //use mkldnn or not
  //use memory opotim or not
  int device_id_ = 0;


  int gpu_id_ = 0;
  bool enable_mkldnn_ = true;
  bool enable_mem_opt_ = true;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
