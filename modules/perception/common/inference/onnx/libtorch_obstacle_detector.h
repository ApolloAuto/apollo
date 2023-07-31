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

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <torch/script.h>
#include <torch/torch.h>

#include "modules/perception/common/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {


class ObstacleDetector : public Inference {
 public:
  ObstacleDetector(const std::string &net_file, const std::string &model_file,
           const std::vector<std::string> &outputs);

  ObstacleDetector(const std::string &net_file, const std::string &model_file,
           const std::vector<std::string> &outputs,
           const std::vector<std::string> &inputs);

  virtual ~ObstacleDetector() {}

  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;

  void Infer() override;
  base::BlobPtr<float> get_blob(const std::string &name) override;

 protected:
  bool shape(const std::string &name, std::vector<int> *res);
  torch::jit::script::Module net_;

 private:
  std::string net_file_;
  std::string model_file_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;

  torch::DeviceType device_type_;
  int device_id_ = 0;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
