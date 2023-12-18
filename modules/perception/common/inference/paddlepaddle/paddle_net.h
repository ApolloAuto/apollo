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
#include <string>
#include <vector>
#include <memory>

#include "paddle/include/paddle_inference_api.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/inference/inference.h"


namespace apollo {
namespace perception {
namespace inference {

constexpr uint64_t MemoryPoolInitSizeMb = 1000;

class PaddleNet : public Inference {
 public:
  PaddleNet(const std::string& model_file,
            const std::string& params_file,
            const std::vector<std::string>& outputs,
            const std::vector<std::string>& inputs);
  virtual ~PaddleNet();

  bool Init(const std::map<std::string, std::vector<int>>& shapes) override;

  void Infer() override;

  base::BlobPtr<float> get_blob(const std::string& name) override;

 protected:
  bool shape(const std::string &name, std::vector<int> *res);
  std::shared_ptr<paddle_infer::Predictor> predictor_;

 private:
  std::string model_file_;
  std::string params_file_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
