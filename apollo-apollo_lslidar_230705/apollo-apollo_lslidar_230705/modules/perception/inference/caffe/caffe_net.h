/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "caffe/caffe.hpp"

#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

typedef std::shared_ptr<apollo::perception::base::Blob<float>> BlobPtr;

class CaffeNet : public Inference {
 public:
  CaffeNet(const std::string &net_file, const std::string &model_file,
           const std::vector<std::string> &outputs);

  CaffeNet(const std::string &net_file, const std::string &model_file,
           const std::vector<std::string> &outputs,
           const std::vector<std::string> &inputs);

  virtual ~CaffeNet() {}

  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;

  void Infer() override;
  BlobPtr get_blob(const std::string &name) override;

 protected:
  bool reshape();
  bool shape(const std::string &name, std::vector<int> *res);
  std::shared_ptr<caffe::Net<float>> net_ = nullptr;

 private:
  std::string net_file_;
  std::string model_file_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
