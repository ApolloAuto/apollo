/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "paddle/paddle_inference_api.h"

#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

typedef std::shared_ptr<apollo::perception::base::Blob<float>> BlobPtr;

constexpr uint64_t MemoryPoolInitSizeMb = 100;

class PaddleNet : public Inference {
 public:
  PaddleNet(const std::string &model_file, const std::string &param_file,
            const std::vector<std::string> &outputs);

  PaddleNet(const std::string &model_file, const std::string &param_file,
            const std::vector<std::string> &outputs,
            const std::vector<std::string> &inputs);

  virtual ~PaddleNet() {}

  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;

  void Infer() override;
  BlobPtr get_blob(const std::string &name) override;

 protected:
  bool reshape();
  bool shape(const std::string &name, std::vector<int> *res);
  std::shared_ptr<paddle::PaddlePredictor> predictor_ = nullptr;

 private:
  std::string model_file_;
  std::string param_file_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;

  std::unordered_map<std::string, std::string> name_map_ = {
      // object detection
      {"data", "input"},
      {"obj_pred", "save_infer_model/scale_0"},
      {"cls_pred", "save_infer_model/scale_1"},
      {"ori_pred", "save_infer_model/scale_2"},
      {"dim_pred", "save_infer_model/scale_3"},
      {"brvis_pred", "save_infer_model/scale_4"},
      {"ltvis_pred", "save_infer_model/scale_5"},
      {"rtvis_pred", "save_infer_model/scale_6"},
      {"brswt_pred", "save_infer_model/scale_7"},
      {"ltswt_pred", "save_infer_model/scale_8"},
      {"rtswt_pred", "save_infer_model/scale_9"},
      {"loc_pred", "save_infer_model/scale_13"},
      {"conv3_3", "save_infer_model/scale_14"},
      // lane line
      {"softmax", "save_infer_model/scale_0"},
      // lidar cnn_seg
      {"confidence_score", "save_infer_model/scale_0"},
      {"class_score", "save_infer_model/scale_1"},
      {"category_score", "save_infer_model/scale_2"},
      {"instance_pt", "save_infer_model/scale_3"},
      {"heading_pt", "save_infer_model/scale_4"},
      {"height_pt", "save_infer_model/scale_5"}};
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
