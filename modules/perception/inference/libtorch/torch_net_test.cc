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

#include <map>
#include <string>
#include <vector>

#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "modules/perception/base/blob.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/utils/resize.h"

int main() {
  std::shared_ptr<apollo::perception::inference::Inference> rt_net_ = nullptr;
  std::string weight_file = "/apollo/modules/perception/production/data/"\
                    "perception/camera/models/traffic_light_recognition/"\
                    "vertical/vertical_net_libtorch.pth";
  std::vector<std::string> net_inputs_ = {"data_org"};
  std::vector<std::string> net_outputs_ = {"prob"};
  std::string imagePath = "/apollo/modules/perception/inference/libtorch/"\
        "test_data/0_green/DE_BBBR667_2015-04-17_11-05-32-836043_k0_2.jpg";

  rt_net_.reset(apollo::perception::inference::CreateInferenceByName(
                                  "TorchNet", weight_file, weight_file,
                                  net_outputs_, net_inputs_, "std"));
  std::cout << "create success" << std::endl;

  std::vector<int> shape = {1, 96, 32, 3};
  // std::vector<int> shape = {1, 3, 96, 32};
  std::map<std::string, std::vector<int>> input_reshape{
      {net_inputs_[0], shape}};
  std::cout << "input_reshape: " << input_reshape[net_inputs_[0]][0] << ", "
            << input_reshape[net_inputs_[0]][1] << ", "
            << input_reshape[net_inputs_[0]][2] << ", "
            << input_reshape[net_inputs_[0]][3] << std::endl;

  if (!rt_net_->Init(input_reshape)) {
    std::cout << "net init fail." << std::endl;
  }

  auto input_blob_recog = rt_net_->get_blob(net_inputs_[0]);
  auto output_blob_recog = rt_net_->get_blob(net_outputs_[0]);

  auto image_transfomed = cv::imread(imagePath, cv::ImreadModes::IMREAD_COLOR);
  cv::Mat image;
  cv::Mat image_resize;
  // cv::resize(image_transfomed, image_resize, cv::Size(32, 96));
  cv::cvtColor(image_transfomed, image, cv::COLOR_BGR2RGB);
  // input_blob_recog->data()->set_cpu_data(image.data);

  std::shared_ptr<apollo::perception::base::Image8U> image_;
  std::shared_ptr<apollo::perception::base::Blob<uint8_t>> blob;
  blob.reset(new apollo::perception::base::Blob<uint8_t>({image.rows,
                                                    image.cols, 3}));
  blob->data()->set_cpu_data(image.data);
  image_.reset(new apollo::perception::base::Image8U(image.rows, image.cols,
                                apollo::perception::base::Color::RGB, blob));
  apollo::perception::inference::ResizeGPU(*image_, input_blob_recog,
                            image.cols, 0, 123.675, 116.28, 103.53,
                            true, 1);

  rt_net_->Infer();
  float* output_data =  output_blob_recog->mutable_cpu_data();
  for (int i = 0; i < output_blob_recog->count(); i++) {
    std::cout << "output blob: " << output_data[i] << std::endl;
  }
}
