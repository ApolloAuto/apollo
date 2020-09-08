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

#include <torch/torch.h>
#include <torch/script.h>

#include <opencv2/opencv.hpp>
#include <iostream>

int main(){
  torch::DeviceType device_type; //设置Device类型
  device_type = torch::kCUDA;  //torch::kCUDA  and torch::kCPU
  torch::Device device(device_type, 0);

  torch::Tensor tensor = torch::rand({2, 3}, device);
  std::cout << tensor << std::endl;

  auto image = cv::imread(
                          "", 
                          cv::ImreadModes::IMREAD_COLOR);
  cv::Mat image_transfomed;
  cv::resize(image, image_transfomed, cv::Size(32, 96));
  cv::cvtColor(image_transfomed, image, cv::COLOR_BGR2RGB);

  torch::Tensor tensor_image = torch::from_blob(image.data, 
                        {image.rows, image.cols, 3}, torch::kByte).cuda();
  tensor_image = tensor_image.permute({2, 0, 1});
  tensor_image = tensor_image.toType(torch::kFloat32);
  // std::cout << tensor_image[0][0] << std::endl;
  tensor_image = tensor_image.div(255);
  tensor_image[0] = tensor_image[0].sub_(0.485).div_(0.229);
  tensor_image[1] = tensor_image[1].sub_(0.456).div_(0.224);
  tensor_image[2] = tensor_image[2].sub_(0.406).div_(0.225);

  std::cout << tensor_image.sizes() << std::endl;

  torch::jit::script::Module module = torch::jit::load(
                      "/apollo/recognition_infer/vertical_net_libtorch.pth",
                      device);
  // module.eval();
  std::cout << "Load model" << std::endl;

  tensor_image = tensor_image.unsqueeze(0);
  torch::Tensor output = module.forward({tensor_image}).toTensor();

  std::cout << output.sizes() << std::endl;
  std::cout << output << std::endl;
  return 0;

}