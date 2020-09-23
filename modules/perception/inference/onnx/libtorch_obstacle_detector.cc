  
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

#include "modules/perception/inference/onnx/libtorch_obstacle_detector.h"

#include <utility>
#include <vector>
#include <omp.h>

#include "cyber/common/log.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace inference {

LibtorchObstacleDetection::LibtorchObstacleDetection() : device_(torch::kCPU) {
  LoadModel();
}

bool LibtorchObstacleDetection::Evaluate(const std::vector<std::vector<double>>& imageFrame) {
  // Sanity checks.
  omp_set_num_threads(1);
  if (imageFrame.size() == 0) {
    AERROR << "Got no channel in image frame!";
    return false;
  }
  if (imageFrame[0].size() == 0) {
    AERROR << "Got no image frame in channel 0!";
    return false;
  }
  if (imageFrame[0].size() != 72000) {
    AERROR << "imageFrame[0].size() = " << imageFrame[0].size() << ", skiping!";
    return false;
  }
  // image imput size is 1920 * 1080 = 2073600
  torch::Tensor image_tensor = torch::empty(32 * 3 * 3 * 3);
  float* data = image_tensor.data_ptr<float>();

  for (const auto& channel : imageFrame) {
    for (const auto& i : channel) {
      *data++ = static_cast<float>(i) / 32767.0;
    }
  }

  torch::Tensor torch_input = torch::from_blob(image_tensor.data_ptr<float>(),
                                               {32, 3, 3, 3, 3});
  std::vector<torch::jit::IValue> torch_inputs;
  torch_inputs.push_back(torch_input.to(device_));

  auto start_time = std::chrono::system_clock::now();
  at::Tensor torch_output_tensor = torch_model_.forward(torch_inputs).toTensor()
                                               .to(torch::kCPU);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO << "LibtorchObstacleDetection used time: " << diff.count() * 1000 << " ms.";
  auto torch_output = torch_output_tensor.accessor<float, 2>();

  // majority vote with 4 channels
  float neg_score = torch_output[0][0] + torch_output[1][0] +
                    torch_output[2][0] + torch_output[3][0];
  float pos_score = torch_output[0][1] + torch_output[1][1] +
                    torch_output[2][1] + torch_output[3][1];
  ADEBUG << "neg_score = " << neg_score << ", pos_score = " << pos_score;
  if (neg_score < pos_score) {
    return true;
  } else {
    return false;
  }
  return true;
}

void LibtorchObstacleDetection::LoadModel() {
  if (torch::cuda::is_available()) {
    AINFO << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
  }
  torch::set_num_threads(1);
  torch_model_ = torch::jit::load(FLAGS_torch_detector_model, device_);
}

} // inference
} // perception
} // apollo