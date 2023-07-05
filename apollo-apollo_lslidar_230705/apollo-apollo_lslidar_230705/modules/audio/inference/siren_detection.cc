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

#include "modules/audio/inference/siren_detection.h"

#include <utility>
#include <vector>

#include <omp.h>

#include "cyber/common/log.h"
#include "modules/audio/common/audio_gflags.h"

namespace apollo {
namespace audio {

SirenDetection::SirenDetection() : device_(torch::kCPU) {
  LoadModel();
}

bool SirenDetection::Evaluate(const std::vector<std::vector<double>>& signals) {
  // Sanity checks.
  omp_set_num_threads(1);
  if (signals.size() == 0) {
    AERROR << "Got no channel in signals!";
    return false;
  }
  if (signals[0].size() == 0) {
    AERROR << "Got no signal in channel 0!";
    return false;
  }
  if (signals[0].size() != 72000) {
    AERROR << "signals[0].size() = " << signals[0].size() << ", skiping!";
    return false;
  }
  torch::Tensor audio_tensor = torch::empty(4 * 1 * 72000);
  float* data = audio_tensor.data_ptr<float>();

  for (const auto& channel : signals) {
    for (const auto& i : channel) {
      *data++ = static_cast<float>(i) / 32767.0;
    }
  }

  torch::Tensor torch_input = torch::from_blob(audio_tensor.data_ptr<float>(),
                                               {4, 1, 72000});
  std::vector<torch::jit::IValue> torch_inputs;
  torch_inputs.push_back(torch_input.to(device_));

  auto start_time = std::chrono::system_clock::now();
  at::Tensor torch_output_tensor = torch_model_.forward(torch_inputs).toTensor()
                                               .to(torch::kCPU);

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO << "SirenDetection used time: " << diff.count() * 1000 << " ms.";
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
}

void SirenDetection::LoadModel() {
  if (torch::cuda::is_available()) {
    AINFO << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
  }
  torch::set_num_threads(1);
  torch_model_ = torch::jit::load(FLAGS_torch_siren_detection_model, device_);
}

}  // namespace audio
}  // namespace apollo
