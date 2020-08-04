
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

#include <algorithm>
#include <cmath>
#include <vector>

#include "ATen/ATen.h"
#include "cyber/cyber.h"
#include "torch/torch.h"

namespace apollo {
namespace audio {

constexpr float SOUND_SPEED = 343.2;

float gcc_phat(const torch::Tensor& sig, const torch::Tensor& refsig, int fs,
               double max_tau, int interp);
int get_direction(std::vector<std::vector<float>>&& channels_vec,
                  const int sample_rate, const int mic_distance);

}  // namespace audio
}  // namespace apollo
