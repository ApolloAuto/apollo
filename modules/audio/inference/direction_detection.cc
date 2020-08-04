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

#include "modules/audio/inference/direction_detection.h"

namespace apollo {
namespace audio {

using torch::indexing::None;
using torch::indexing::Slice;

int get_direction(std::vector<std::vector<float>>&& channels_vec,
                  const int sample_rate, const int mic_distance) {
  std::vector<torch::Tensor> channels_ts;
  auto options = torch::TensorOptions().dtype(torch::kFloat32);
  int size = static_cast<int>(channels_vec[0].size());
  for (auto& signal : channels_vec) {
    channels_ts.push_back(torch::from_blob(signal.data(), {size}, options));
  }

  float tau0, tau1;
  int theta0, theta1;
  const float max_tau = mic_distance / SOUND_SPEED;
  tau0 = gcc_phat(channels_ts[0], channels_ts[2], sample_rate, max_tau, 1);
  theta0 = asin(tau0 / max_tau) * 180 / M_PI;
  tau1 = gcc_phat(channels_ts[1], channels_ts[3], sample_rate, max_tau, 1);
  theta1 = asin(tau1 / max_tau) * 180 / M_PI;

  int best_guess = 0;
  if (fabs(theta0) < fabs(theta1)) {
    best_guess = theta1 > 0 ? (theta0 + 360) % 360 : (180 - theta0);
  } else {
    best_guess = theta0 < 0 ? (theta1 + 360) % 360 : (180 - theta1);
    best_guess = (best_guess + 90 + 180) % 360;
  }
  best_guess = (-best_guess + 120) % 360;

  return best_guess;
}

/*
 * This function computes the offset between the signal sig and the reference
 * signal refsig using the Generalized Cross Correlation - Phase Transform
 * (GCC-PHAT)method.
 */
float gcc_phat(const torch::Tensor& sig, const torch::Tensor& refsig, int fs,
               double max_tau, int interp) {
  const int n_sig = sig.size(0), n_refsig = refsig.size(0),
            n = n_sig + n_refsig;
  torch::Tensor psig = at::constant_pad_nd(sig, {0, n_refsig}, 0);
  torch::Tensor prefsig = at::constant_pad_nd(refsig, {0, n_sig}, 0);
  psig = at::rfft(psig, 1, false, true);
  prefsig = at::rfft(prefsig, 1, false, true);
  torch::Tensor r = psig * at::conj(prefsig);
  torch::Tensor cc = at::irfft(r / at::abs(r), 1, false, true, {interp * n});
  int max_shift = static_cast<int>(interp * n / 2);
  if (max_tau != 0)
    max_shift = std::min(static_cast<int>(interp * fs * max_tau), max_shift);

  auto begin = cc.index({Slice(0, cc.size(0) - max_shift, None)});
  auto end = cc.index({Slice(0, None, max_shift + 1)});
  cc = at::cat({begin, end});
  auto ttt = at::argmax(at::abs(cc), 0);
  // find max cross correlation index
  const int shift = at::argmax(at::abs(cc), 0).item<int>() - max_shift;
  const float tau = shift / static_cast<float>(interp * fs);

  return tau;
}

}  // namespace audio
}  // namespace apollo
