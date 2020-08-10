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

#include "modules/audio/inference/moving_detection.h"

#include <fftw3.h>

namespace apollo {
namespace audio {

std::vector<std::complex<double>> MovingDetection::fft1d(
    const std::vector<double>& signal) {
  int n = static_cast<int>(signal.size());
  fftw_complex in[n];  // NOLINT
  fftw_complex out[n];  // NOLINT
  for (int i = 0; i < n; ++i) {
    in[i][0] = signal[i];
    in[i][1] = 0.0;
  }

  fftw_plan p = fftw_plan_dft_1d(n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
  fftw_execute(p);

  std::vector<std::complex<double>> output;
  output.reserve(n);
  for (int i = 0; i < n; ++i) {
    output.emplace_back(out[i][0], out[i][1]);
  }
  return output;
}

}  // namespace audio
}  // namespace apollo
