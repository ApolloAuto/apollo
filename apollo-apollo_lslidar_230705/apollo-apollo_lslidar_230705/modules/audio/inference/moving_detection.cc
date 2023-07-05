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

MovingResult MovingDetection::Detect(
    const std::vector<std::vector<double>>& signals) {
  int approaching_count = 0;
  int departing_count = 0;
  for (std::size_t i = 0; i < signals.size(); ++i) {
    while (signal_stats_.size() <= i) {
      signal_stats_.push_back({});
    }
    MovingResult result = DetectSingleChannel(i, signals[i]);
    if (result == APPROACHING) {
      ++approaching_count;
    } else if (result == DEPARTING) {
      ++departing_count;
    }
  }
  if (approaching_count > departing_count) {
    return APPROACHING;
  }
  if (approaching_count < departing_count) {
    return DEPARTING;
  }
  return UNKNOWN;
}

MovingResult MovingDetection::DetectSingleChannel(
    const std::size_t channel_index, const std::vector<double>& signals) {
  static constexpr int kStartFrequency = 3;
  static constexpr int kFrameNumStored = 10;
  std::vector<std::complex<double>> fft_results = fft1d(signals);
  SignalStat signal_stat = GetSignalStat(fft_results, kStartFrequency);
  signal_stats_[channel_index].push_back(signal_stat);
  while (static_cast<int>(signal_stats_[channel_index].size()) >
         kFrameNumStored) {
    signal_stats_[channel_index].pop_front();
  }
  // TODO(kechxu) refactor the following initial version
  MovingResult power_result = AnalyzePower(signal_stats_[channel_index]);
  if (power_result != UNKNOWN) {
    return power_result;
  }
  MovingResult top_frequency_result =
      AnalyzeTopFrequence(signal_stats_[channel_index]);
  return top_frequency_result;
}

MovingResult MovingDetection::AnalyzePower(
    const std::deque<SignalStat>& signal_stats) {
  int n = static_cast<int>(signal_stats.size());
  if (n < 3) {
    return UNKNOWN;
  }
  double first = signal_stats[n - 3].power();
  double second = signal_stats[n - 2].power();
  double third = signal_stats[n - 1].power();
  if (first < second && second < third) {
    return APPROACHING;
  }
  if (first > second && second > third) {
    return DEPARTING;
  }
  return UNKNOWN;
}

MovingResult MovingDetection::AnalyzeTopFrequence(
    const std::deque<SignalStat>& signal_stats) {
  int n = static_cast<int>(signal_stats.size());
  if (n < 3) {
    return UNKNOWN;
  }
  int first = signal_stats[n - 3].top_frequency();
  int second = signal_stats[n - 2].top_frequency();
  int third = signal_stats[n - 1].top_frequency();
  if (first < second && second < third) {
    return APPROACHING;
  }
  if (first > second && second > third) {
    return DEPARTING;
  }
  return UNKNOWN;
}

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

MovingDetection::SignalStat MovingDetection::GetSignalStat(
    const std::vector<std::complex<double>>& fft_results,
    const int start_frequency) {
  double total_power = 0.0;
  int top_frequency = -1;
  double max_power = -1.0;
  for (int i = start_frequency; i < static_cast<int>(fft_results.size()); ++i) {
    double power = std::abs(fft_results[i]);
    total_power += power;
    if (power > max_power) {
      max_power = power;
      top_frequency = i;
    }
  }
  return {total_power, top_frequency};
}

}  // namespace audio
}  // namespace apollo
