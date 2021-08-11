/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <cassert>
#include <vector>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

// histogram related params
struct HistogramEstimatorParams {
  HistogramEstimatorParams() { Init(); }
  void Init();

  int nr_bins_in_histogram = 0;
  float data_sp = 0.0f;  // start point
  float data_ep = 0.0f;  // end point => [data_sp, data_ep)
  float step_bin = 0.0f;

  std::vector<uint32_t> smooth_kernel = {};
  int smooth_kernel_width = 0;  // only consider odd number
  int smooth_kernel_radius = 0;

  float hat_min_allowed = 0.0f;
  float hat_std_allowed = 0.0f;

  uint32_t histogram_mass_limit = 0;

  float decay_factor = 0.0f;

  void operator=(const HistogramEstimatorParams &params) {
    nr_bins_in_histogram = params.nr_bins_in_histogram;

    data_sp = params.data_sp;
    data_ep = params.data_ep;
    assert(nr_bins_in_histogram > 0 && data_ep > data_sp);

    step_bin = (data_ep - data_sp) / static_cast<float>(nr_bins_in_histogram);

    int w = static_cast<int>(params.smooth_kernel.size());
    assert(w >= 1);
    smooth_kernel_width = w;
    smooth_kernel.resize(smooth_kernel_width);
    memcpy(smooth_kernel.data(), params.smooth_kernel.data(),
           sizeof(uint32_t) * smooth_kernel_width);
    smooth_kernel_radius = smooth_kernel_width >> 1;

    hat_min_allowed = params.hat_min_allowed;
    hat_std_allowed = params.hat_std_allowed;

    histogram_mass_limit = params.histogram_mass_limit;
    assert(histogram_mass_limit > 0);

    decay_factor = params.decay_factor;
    assert(decay_factor > 0.0f);
  }
};

// histogram and its process
class HistogramEstimator {
 public:
  static const int kMaxNrBins = 1000;

  HistogramEstimator() {
    hist_.resize(kMaxNrBins);
    std::fill(hist_.begin(), hist_.end(), 0);

    hist_buffer_.resize(kMaxNrBins);
    std::fill(hist_buffer_.begin(), hist_buffer_.end(), 0);

    hist_hat_.resize(kMaxNrBins);
    std::fill(hist_hat_.begin(), hist_hat_.end(), 0.0f);

    Init();
  }

  ~HistogramEstimator() {}

  void Init(const HistogramEstimatorParams *params = nullptr);

  bool Push(float val) {
    if (val < params_.data_sp || val >= params_.data_ep) {
      AERROR << "Input data to histogram: out-of-range";
      AERROR << params_.data_sp;
      AERROR << params_.data_ep;
      AERROR << val;
      return false;
    }
    int index = GetIndex(val);
    if (index < 0 || index >= params_.nr_bins_in_histogram) {
      return false;
    }
    ++hist_[index];
    val_cur_ = val;
    return true;
  }

  void Clear() {
    std::fill(hist_.begin(), hist_.end(), 0);
    val_cur_ = val_estimation_ = 0.0f;
  }

  const std::vector<uint32_t> &get_hist() const { return hist_; }

  float get_val_cur() const { return val_cur_; }

  float get_val_estimation() const { return val_estimation_; }

  uint32_t get_bin_value(int i) const {
    assert(i >= 0 && i < params_.nr_bins_in_histogram);
    return hist_[i];
  }

  // main function:
  // smooth -> shape analysis -> update value -> decay
  bool Process();

 private:
  int GetIndex(float val) const {
    return static_cast<int>((val - params_.data_sp) * step_bin_reversed_);
  }

  float GetValFromIndex(int index) const {
    return (params_.data_sp +
            (static_cast<float>(index) + 0.5f) * params_.step_bin);
  }

  void Smooth(const uint32_t *hist_input, int nr_bins, uint32_t *hist_output);

  bool IsGoodShape(const uint32_t *hist, int nr_bins, int max_index);

  void GetPeakIndexAndMass(const uint32_t *hist, int nr_bins, int *index,
                           uint32_t *mass);

  void Decay(uint32_t *hist, int nr_bins) {
    float df = params_.decay_factor;
    for (int i = 0; i < nr_bins; ++i) {
      hist[i] = static_cast<uint32_t>(static_cast<float>(hist[i]) * df);
    }
  }

  void GenerateHat(float *hist_hat, int nr_bins);

  // bool SaveHist(const std::string &filename,
  //               const uint32_t *hist, int nr_bins);

 private:
  std::vector<uint32_t> hist_ = {};
  std::vector<uint32_t> hist_buffer_ = {};

  std::vector<float> hist_hat_ = {};

  HistogramEstimatorParams params_;

  float step_bin_reversed_ = 0.0f;
  float val_cur_ = 0.0f;
  float val_estimation_ = 0.0f;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
