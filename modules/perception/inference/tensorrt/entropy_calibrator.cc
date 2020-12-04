/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/inference/tensorrt/entropy_calibrator.h"

#include <fstream>

#include <cuda_runtime_api.h>

namespace nvinfer1 {
DimsHW ICaffePoolOutputDimensionsFormula::compute(DimsHW input_dims,
                                                  DimsHW kernel_size,
                                                  DimsHW stride, DimsHW padding,
                                                  DimsHW dilation,
                                                  const char *layerName) const {
  const int kernel_extent_h = dilation.d[0] * (kernel_size.d[0] - 1) + 1;
  const int kernel_extent_w = dilation.d[1] * (kernel_size.d[1] - 1) + 1;
  auto &&h_temp = (input_dims.d[0] + 2 * padding.d[0] - kernel_extent_h) * 1.0 /
                  stride.d[0];
  auto &&w_temp = (input_dims.d[1] + 2 * padding.d[1] - kernel_extent_w) * 1.0 /
                  stride.d[1];

  std::string str_name(layerName);
  if (str_name.find("as_conv") == std::string::npos) {
    return DimsHW(static_cast<int>(ceil(h_temp)) + 1,
                  static_cast<int>(ceil(w_temp)) + 1);
  } else {
    return DimsHW(static_cast<int>(h_temp) + 1, static_cast<int>(w_temp) + 1);
  }
}

Int8EntropyCalibrator::Int8EntropyCalibrator(
    const apollo::perception::inference::BatchStream &stream, int first_batch,
    bool read_cache, std::string network)
    : stream_(stream), read_cache_(read_cache), network_(network) {
  DimsNCHW dims = stream_.getDims();
  input_count_ = stream_.getBatchSize() * dims.c() * dims.h() * dims.w();
  cudaMalloc(&device_input_, input_count_ * sizeof(float));
  stream_.reset(first_batch);
}

Int8EntropyCalibrator::~Int8EntropyCalibrator() {
  if (device_input_) {
    (cudaFree(device_input_));
  }
}

bool Int8EntropyCalibrator::getBatch(void *bindings[], const char *names[],
                                     int nbBindings) {
  if (!stream_.next()) {
    return false;
  }

  (cudaMemcpy(device_input_, stream_.getBatch(), input_count_ * sizeof(float),
              cudaMemcpyHostToDevice));
  bindings[0] = device_input_;
  return true;
}

const void *Int8EntropyCalibrator::readCalibrationCache(size_t &length) {
  calibration_cache_.clear();
  std::ifstream input(
      apollo::perception::inference::locateFile(network_, "CalibrationTable"),
      std::ios::binary);
  input >> std::noskipws;
  if (read_cache_ && input.good()) {
    std::copy(std::istream_iterator<char>(input), std::istream_iterator<char>(),
              std::back_inserter(calibration_cache_));
  }

  length = calibration_cache_.size();
  return length ? &calibration_cache_[0] : nullptr;
}

void Int8EntropyCalibrator::writeCalibrationCache(const void *cache,
                                                  size_t length) {
  std::ofstream output(
      apollo::perception::inference::locateFile(network_, "CalibrationTable"),
      std::ios::binary);
  output.write(reinterpret_cast<const char *>(cache), length);
}

}  //  namespace nvinfer1
