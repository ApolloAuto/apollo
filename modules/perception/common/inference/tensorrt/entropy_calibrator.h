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

#pragma once

#include <string>
#include <vector>

#include "NvInfer.h"
#include "NvInferPlugin.h"

#include "modules/perception/common/inference/tensorrt/batch_stream.h"
#include "modules/perception/common/inference/tensorrt/rt_utils.h"

namespace nvinfer1 {
class Int8EntropyCalibrator : public IInt8EntropyCalibrator {
 public:
  Int8EntropyCalibrator(
      const apollo::perception::inference::BatchStream &stream, int first_batch,
      bool read_cache, std::string network);

  virtual ~Int8EntropyCalibrator();
  int getBatchSize() const noexcept override { return stream_.getBatchSize(); }

  bool getBatch(void *bindings[],
    const char *names[], int nbBindings) noexcept override;

  const void *readCalibrationCache(size_t &length) noexcept override;

  void writeCalibrationCache(const void *cache,
    size_t length) noexcept override;

 private:
  apollo::perception::inference::BatchStream stream_;
  bool read_cache_ = true;
  std::string network_ = "yolo";
  size_t input_count_;
  void *device_input_ = nullptr;
  std::vector<char> calibration_cache_;
};

}  //  namespace nvinfer1
