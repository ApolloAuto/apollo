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

#include <algorithm>
#include <vector>

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

class SLICEPlugin : public nvinfer1::IPlugin {
 public:
  SLICEPlugin(const SliceParameter &param, const nvinfer1::Dims &in_dims) {
    CHECK_GT(param.slice_point_size(), 0);
    for (int i = 0; i < param.slice_point_size(); ++i) {
      slice_point_.push_back(param.slice_point(i));
    }
    axis_ = std::max(param.axis() - 1, 0);
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; ++i) {
      input_dims_.d[i] = in_dims.d[i];
      input_dims_.type[i] = in_dims.type[i];
    }

    for (size_t i = 0; i < slice_point_.size(); ++i) {
      if (i == 0) {
        out_slice_dims_.push_back(slice_point_[i]);
      } else {
        out_slice_dims_.push_back(slice_point_[i] - slice_point_[i - 1]);
      }
    }
    out_slice_dims_.push_back(input_dims_.d[axis_] -
                              slice_point_[slice_point_.size() - 1]);
  }
  SLICEPlugin() {}
  ~SLICEPlugin() {}
  int initialize() override { return 0; }
  void terminate() override {}
  int getNbOutputs() const override {
    return static_cast<int>(slice_point_.size()) + 1;
  }
  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) override {
    nvinfer1::Dims out_dims = inputs[0];
    out_dims.d[axis_] = out_slice_dims_[index];
    return out_dims;
  }

  void configure(const nvinfer1::Dims *inputDims, int nbInputs,
                 const nvinfer1::Dims *outputDims, int nbOutputs,
                 int maxBatchSize) override {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int maxBatchSize) const override { return 0; }

  int enqueue(int batchSize, const void *const *inputs, void **outputs,
              void *workspace, cudaStream_t stream) override;

  size_t getSerializationSize() override { return 0; }

  void serialize(void *buffer) override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

 private:
  std::vector<int> slice_point_;
  std::vector<int> out_slice_dims_;
  int axis_;
  nvinfer1::Dims input_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
