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
#include <limits>
#include <vector>

#include "modules/perception/inference/tensorrt/plugins/argmax_plugin.h"
namespace apollo {
namespace perception {
namespace inference {
__global__ void cmp(const int nthreads, const float *in_data,
                    const int channels, const int height, const int width,
                    const bool out_max_val, float *out_data,
                    const float float_min) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < nthreads) {
    int w = idx % width;
    idx = idx / width;
    int h = idx % height;
    idx = idx / height;
    int c = idx % channels;
    int n = idx / channels;
    if (c != 0) {
      return;
    }
    int c_max = 0;
    float v_max = float_min;
    for (int c = 0; c < channels; ++c) {
      int in_idx = ((n * channels + c) * height + h) * width + w;
      if (v_max < in_data[in_idx]) {
        v_max = in_data[in_idx];
        c_max = c;
      }
    }
    int out_idx_idx = ((n * channels + 0) * height + h) * width + w;
    out_data[out_idx_idx] = c_max;
    if (out_max_val) {
      int out_val_idx = ((n * channels + 1) * height + h) * width + w;
      out_data[out_val_idx] = v_max;
    }
  }
}
int ArgMax1Plugin::enqueue(int batchSize, const void *const *inputs,
                           void **outputs, void *workspace,
                           cudaStream_t stream) {
  const int thread_size = 512;
  int block_size =
      (input_dims_.d[0] * input_dims_.d[1] * input_dims_.d[2] * batchSize +
       thread_size - 1) /
      thread_size;
  cmp<<<block_size, thread_size>>>(
      input_dims_.d[0] * input_dims_.d[1] * input_dims_.d[2] * batchSize,
      (const float *)inputs[0], input_dims_.d[0], input_dims_.d[1],
      input_dims_.d[2], out_max_val_, reinterpret_cast<float *>(outputs[0]),
      float_min_);
  return 0;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
