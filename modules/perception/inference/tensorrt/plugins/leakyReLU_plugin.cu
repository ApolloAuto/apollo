/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/inference/tensorrt/plugins/leakyReLU_plugin.h"

#include <vector>

namespace apollo {
namespace perception {
namespace inference {

template<typename Dtype>
__global__ void ReLU(const int nthreads, const Dtype *in_data,
                     const float negative_slope, Dtype *out_data) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < nthreads) {
    out_data[index] = in_data[index];
    if (out_data[index] < 0.0) {
      out_data[index] *= negative_slope;
    }
  }
}

int ReLUPlugin::enqueue(int batchSize,
                         const void *const *inputs,
                         void **outputs,
                         void *workspace,
                         cudaStream_t stream) {
  const int thread_size = 512;
  const int block_size =
      (input_dims_.d[0] * input_dims_.d[1] * input_dims_.d[2] * batchSize
          + thread_size - 1) / thread_size;
  const int nthreads = input_dims_.d[0] * input_dims_.d[1]
                       * input_dims_.d[2] * batchSize;

  ReLU<< < block_size, thread_size, 0, stream >> > (
          nthreads, (const float *) (inputs[0]),
          negative_slope_,  reinterpret_cast<float *>(outputs[0]));
  return 1;
}
}  // namespace inference
}  // namespace perception
}  // namespace apollo
