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

#include <vector>
#include "modules/perception/inference/tensorrt/plugins/slice_plugin.h"

namespace apollo {
namespace perception {
namespace inference {

typedef int8_t int8;

template<typename Dtype>
__global__ void Slice(const int nthreads, const Dtype *in_data,
                      const int num_slices, const int slice_size,
                      const int bottom_slice_axis, const int top_slice_axis,
                      const int offset_slice_axis, Dtype *out_data) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < nthreads) {
    const int total_slice_size = slice_size * top_slice_axis;
    const int slice_num = index / total_slice_size;
    const int slice_index = index % total_slice_size;
    const int bottom_index = slice_index
        + (slice_num * bottom_slice_axis + offset_slice_axis) * slice_size;
    out_data[index] = in_data[bottom_index];
  }
}

int SLICEPlugin::enqueue(int batchSize,
                         const void *const *inputs,
                         void **outputs,
                         void *workspace,
                         cudaStream_t stream) {
  int slice_size = 1;
  for (size_t index = axis_ + 1; index < input_dims_.nbDims; index++) {
    slice_size *= input_dims_.d[index];
  }
  int num_slices = batchSize;
  for (size_t index = 0; index < axis_; index++) {
    num_slices *= input_dims_.d[index];
  }
  int offset_slice_axis = 0;

  for (int i = 0; i < out_slice_dims_.size(); i++) {
    const int top_slice_axis = out_slice_dims_[i];
    const int top_slice_size = top_slice_axis * slice_size;
    const int nthreads = top_slice_size * num_slices;
    const int block_num = (nthreads + 511) / 512;

    Slice  // NOLINT_NEXT_LINE(whitespace/operators)
        << < block_num, 512, 0, stream >> > (
        nthreads, (const float *) (inputs[0]), num_slices, slice_size,
            input_dims_.d[axis_], top_slice_axis,
            offset_slice_axis,  reinterpret_cast<float *>(outputs[i]));
    offset_slice_axis += top_slice_axis;
  }
  return 1;
}
}  // namespace inference
}  // namespace perception
}  // namespace apollo
