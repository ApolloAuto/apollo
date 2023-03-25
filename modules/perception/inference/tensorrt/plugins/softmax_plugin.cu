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

#include "modules/perception/inference/tensorrt/plugins/softmax_plugin.h"

namespace apollo {
namespace perception {
namespace inference {

int SoftmaxPlugin::enqueue(int batch_size, const void *const *inputs,
                           void **outputs, void *workspace,
                           cudaStream_t stream) {
  const float *in_data = reinterpret_cast<const float *>(inputs[0]);
  float *out_data = reinterpret_cast<float *>(outputs[0]);
  int w = 1;
  int h = inner_num_;
  int c = input_dims_.d[axis_];
  int n = batch_size * outer_num_;
  int w_stride = 1;
  int h_stride = w * w_stride;
  int c_stride = h * h_stride;
  int n_stride = c * c_stride;

  cudnnSetTensor4dDescriptorEx(input_desc_, CUDNN_DATA_FLOAT, n, c, h, w,
                               n_stride, c_stride, h_stride, w_stride);
  cudnnSetTensor4dDescriptorEx(output_desc_, CUDNN_DATA_FLOAT, n, c, h, w,
                               n_stride, c_stride, h_stride, w_stride);

  float a = 1.0;
  float b = 0.0;
  cudnnSetStream(cudnn_, stream);
#if GPU_PLATFORM == NVIDIA
  cudnnSoftmaxForward(cudnn_, CUDNN_SOFTMAX_ACCURATE,
                      CUDNN_SOFTMAX_MODE_CHANNEL, (const void *)(&a),
                      input_desc_, in_data, (const void *)(&b), output_desc_,
                      out_data);
#elif GPU_PLATFORM == AMD
  miopenSoftmaxForward_V2(cudnn_, (const void *)(&a), input_desc_, in_data,
                          (const void *)(&b), output_desc_, out_data,
                          CUDNN_SOFTMAX_ACCURATE, CUDNN_SOFTMAX_MODE_CHANNEL);
#endif

  return 1;
}
}  // namespace inference
}  // namespace perception
}  // namespace apollo
