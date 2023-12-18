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

#include "modules/perception/common/inference/utils/util.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {
// todo(huqilin): the code is not used and the function is redefined, it needs to be fixed.
// __global__ void resize_linear_kernel(const unsigned char *src, float *dst,
//                                      int channel, int height, int width,
//                                      int stepwidth, int dst_height,
//                                      int dst_width, float fx, float fy) {
//   const int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
//   const int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
//   if (dst_x < dst_width && dst_y < dst_height) {
//     float src_x = (dst_x + 0.5) * fx - 0.5;
//     float src_y = (dst_y + 0.5) * fy - 0.5;
//     const int x1 = __float2int_rd(src_x);
//     const int y1 = __float2int_rd(src_y);
//     const int x1_read = max(x1, 0);
//     const int y1_read = max(y1, 0);
//     const int x2 = x1 + 1;
//     const int y2 = y1 + 1;
//     const int x2_read = min(x2, width - 1);
//     const int y2_read = min(y2, height - 1);
//     int src_reg = 0;
//     for (int c = 0; c < channel; c++) {
//       float out = 0;

//       int idx11 = (y1_read * stepwidth + x1_read) * channel;
//       src_reg = src[idx11 + c];
//       out = out + (x2 - src_x) * (y2 - src_y) * src_reg;
//       int idx12 = (y1_read * stepwidth + x2_read) * channel;
//       src_reg = src[idx12 + c];
//       out = out + src_reg * (src_x - x1) * (y2 - src_y);

//       int idx21 = (y2_read * stepwidth + x1_read) * channel;
//       src_reg = src[idx21 + c];
//       out = out + src_reg * (x2 - src_x) * (src_y - y1);

//       int idx22 = (y2_read * stepwidth + x2_read) * channel;
//       src_reg = src[idx22 + c];
//       out = out + src_reg * (src_x - x1) * (src_y - y1);
//       if (out < 0) {
//         out = 0;
//       }
//       if (out > 255) {
//         out = 255;
//       }
//       int dst_idx = (dst_y * dst_width + dst_x) * channel + c;
//       dst[dst_idx] = out;
//     }
//   }
// }
// int util_divup(int a, int b) {
//   if (a % b) {
//     return a / b + 1;
//   } else {
//     return a / b;
//   }
// }

// bool resize(int origin_channel, int origin_height, int origin_width,
//             int stepwidth,
//             std::shared_ptr<apollo::perception::base::Blob<float>> dst,
//             std::shared_ptr<apollo::perception::base::SyncedMemory> src_gpu,
//             int start_axis) {
//   int width = dst->shape(2);
//   int height = dst->shape(1);
//   int channel = dst->shape(3);

//   if (origin_channel != dst->shape(3)) {
//     AERROR << "channel should be the same after resize.";
//     return false;
//   }
//   if (src_gpu == nullptr) {
//     AERROR << "src_gpu should be allocated the same as the origin image.";
//     return false;
//   }
//   float fx = static_cast<float>(origin_width) / static_cast<float>(width);
//   float fy = static_cast<float>(origin_height) / static_cast<float>(height);
//   const dim3 block(32, 8);

//   const dim3 grid(util_divup(width, block.x), util_divup(height, block.y));

//   resize_linear_kernel<<<grid, block>>>(
//       (const unsigned char *)src_gpu->gpu_data(), dst->mutable_gpu_data(),
//       origin_channel, origin_height, origin_width, stepwidth, height, width, fx,
//       fy);
//   return true;
// }
}  // namespace inference
}  // namespace perception
}  // namespace apollo
