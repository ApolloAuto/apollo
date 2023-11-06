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

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/perception/common/inference/utils/cuda_util.h"
#include "modules/perception/common/inference/utils/resize.h"
#include "modules/perception/common/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace inference {
__global__ void resize_linear_kernel(const unsigned char *src, float *dst,
                                     int channel, int height, int width,
                                     int stepwidth, int dst_height,
                                     int dst_width, float fx, float fy) {
  const int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
  const int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
  if (dst_x < dst_width && dst_y < dst_height) {
    float src_x = (dst_x + 0.5) * fx - 0.5;
    float src_y = (dst_y + 0.5) * fy - 0.5;
    const int x1 = __float2int_rd(src_x);
    const int y1 = __float2int_rd(src_y);
    const int x1_read = max(x1, 0);
    const int y1_read = max(y1, 0);
    const int x2 = x1 + 1;
    const int y2 = y1 + 1;
    const int x2_read = min(x2, width - 1);
    const int y2_read = min(y2, height - 1);
    int src_reg = 0;
    for (int c = 0; c < channel; c++) {
      float out = 0;

      int idx11 = (y1_read * stepwidth + x1_read) * channel;
      src_reg = src[idx11 + c];
      out = out + (x2 - src_x) * (y2 - src_y) * src_reg;
      int idx12 = (y1_read * stepwidth + x2_read) * channel;
      src_reg = src[idx12 + c];
      out = out + src_reg * (src_x - x1) * (y2 - src_y);

      int idx21 = (y2_read * stepwidth + x1_read) * channel;
      src_reg = src[idx21 + c];
      out = out + src_reg * (x2 - src_x) * (src_y - y1);

      int idx22 = (y2_read * stepwidth + x2_read) * channel;
      src_reg = src[idx22 + c];
      out = out + src_reg * (src_x - x1) * (src_y - y1);
      if (out < 0) {
        out = 0;
      }
      if (out > 255) {
        out = 255;
      }
      int dst_idx = (dst_y * dst_width + dst_x) * channel + c;
      dst[dst_idx] = out;
    }
  }
}
int divup(int a, int b) {
  if (a % b) {
    return a / b + 1;
  } else {
    return a / b;
  }
}

template <typename T>
__global__ void resize_linear_kernel_mean(const unsigned char *src, float *dst,
                                          int channel, int height, int width,
                                          int stepwidth, int dst_height,
                                          int dst_width, float fx, float fy,
                                          T mean_b, T mean_g, T mean_r,
                                          bool channel_axis, float scale) {
  const int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
  const int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
  if (dst_x < dst_width && dst_y < dst_height) {
    float src_x = (dst_x + 0.5) * fx - 0.5;
    float src_y = (dst_y + 0.5) * fy - 0.5;
    const int x1 = __float2int_rd(src_x);
    const int y1 = __float2int_rd(src_y);
    const int x1_read = max(x1, 0);
    const int y1_read = max(y1, 0);
    const int x2 = x1 + 1;
    const int y2 = y1 + 1;
    const int x2_read = min(x2, width - 1);
    const int y2_read = min(y2, height - 1);
    // (h*width+w)*channel+c
    int src_reg = 0;
    for (int c = 0; c < channel; c++) {
      float out = 0;

      int idx11 = (y1_read * stepwidth + x1_read) * channel;
      src_reg = src[idx11 + c];
      out = out + (x2 - src_x) * (y2 - src_y) * src_reg;
      int idx12 = (y1_read * stepwidth + x2_read) * channel;
      src_reg = src[idx12 + c];
      out = out + src_reg * (src_x - x1) * (y2 - src_y);

      int idx21 = (y2_read * stepwidth + x1_read) * channel;
      src_reg = src[idx21 + c];
      out = out + src_reg * (x2 - src_x) * (src_y - y1);

      int idx22 = (y2_read * stepwidth + x2_read) * channel;
      src_reg = src[idx22 + c];
      out = out + src_reg * (src_x - x1) * (src_y - y1);
      if (out < 0) {
        out = 0;
      }
      if (out > 255) {
        out = 255;
      }
      int dst_idx;
      if (channel_axis) {
        dst_idx = (dst_y * dst_width + dst_x) * channel + c;
      } else {
        dst_idx = (c * dst_height + dst_y) * dst_width + dst_x;
      }
      // printf("%f %d %d %d %d %d %d %d\n",out,x1,y1,x2,y2,c,dst_y,dst_x);
      // dst[dst_idx] = (out - mean[c]) * scale;
      if (c == 0) {
        dst[dst_idx] = (out - mean_b) * scale;
      } else if (c == 1) {
        dst[dst_idx] = (out - mean_g) * scale;
      } else if (c == 2) {
        dst[dst_idx] = (out - mean_r) * scale;
      }
    }
  }
}

bool ResizeGPU(const base::Image8U &src,
               std::shared_ptr<apollo::perception::base::Blob<float>> dst,
               int stepwidth, int start_axis) {
  int width = dst->shape(2);
  int height = dst->shape(1);
  int channel = dst->shape(3);
  int origin_channel = src.channels();
  int origin_height = src.rows();
  int origin_width = src.cols();

  if (origin_channel != dst->shape(3)) {
    AERROR << "channel should be the same after resize.";
    return false;
  }
  float fx = static_cast<float>(origin_width) / static_cast<float>(width);
  float fy = static_cast<float>(origin_height) / static_cast<float>(height);
  const dim3 block(32, 8);

  const dim3 grid(divup(width, block.x), divup(height, block.y));

  resize_linear_kernel<<<grid, block>>>(
      src.gpu_data(), dst->mutable_gpu_data(), origin_channel, origin_height,
      origin_width, stepwidth, height, width, fx, fy);
  return true;
}

bool ResizeGPU(const apollo::perception::base::Blob<uint8_t> &src_gpu,
               std::shared_ptr<apollo::perception::base::Blob<float>> dst,
               int stepwidth, int start_axis, int mean_b, int mean_g,
               int mean_r, bool channel_axis, float scale) {
  int width = dst->shape(2);
  int height = dst->shape(1);
  int channel = dst->shape(3);
  int origin_channel = src_gpu.shape(3);
  int origin_height = src_gpu.shape(1);
  int origin_width = src_gpu.shape(2);

  if (!channel_axis) {
    // channel_axis: false
    // SRC: 1 H W C
    // DST: 1 C H W
    width = dst->shape(3);
    height = dst->shape(2);
    channel = dst->shape(1);
  }
  // channel_axis: true
  // SRC: 1 H W C
  // DST: 1 H W C
  if (origin_channel != channel) {
    AERROR << "channel should be the same after resize.";
    return false;
  }

  float fx = static_cast<float>(origin_width) / static_cast<float>(width);
  float fy = static_cast<float>(origin_height) / static_cast<float>(height);
  const dim3 block(32, 8);
  const dim3 grid(divup(width, block.x), divup(height, block.y));

  resize_linear_kernel_mean<<<grid, block>>>(
      (const unsigned char *)src_gpu.gpu_data(),
      dst->mutable_gpu_data() + dst->offset(start_axis), origin_channel,
      origin_height, origin_width, stepwidth, height, width, fx, fy, mean_b,
      mean_g, mean_r, channel_axis, scale);
  return true;
}

bool ResizeGPU(const base::Image8U &src,
               std::shared_ptr<apollo::perception::base::Blob<float>> dst,
               int stepwidth, int start_axis, float mean_b, float mean_g,
               float mean_r, bool channel_axis, float scale) {
  int width = dst->shape(2);
  int height = dst->shape(1);
  int channel = dst->shape(3);
  int origin_channel = src.channels();
  int origin_height = src.rows();
  int origin_width = src.cols();

  if (!channel_axis) {
    // channel_axis: false
    // SRC: 1 H W C
    // DST: 1 C H W
    width = dst->shape(3);
    height = dst->shape(2);
    channel = dst->shape(1);
  }
  // channel_axis: true
  // SRC: 1 H W C
  // DST: 1 H W C
  if (origin_channel != channel) {
    AERROR << "channel should be the same after resize.";
    return false;
  }

  float fx = static_cast<float>(origin_width) / static_cast<float>(width);
  float fy = static_cast<float>(origin_height) / static_cast<float>(height);
  const dim3 block(32, 8);
  const dim3 grid(divup(width, block.x), divup(height, block.y));

  resize_linear_kernel_mean<<<grid, block>>>(
      src.gpu_data(), dst->mutable_gpu_data() + dst->offset(start_axis),
      origin_channel, origin_height, origin_width, stepwidth, height, width, fx,
      fy, mean_b, mean_g, mean_r, channel_axis, scale);
  return true;
}

__global__ void image_padding_kernel(const unsigned char *src, uint8_t *dst,
                                     int origin_height, int origin_width,
                                     int origin_channel, int stepwidth,
                                     int dst_height, int dst_width,
                                     int dst_channel, int left_pad,
                                     int right_pad, int top_pad, int bottom_pad,
                                     int value) {
  const int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
  const int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
  // src 1 H W C
  // dst 1 C H W

  if (dst_x < dst_width && dst_y < dst_height) {
    if (dst_x < left_pad || dst_x >= (dst_width - right_pad) ||
        dst_y < top_pad || dst_y >= (dst_height - bottom_pad)) {
      for (int c = 0; c < dst_channel; c++) {
        int dst_idx = dst_height * dst_width * c + dst_y * dst_width + dst_x;
        dst[dst_idx] = value;
      }
    } else {
      int origin_x = dst_x - left_pad;
      int origin_y = dst_y - top_pad;
      int origin_idx = (origin_y * stepwidth + origin_x) * origin_channel;
      for (int c = 0; c < dst_channel; c++) {
        int dst_idx = dst_height * dst_width * c + dst_y * dst_width + dst_x;
        dst[dst_idx] = src[origin_idx + c];
      }
    }
  }
}

__global__ void same_order_padding_kernel(
    const unsigned char *src, uint8_t *dst, int origin_height, int origin_width,
    int origin_channel, int stepwidth, int dst_height, int dst_width,
    int dst_channel, int left_pad, int right_pad, int top_pad, int bottom_pad,
    int value) {
  const int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
  const int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
  // src 1 H W C
  // dst 1 H W C

  if (dst_x < dst_width && dst_y < dst_height) {
    if (dst_x < left_pad || dst_x >= (dst_width - right_pad) ||
        dst_y < top_pad || dst_y >= (dst_height - bottom_pad)) {
      int dst_idx = (dst_y * dst_width + dst_x) * origin_channel;
      for (int c = 0; c < dst_channel; c++) {
        dst[dst_idx + c] = value;
      }
    } else {
      int origin_x = dst_x - left_pad;
      int origin_y = dst_y - top_pad;
      int origin_idx = (origin_y * stepwidth + origin_x) * origin_channel;
      int dst_idx =
          (dst_y * dst_width + dst_x) * origin_channel;  // with same order
      for (int c = 0; c < dst_channel; c++) {
        dst[dst_idx + c] = src[origin_idx + c];  // with same order
      }
    }
  }
}
bool ImageZeroPadding(const base::Image8U &src, base::Image8U *dst,
                      int stepwidth, int left_pad, int right_pad, int top_pad,
                      int bottom_pad, int value, cudaStream_t stream,
                      bool same_order) {
  int src_width = src.cols();
  int src_height = src.rows();
  int src_channels = src.channels();

  int dst_width = dst->cols();
  int dst_height = dst->rows();
  int dst_channels = dst->channels();

  const dim3 block(32, 8);
  const dim3 grid(divup(dst_width, block.x), divup(dst_height, block.y));
  if (same_order) {
    same_order_padding_kernel<<<grid, block, 0, stream>>>(
        src.gpu_data(), dst->mutable_gpu_data(), src_height, src_width,
        src_channels, stepwidth, dst_height, dst_width, dst_channels, left_pad,
        right_pad, top_pad, bottom_pad, value);
  } else {
    image_padding_kernel<<<grid, block, 0, stream>>>(
        src.gpu_data(), dst->mutable_gpu_data(), src_height, src_width,
        src_channels, stepwidth, dst_height, dst_width, dst_channels, left_pad,
        right_pad, top_pad, bottom_pad, value);
  }

  return true;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
