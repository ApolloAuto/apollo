/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/inference/tensorrt/plugins/kernels.h"

namespace apollo {
namespace perception {
namespace inference {

// Decode bbox.
// boxes dims: [num_box, 4], deltas dims: [N, num_box, C, 4],
// out_boxes dims: [N, num_box, C, 4]
// nthreads = N * num_box * C
__global__ void bbox_transform_inv_kernel(
    const int nthreads, const float *boxes, const float *deltas,
    const int num_box, const int num_channel, float *out_boxes) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index >= nthreads) {
    return;
  }

  int box_id = (index / num_channel) % num_box;

  float x_min = boxes[box_id * 4];
  float y_min = boxes[box_id * 4 + 1];
  float x_max = boxes[box_id * 4 + 2];
  float y_max = boxes[box_id * 4 + 3];
  float w = x_max - x_min + 1;
  float h = y_max - y_min + 1;
  float x_ctr = x_min + 0.5 * (w - 1);
  float y_ctr = y_min + 0.5 * (h - 1);

  float dx = deltas[index * 4];
  float dy = deltas[index * 4 + 1];
  float dw = deltas[index * 4 + 2];
  float dh = deltas[index * 4 + 3];

  float pred_x_ctr = dx * w + x_ctr;
  float pred_y_ctr = dy * h + y_ctr;
  float pred_w = std::exp(dw) * w;
  float pred_h = std::exp(dh) * h;

  out_boxes[index * 4] = pred_x_ctr - 0.5 * (pred_w - 1);      // pred x_min
  out_boxes[index * 4 + 1] = pred_y_ctr - 0.5 * (pred_h - 1);  // pred y_min
  out_boxes[index * 4 + 2] = pred_x_ctr + 0.5 * (pred_w - 1);  // pred x_max
  out_boxes[index * 4 + 3] = pred_y_ctr + 0.5 * (pred_h - 1);  // pred y_max
}

// boxes dim: [N, num_box, 4], nthreads = N * num_box * 4
__global__ void clip_boxes_kernel(const int nthreads, float *boxes,
                                  const float height, const float width) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index >= nthreads) {
    return;
  }

  // refine x_min, x_max to be in [0, img_width)
  if (index % 4 == 0 || index % 4 == 2) {
    if (boxes[index] < 0) {
      boxes[index] = 0;
    } else if (boxes[index] > width - 1) {
      boxes[index] = width - 1;
    }
  } else {  // refine y_min, y_max to be in [0, img_height)
    if (boxes[index] < 0) {
      boxes[index] = 0;
    } else if (boxes[index] > height - 1) {
      boxes[index] = height - 1;
    }
  }
}

// boxes dims: [N, num_box, num_channel, 4],
// filtered_boxes dims: [N, num_box, 4]
// scores dims: [N, num_box, num_class], filtered_scores dims: [N, num_box]
// all_probs dims: [N, num_box, num_prob],
// filtered_all_probs dims: [N, num_box, num_prob]
// filtered_count dims: [N]
__global__ void filter_boxes_kernel(
    const int nthreads, const float *boxes, const float *scores,
    const float *all_probs, const int num_box, const int num_channel,
    const int num_class, const int num_prob, const int filter_channel,
    const int filter_class, const int min_size_mode, const float min_size_h,
    const float min_size_w, const float threshold_score, float *filtered_boxes,
    float *filtered_scores, float *filtered_all_probs, int *filtered_count) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index >= nthreads) {
    return;
  }

  int batch_id = index / num_box;
  if (scores[index * num_class + filter_class] > threshold_score) {
    bool keep = true;
    int box_id = index * num_channel + filter_channel;
    float w = boxes[box_id * 4 + 2] - boxes[box_id * 4] + 1;
    float h = boxes[box_id * 4 + 3] - boxes[box_id * 4 + 1] + 1;
    if (min_size_mode == 0) {
      // filter boxes with minimum size of height & width
      if (h < min_size_h || w < min_size_w) {
        keep = false;
      }
    } else if (min_size_mode == 1) {
      // filter boxes with minimum size of height or width
      if (h < min_size_h && w < min_size_w) {
        keep = false;
      }
    }

    if (keep) {
      int counter = atomicAdd(&filtered_count[batch_id], 1);
      for (int i = 0; i < 4; ++i) {
        filtered_boxes[batch_id * num_box * 4 + counter * 4 + i] =
            boxes[box_id * 4 + i];
      }
      filtered_scores[batch_id * num_box + counter] =
          scores[index * num_class + filter_class];
      if (all_probs != nullptr && filtered_all_probs != nullptr) {
        for (int i = 0; i < num_prob; ++i) {
          filtered_all_probs[batch_id * num_box * num_prob +
                             counter * num_prob + i] =
              all_probs[index * num_prob + i];
        }
      }
    }
  }
}

// Gather boxes by indexes and keep top N boxes.
// boxes dims: [N, num_box, 4], scores dims: [N, num_box],
// all_probs dims: [N, num_box, num_prob]
// indexes dims: [N, num_box], count dims: [N]
// out_boxes dims: [N, topN, 4], out_scores dims: [N, topN]
// out_all_probs dims: [N, topN, num_prob]
// nthreads = N * max_num_box
__global__ void keep_topN_boxes_kernel(
    const int nthreads, const float *boxes, const float *scores,
    const float *all_probs, const int *indexes, const int *count,
    const bool keep_score, const int num_box, const int num_prob,
    const int topN, float *out_boxes, float *out_scores, float *out_all_probs) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index >= nthreads) {
    return;
  }

  int batch_id = index / topN;
  int box_id = index % topN;
  if (box_id < count[batch_id]) {
    int in_box_id = indexes[batch_id * num_box + box_id];
    for (int i = 0; i < 4; ++i) {
      out_boxes[index * 4 + i] =
          boxes[batch_id * num_box * 4 + in_box_id * 4 + i];
    }

    if (keep_score) {
      out_scores[index] = scores[batch_id * num_box + in_box_id];
      for (int i = 0; i < num_prob; ++i) {
        out_all_probs[index * num_prob + i] =
            all_probs[batch_id * num_box * num_prob + in_box_id * num_prob + i];
      }
    }
  }
}

__global__ void repeatedly_add_kernel(const int nthreads, const float *in_data,
                                      float *out_data, const float *add_vec,
                                      int add_vec_size) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < nthreads) {
    out_data[index] = in_data[index] + add_vec[index % add_vec_size];
  }
}

__global__ void repeatedly_mul_kernel(const int nthreads, const float *in_data,
                                      float *out_data, const float *mul_vec,
                                      int mul_vec_size) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < nthreads) {
    out_data[index] = in_data[index] * mul_vec[index % mul_vec_size];
  }
}

// input dims: [N, C], output dims: [N, C_sliced]
__global__ void slice2d_kernel(const int nthreads, const float *in_data,
                               float *out_data, const int *slice_axises,
                               int slice_axis_num, int input_axis_size) {
  int out_index = threadIdx.x + blockIdx.x * blockDim.x;
  if (out_index < nthreads) {
    int id = out_index / slice_axis_num;
    int slice_axis_id = out_index % slice_axis_num;
    int in_index = slice_axises[slice_axis_id] + id * input_axis_size;
    out_data[out_index] = in_data[in_index];
  }
}

void bbox_transform_inv_cuda(int block_size, int thread_size, int shared_mem,
                             cudaStream_t stream, const int nthreads,
                             const float *boxes, const float *deltas,
                             const int num_box, const int num_channel,
                             float *out_boxes) {
  bbox_transform_inv_kernel<<<block_size, thread_size, shared_mem, stream>>>(
      nthreads, boxes, deltas, num_box, num_channel, out_boxes);
}

void clip_boxes_cuda(int block_size, int thread_size, int shared_mem,
                     cudaStream_t stream, const int nthreads, float *boxes,
                     const float height, const float width) {
  clip_boxes_kernel<<<block_size, thread_size, shared_mem, stream>>>(
      nthreads, boxes, height, width);
}

void filter_boxes_cuda(
    int block_size, int thread_size, int shared_mem, cudaStream_t stream,
    const int nthreads, const float *boxes, const float *scores,
    const float *all_probs, const int num_box, const int num_channel,
    const int num_class, const int num_prob, const int filter_channel,
    const int filter_class, const int min_size_mode, const float min_size_h,
    const float min_size_w, const float threshold_score, float *filtered_boxes,
    float *filtered_scores, float *filtered_all_probs, int *filtered_count) {
  filter_boxes_kernel<<<block_size, thread_size, shared_mem, stream>>>(
      nthreads, boxes, scores, all_probs, num_box, num_channel, num_class,
      num_prob, filter_channel, filter_class, min_size_mode, min_size_h,
      min_size_w, threshold_score, filtered_boxes, filtered_scores,
      filtered_all_probs, filtered_count);
}

void keep_topN_boxes_cuda(int block_size, int thread_size, int shared_mem,
                          cudaStream_t stream, const int nthreads,
                          const float *boxes, const float *scores,
                          const float *all_probs, const int *indexes,
                          const int *count, const bool keep_score,
                          const int num_box, const int num_prob, const int topN,
                          float *out_boxes, float *out_scores,
                          float *out_all_probs) {
  keep_topN_boxes_kernel<<<block_size, thread_size, shared_mem, stream>>>(
      nthreads, boxes, scores, all_probs, indexes, count, keep_score, num_box,
      num_prob, topN, out_boxes, out_scores, out_all_probs);
}

void repeatedly_add_cuda(int block_size, int thread_size, int shared_mem,
                         cudaStream_t stream, const int nthreads,
                         const float *in_data, float *out_data,
                         const float *add_vec, int add_vec_size) {
  repeatedly_add_kernel<<<block_size, thread_size, shared_mem, stream>>>(
      nthreads, in_data, out_data, add_vec, add_vec_size);
}

void repeatedly_mul_cuda(int block_size, int thread_size, int shared_mem,
                         cudaStream_t stream, const int nthreads,
                         const float *in_data, float *out_data,
                         const float *mul_vec, int mul_vec_size) {
  repeatedly_mul_kernel<<<block_size, thread_size, shared_mem, stream>>>(
      nthreads, in_data, out_data, mul_vec, mul_vec_size);
}

void slice2d_cuda(int block_size, int thread_size, int shared_mem,
                  cudaStream_t stream, const int nthreads, const float *in_data,
                  float *out_data, const int *slice_axises, int slice_axis_num,
                  int input_axis_size) {
  slice2d_kernel<<<block_size, thread_size, shared_mem, stream>>>(
      nthreads, in_data, out_data, slice_axises, slice_axis_num,
      input_axis_size);
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
