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

// ------------------------------------------------------------------
// Copyright (c) 2015 Microsoft
// Licensed under The MIT License
// Modified from MATLAB Faster R-CNN
// (https://github.com/shaoqingren/faster_rcnn)
// ------------------------------------------------------------------

#include <algorithm>
#include <vector>

#include "modules/perception/common/inference/migraphx/plugins/kernels.h"

namespace apollo {
namespace perception {
namespace inference {

__device__ inline float devIoU(float const *const a, float const *const b) {
  float left = max(a[0], b[0]), right = min(a[2], b[2]);
  float top = max(a[1], b[1]), bottom = min(a[3], b[3]);
  float width = max(right - left + 1, 0.f), height = max(bottom - top + 1, 0.f);
  float interS = width * height;
  float Sa = (a[2] - a[0] + 1) * (a[3] - a[1] + 1);
  float Sb = (b[2] - b[0] + 1) * (b[3] - b[1] + 1);
  return interS / (Sa + Sb - interS);
}

__global__ void nms_kernel(const int n_boxes, const float nms_overlap_thresh,
                           const float *dev_boxes, uint64_t *dev_mask,
                           const int num_box_corners) {
  const int row_start = blockIdx.y;
  const int col_start = blockIdx.x;

  const int block_threads = blockDim.x;

  const int row_size = min(n_boxes - row_start * block_threads, block_threads);
  const int col_size = min(n_boxes - col_start * block_threads, block_threads);

  __shared__ float block_boxes[NUM_THREADS_MACRO * NUM_2D_BOX_CORNERS_MACRO];
  if (threadIdx.x < col_size) {
    block_boxes[threadIdx.x * num_box_corners + 0] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * num_box_corners +
                  0];
    block_boxes[threadIdx.x * num_box_corners + 1] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * num_box_corners +
                  1];
    block_boxes[threadIdx.x * num_box_corners + 2] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * num_box_corners +
                  2];
    block_boxes[threadIdx.x * num_box_corners + 3] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * num_box_corners +
                  3];
  }
  __syncthreads();

  if (threadIdx.x < row_size) {
    const int cur_box_idx = block_threads * row_start + threadIdx.x;
    const float cur_box[NUM_2D_BOX_CORNERS_MACRO] = {
        dev_boxes[cur_box_idx * num_box_corners + 0],
        dev_boxes[cur_box_idx * num_box_corners + 1],
        dev_boxes[cur_box_idx * num_box_corners + 2],
        dev_boxes[cur_box_idx * num_box_corners + 3]};
    uint64_t t = 0;
    int start = 0;
    if (row_start == col_start) {
      start = threadIdx.x + 1;
    }
    for (int i = start; i < col_size; ++i) {
      if (devIoU(cur_box, block_boxes + i * num_box_corners) >
          nms_overlap_thresh) {
        t |= 1ULL << i;
      }
    }
    const int col_blocks = DIVUP(n_boxes, block_threads);
    dev_mask[cur_box_idx * col_blocks + col_start] = t;
  }
}

// boxes dims: [topN, 4], all_probs dims: [topN, num_prob]
// out_boxes dims: [num_out, batch_id + 4 + (optional) num_class]
// num_out <= topN
__global__ void convert_out_boxes_kernel(const int nthreads, const float *boxes,
                                         const float *all_probs,
                                         const int num_prob, const int batch_id,
                                         const bool rpn_proposal_output_score,
                                         float *out_boxes) {
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  if (index < nthreads) {
    int out_channel = rpn_proposal_output_score ? (5 + num_prob) : 5;
    out_boxes[index * out_channel] = (float)batch_id;
    for (int i = 1; i < 5; ++i) {
      out_boxes[index * out_channel + i] = boxes[index * 4 + i - 1];
    }
    if (rpn_proposal_output_score) {
      for (int i = 5; i < out_channel; ++i) {
        out_boxes[index * out_channel + i] =
            all_probs[index * num_prob + i - 5];
      }
    }
  }
}

void NmsForward(const bool rpn_proposal_output_score,
                const int host_filter_count, const int num_box_corners,
                const float nms_overlap_threshold, const int num_candidate,
                const int top_n, const int batch_id, const int num_prob,
                float *dev_sorted_box_for_nms, float *scores, float *all_probs,
                float *out_boxes, int *acc_box_num, cudaStream_t stream) {
  const int col_blocks = DIVUP(host_filter_count, NUM_THREADS_MACRO);
  dim3 blocks(DIVUP(host_filter_count, NUM_THREADS_MACRO),
              DIVUP(host_filter_count, NUM_THREADS_MACRO));
  dim3 threads(NUM_THREADS_MACRO);

  int out_num_to_keep = 0;
  int *out_keep_inds = new int[host_filter_count]();

  uint64_t *dev_mask = NULL;
  BASE_GPU_CHECK(
      cudaMalloc(&dev_mask, host_filter_count * col_blocks * sizeof(uint64_t)));

  nms_kernel<<<blocks, threads, 0, stream>>>(
      host_filter_count, nms_overlap_threshold, dev_sorted_box_for_nms,
      dev_mask, num_box_corners);

  // postprocess for nms output
  std::vector<uint64_t> host_mask(host_filter_count * col_blocks);
  BASE_GPU_CHECK(
      cudaMemcpyAsync(&host_mask[0], dev_mask,
                      sizeof(uint64_t) * host_filter_count * col_blocks,
                      cudaMemcpyDeviceToHost, stream));
  std::vector<uint64_t> remv(col_blocks);
  memset(&remv[0], 0, sizeof(uint64_t) * col_blocks);

  for (int i = 0; i < host_filter_count; ++i) {
    int nblock = i / NUM_THREADS_MACRO;
    int inblock = i % NUM_THREADS_MACRO;

    if (!(remv[nblock] & (1ULL << inblock))) {
      out_keep_inds[out_num_to_keep++] = i;
      uint64_t *p = &host_mask[0] + i * col_blocks;
      for (int j = nblock; j < col_blocks; ++j) {
        remv[j] |= p[j];
      }
    }
  }

  // gather boxes by kept indexes, and keep top N boxes
  int *dev_keep_inds, *dev_keep_num;
  float *kept_boxes, *kept_scores, *kept_all_probs;
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&dev_keep_inds),
                             num_candidate * sizeof(int)));
  BASE_GPU_CHECK(
      cudaMalloc(reinterpret_cast<void **>(&dev_keep_num), sizeof(int)));
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&kept_boxes),
                             top_n * 4 * sizeof(float)));
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&kept_scores),
                             top_n * sizeof(float)));
  BASE_GPU_CHECK(cudaMalloc(reinterpret_cast<void **>(&kept_all_probs),
                             top_n * num_prob * sizeof(float)));
  BASE_GPU_CHECK(cudaMemcpyAsync(dev_keep_inds, out_keep_inds,
                                  out_num_to_keep * sizeof(int),
                                  cudaMemcpyHostToDevice, stream));
  BASE_GPU_CHECK(cudaMemcpyAsync(dev_keep_num, &out_num_to_keep, sizeof(int),
                                  cudaMemcpyHostToDevice, stream));
  int nthreads = top_n;
  int block_size = (nthreads - 1) / NUM_THREADS_MACRO + 1;
  keep_topN_boxes_cuda(block_size, NUM_THREADS_MACRO, 0, stream, nthreads,
                       dev_sorted_box_for_nms, scores, all_probs, dev_keep_inds,
                       dev_keep_num, rpn_proposal_output_score, num_candidate,
                       num_prob, top_n, kept_boxes, kept_scores,
                       kept_all_probs);

  // convert to output format
  int cur_box_num = std::min(out_num_to_keep, top_n);
  nthreads = cur_box_num;
  block_size = (nthreads - 1) / NUM_THREADS_MACRO + 1;
  convert_out_boxes_kernel<<<block_size, NUM_THREADS_MACRO, 0, stream>>>(
      nthreads, kept_boxes, kept_all_probs, num_prob, batch_id,
      rpn_proposal_output_score, out_boxes);

  *acc_box_num += cur_box_num;

  BASE_GPU_CHECK(cudaFree(dev_mask));
  BASE_GPU_CHECK(cudaFree(dev_keep_inds));
  BASE_GPU_CHECK(cudaFree(dev_keep_num));
  BASE_GPU_CHECK(cudaFree(kept_boxes));
  BASE_GPU_CHECK(cudaFree(kept_scores));
  BASE_GPU_CHECK(cudaFree(kept_all_probs));

  delete[] out_keep_inds;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
