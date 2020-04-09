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

// headers in local files
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/nms_cuda.h"

namespace apollo {
namespace perception {
namespace lidar {

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
                           const int NUM_BOX_CORNERS) {
  const int row_start = blockIdx.y;
  const int col_start = blockIdx.x;

  const int block_threads = blockDim.x;

  const int row_size = min(n_boxes - row_start * block_threads, block_threads);
  const int col_size = min(n_boxes - col_start * block_threads, block_threads);

  __shared__ float block_boxes[NUM_THREADS_MACRO * NUM_2D_BOX_CORNERS_MACRO];
  if (threadIdx.x < col_size) {
    block_boxes[threadIdx.x * NUM_BOX_CORNERS + 0] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * NUM_BOX_CORNERS +
                  0];
    block_boxes[threadIdx.x * NUM_BOX_CORNERS + 1] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * NUM_BOX_CORNERS +
                  1];
    block_boxes[threadIdx.x * NUM_BOX_CORNERS + 2] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * NUM_BOX_CORNERS +
                  2];
    block_boxes[threadIdx.x * NUM_BOX_CORNERS + 3] =
        dev_boxes[(block_threads * col_start + threadIdx.x) * NUM_BOX_CORNERS +
                  3];
  }
  __syncthreads();

  if (threadIdx.x < row_size) {
    const int cur_box_idx = block_threads * row_start + threadIdx.x;
    const float cur_box[NUM_2D_BOX_CORNERS_MACRO] = {
        dev_boxes[cur_box_idx * NUM_BOX_CORNERS + 0],
        dev_boxes[cur_box_idx * NUM_BOX_CORNERS + 1],
        dev_boxes[cur_box_idx * NUM_BOX_CORNERS + 2],
        dev_boxes[cur_box_idx * NUM_BOX_CORNERS + 3]};
    uint64_t t = 0;
    int start = 0;
    if (row_start == col_start) {
      start = threadIdx.x + 1;
    }
    for (int i = start; i < col_size; i++) {
      if (devIoU(cur_box, block_boxes + i * NUM_BOX_CORNERS) >
          nms_overlap_thresh) {
        t |= 1ULL << i;
      }
    }
    const int col_blocks = DIVUP(n_boxes, block_threads);
    dev_mask[cur_box_idx * col_blocks + col_start] = t;
  }
}

NMSCuda::NMSCuda(const int NUM_THREADS, const int NUM_BOX_CORNERS,
                 const float nms_overlap_threshold)
    : NUM_THREADS_(NUM_THREADS),
      NUM_BOX_CORNERS_(NUM_BOX_CORNERS),
      nms_overlap_threshold_(nms_overlap_threshold) {}

void NMSCuda::doNMSCuda(const int host_filter_count,
                        float *dev_sorted_box_for_nms, int *out_keep_inds,
                        int *out_num_to_keep) {
  const int col_blocks = DIVUP(host_filter_count, NUM_THREADS_);
  dim3 blocks(DIVUP(host_filter_count, NUM_THREADS_),
              DIVUP(host_filter_count, NUM_THREADS_));
  dim3 threads(NUM_THREADS_);

  uint64_t *dev_mask = NULL;
  GPU_CHECK(cudaMalloc(
      &dev_mask, host_filter_count * col_blocks * sizeof(uint64_t)));

  nms_kernel<<<blocks, threads>>>(host_filter_count, nms_overlap_threshold_,
                                  dev_sorted_box_for_nms, dev_mask,
                                  NUM_BOX_CORNERS_);

  // postprocess for nms output
  std::vector<uint64_t> host_mask(host_filter_count * col_blocks);
  GPU_CHECK(
      cudaMemcpy(&host_mask[0], dev_mask,
                 sizeof(uint64_t) * host_filter_count * col_blocks,
                 cudaMemcpyDeviceToHost));
  std::vector<uint64_t> remv(col_blocks);
  memset(&remv[0], 0, sizeof(uint64_t) * col_blocks);

  for (int i = 0; i < host_filter_count; i++) {
    int nblock = i / NUM_THREADS_;
    int inblock = i % NUM_THREADS_;

    if (!(remv[nblock] & (1ULL << inblock))) {
      out_keep_inds[(*out_num_to_keep)++] = i;
      uint64_t *p = &host_mask[0] + i * col_blocks;
      for (int j = nblock; j < col_blocks; j++) {
        remv[j] |= p[j];
      }
    }
  }
  GPU_CHECK(cudaFree(dev_mask));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
