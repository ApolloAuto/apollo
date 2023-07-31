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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <algorithm>

// headers in local files
#include "modules/perception/lidar_detection/detector/point_pillars_detection/anchor_mask_cuda.h"
#include "modules/perception/lidar_detection/detector/point_pillars_detection/common.h"

namespace apollo {
namespace perception {
namespace lidar {

// modified prefix sum code from
// https://www.mimuw.edu.pl/~ps209291/kgkp/slides/scan.pdf
__global__ void scan_x(int* g_odata, int* g_idata, int n) {
  extern __shared__ int temp[];  // allocated on invocation
  int thid = threadIdx.x;
  int bid = blockIdx.x;
  int bdim = blockDim.x;
  int offset = 1;
  temp[2 * thid] =
      g_idata[bid * bdim * 2 + 2 * thid];  // load input into shared memory
  temp[2 * thid + 1] = g_idata[bid * bdim * 2 + 2 * thid + 1];
  for (int d = n >> 1; d > 0; d >>= 1) {  // build sum in place up the tree
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      temp[bi] += temp[ai];
    }
    offset *= 2;
  }
  if (thid == 0) {
    temp[n - 1] = 0;
  }                                 // clear the last element
  for (int d = 1; d < n; d *= 2) {  // traverse down tree & build scan
    offset >>= 1;
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      int t = temp[ai];
      temp[ai] = temp[bi];
      temp[bi] += t;
    }
  }
  __syncthreads();
  g_odata[bid * bdim * 2 + 2 * thid] =
      temp[2 * thid + 1];  // write results to device memory
  int second_ind = 2 * thid + 2;
  if (second_ind == bdim * 2) {
    g_odata[bid * bdim * 2 + 2 * thid + 1] =
        temp[2 * thid + 1] + g_idata[bid * bdim * 2 + 2 * thid + 1];
  } else {
    g_odata[bid * bdim * 2 + 2 * thid + 1] = temp[2 * thid + 2];
  }
}

// modified prefix sum code from
// https://www.mimuw.edu.pl/~ps209291/kgkp/slides/scan.pdf
__global__ void scan_y(int* g_odata, int* g_idata, int n) {
  extern __shared__ int temp[];  // allocated on invocation
  int thid = threadIdx.x;
  int bid = blockIdx.x;
  int bdim = blockDim.x;
  int gdim = gridDim.x;
  int offset = 1;
  temp[2 * thid] =
      g_idata[bid + 2 * thid * gdim];  // load input into shared memory
  temp[2 * thid + 1] = g_idata[bid + 2 * thid * gdim + gdim];
  for (int d = n >> 1; d > 0; d >>= 1) {  // build sum in place up the tree
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      temp[bi] += temp[ai];
    }
    offset *= 2;
  }
  if (thid == 0) {
    temp[n - 1] = 0;
  }                                 // clear the last element
  for (int d = 1; d < n; d *= 2) {  // traverse down tree & build scan
    offset >>= 1;
    __syncthreads();
    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      int t = temp[ai];
      temp[ai] = temp[bi];
      temp[bi] += t;
    }
  }
  __syncthreads();
  g_odata[bid + 2 * thid * gdim] =
      temp[2 * thid + 1];  // write results to device memory
  int second_ind = 2 * thid + 2;
  if (second_ind == bdim * 2) {
    g_odata[bid + 2 * thid * gdim + gdim] =
        temp[2 * thid + 1] + g_idata[bid + 2 * thid * gdim + gdim];
  } else {
    g_odata[bid + 2 * thid * gdim + gdim] = temp[2 * thid + 2];
  }
}

__global__ void make_anchor_mask_kernel(
    const float* dev_box_anchors_min_x, const float* dev_box_anchors_min_y,
    const float* dev_box_anchors_max_x, const float* dev_box_anchors_max_y,
    int* dev_sparse_pillar_map, int* dev_anchor_mask, const float min_x_range,
    const float min_y_range, const float pillar_x_size,
    const float pillar_y_size, const int grid_x_size, const int grid_y_size,
    const int num_inds_for_scan) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  int anchor_coor[NUM_2D_BOX_CORNERS_MACRO] = {0};
  const int grid_x_size_1 = grid_x_size - 1;  // grid_x_size - 1
  const int grid_y_size_1 = grid_y_size - 1;  // grid_y_size - 1

  anchor_coor[0] =
      floor((dev_box_anchors_min_x[tid] - min_x_range) / pillar_x_size);
  anchor_coor[1] =
      floor((dev_box_anchors_min_y[tid] - min_y_range) / pillar_y_size);
  anchor_coor[2] =
      floor((dev_box_anchors_max_x[tid] - min_x_range) / pillar_x_size);
  anchor_coor[3] =
      floor((dev_box_anchors_max_y[tid] - min_y_range) / pillar_y_size);
  anchor_coor[0] = max(anchor_coor[0], 0);
  anchor_coor[1] = max(anchor_coor[1], 0);
  anchor_coor[2] = min(anchor_coor[2], grid_x_size_1);
  anchor_coor[3] = min(anchor_coor[3], grid_y_size_1);

  int right_top = dev_sparse_pillar_map[anchor_coor[3] * num_inds_for_scan +
                                        anchor_coor[2]];
  int left_bottom = dev_sparse_pillar_map[anchor_coor[1] * num_inds_for_scan +
                                          anchor_coor[0]];
  int left_top = dev_sparse_pillar_map[anchor_coor[3] * num_inds_for_scan +
                                       anchor_coor[0]];
  int right_bottom = dev_sparse_pillar_map[anchor_coor[1] * num_inds_for_scan +
                                           anchor_coor[2]];

  int area = right_top - left_top - right_bottom + left_bottom;
  if (area > 1) {
    dev_anchor_mask[tid] = 1;
  } else {
    dev_anchor_mask[tid] = 0;
  }
}

AnchorMaskCuda::AnchorMaskCuda(
    const int num_threads, const int num_inds_for_scan, const int num_anchor,
    const float min_x_range, const float min_y_range, const float pillar_x_size,
    const float pillar_y_size, const int grid_x_size, const int grid_y_size)
    : num_threads_(num_threads),
      num_inds_for_scan_(num_inds_for_scan),
      num_anchor_(num_anchor),
      min_x_range_(min_x_range),
      min_y_range_(min_y_range),
      pillar_x_size_(pillar_x_size),
      pillar_y_size_(pillar_y_size),
      grid_x_size_(grid_x_size),
      grid_y_size_(grid_y_size) {}

void AnchorMaskCuda::DoAnchorMaskCuda(
    int* dev_sparse_pillar_map, int* dev_cumsum_along_x,
    int* dev_cumsum_along_y, const float* dev_box_anchors_min_x,
    const float* dev_box_anchors_min_y, const float* dev_box_anchors_max_x,
    const float* dev_box_anchors_max_y, int* dev_anchor_mask) {
  scan_x<<<num_inds_for_scan_, num_inds_for_scan_ / 2,
           num_inds_for_scan_ * sizeof(int)>>>(
      dev_cumsum_along_x, dev_sparse_pillar_map, num_inds_for_scan_);
  scan_y<<<num_inds_for_scan_, num_inds_for_scan_ / 2,
           num_inds_for_scan_ * sizeof(int)>>>(
      dev_cumsum_along_y, dev_cumsum_along_x, num_inds_for_scan_);
  GPU_CHECK(cudaMemcpy(dev_sparse_pillar_map, dev_cumsum_along_y,
                       num_inds_for_scan_ * num_inds_for_scan_ * sizeof(int),
                       cudaMemcpyDeviceToDevice));

  int num_blocks = DIVUP(num_anchor_, num_threads_);
  make_anchor_mask_kernel<<<num_blocks, num_threads_>>>(
      dev_box_anchors_min_x, dev_box_anchors_min_y, dev_box_anchors_max_x,
      dev_box_anchors_max_y, dev_sparse_pillar_map, dev_anchor_mask,
      min_x_range_, min_y_range_, pillar_x_size_, pillar_y_size_, grid_x_size_,
      grid_y_size_, num_inds_for_scan_);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
