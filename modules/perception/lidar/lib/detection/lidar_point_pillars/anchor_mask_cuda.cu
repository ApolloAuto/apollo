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
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/anchor_mask_cuda.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/common.h"

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
  }                               // clear the last element
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
  }                               // clear the last element
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
    int* dev_sparse_pillar_map, int* dev_anchor_mask, const float MIN_X_RANGE,
    const float MIN_Y_RANGE, const float PILLAR_X_SIZE,
    const float PILLAR_Y_SIZE, const int GRID_X_SIZE, const int GRID_Y_SIZE,
    const int NUM_INDS_FOR_SCAN) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  int anchor_coor[NUM_2D_BOX_CORNERS_MACRO] = {0};
  const int GRID_X_SIZE_1 = GRID_X_SIZE - 1;  // grid_x_size - 1
  const int GRID_Y_SIZE_1 = GRID_Y_SIZE - 1;  // grid_y_size - 1

  anchor_coor[0] =
      floor((dev_box_anchors_min_x[tid] - MIN_X_RANGE) / PILLAR_X_SIZE);
  anchor_coor[1] =
      floor((dev_box_anchors_min_y[tid] - MIN_Y_RANGE) / PILLAR_Y_SIZE);
  anchor_coor[2] =
      floor((dev_box_anchors_max_x[tid] - MIN_X_RANGE) / PILLAR_X_SIZE);
  anchor_coor[3] =
      floor((dev_box_anchors_max_y[tid] - MIN_Y_RANGE) / PILLAR_Y_SIZE);
  anchor_coor[0] = max(anchor_coor[0], 0);
  anchor_coor[1] = max(anchor_coor[1], 0);
  anchor_coor[2] = min(anchor_coor[2], GRID_X_SIZE_1);
  anchor_coor[3] = min(anchor_coor[3], GRID_Y_SIZE_1);

  int right_top = dev_sparse_pillar_map[anchor_coor[3] * NUM_INDS_FOR_SCAN +
                                        anchor_coor[2]];
  int left_bottom = dev_sparse_pillar_map[anchor_coor[1] * NUM_INDS_FOR_SCAN +
                                          anchor_coor[0]];
  int left_top = dev_sparse_pillar_map[anchor_coor[3] * NUM_INDS_FOR_SCAN +
                                       anchor_coor[0]];
  int right_bottom = dev_sparse_pillar_map[anchor_coor[1] * NUM_INDS_FOR_SCAN +
                                           anchor_coor[2]];

  int area = right_top - left_top - right_bottom + left_bottom;
  if (area > 1) {
    dev_anchor_mask[tid] = 1;
  } else {
    dev_anchor_mask[tid] = 0;
  }
}

AnchorMaskCuda::AnchorMaskCuda(
    const int NUM_INDS_FOR_SCAN, const int NUM_ANCHOR_X_INDS,
    const int NUM_ANCHOR_Y_INDS, const int NUM_ANCHOR_R_INDS,
    const float MIN_X_RANGE, const float MIN_Y_RANGE, const float PILLAR_X_SIZE,
    const float PILLAR_Y_SIZE, const int GRID_X_SIZE, const int GRID_Y_SIZE)
    : NUM_INDS_FOR_SCAN_(NUM_INDS_FOR_SCAN),
      NUM_ANCHOR_X_INDS_(NUM_ANCHOR_X_INDS),
      NUM_ANCHOR_Y_INDS_(NUM_ANCHOR_Y_INDS),
      NUM_ANCHOR_R_INDS_(NUM_ANCHOR_R_INDS),
      MIN_X_RANGE_(MIN_X_RANGE),
      MIN_Y_RANGE_(MIN_Y_RANGE),
      PILLAR_X_SIZE_(PILLAR_X_SIZE),
      PILLAR_Y_SIZE_(PILLAR_Y_SIZE),
      GRID_X_SIZE_(GRID_X_SIZE),
      GRID_Y_SIZE_(GRID_Y_SIZE) {}

void AnchorMaskCuda::doAnchorMaskCuda(
    int* dev_sparse_pillar_map, int* dev_cumsum_along_x,
    int* dev_cumsum_along_y, const float* dev_box_anchors_min_x,
    const float* dev_box_anchors_min_y, const float* dev_box_anchors_max_x,
    const float* dev_box_anchors_max_y, int* dev_anchor_mask) {
  scan_x<<<NUM_INDS_FOR_SCAN_, NUM_INDS_FOR_SCAN_ / 2,
           NUM_INDS_FOR_SCAN_ * sizeof(int)>>>(
      dev_cumsum_along_x, dev_sparse_pillar_map, NUM_INDS_FOR_SCAN_);
  scan_y<<<NUM_INDS_FOR_SCAN_, NUM_INDS_FOR_SCAN_ / 2,
           NUM_INDS_FOR_SCAN_ * sizeof(int)>>>(
      dev_cumsum_along_y, dev_cumsum_along_x, NUM_INDS_FOR_SCAN_);
  GPU_CHECK(cudaMemcpy(dev_sparse_pillar_map, dev_cumsum_along_y,
                       NUM_INDS_FOR_SCAN_ * NUM_INDS_FOR_SCAN_ * sizeof(int),
                       cudaMemcpyDeviceToDevice));
  make_anchor_mask_kernel<<<NUM_ANCHOR_X_INDS_ * NUM_ANCHOR_R_INDS_,
                            NUM_ANCHOR_Y_INDS_>>>(
      dev_box_anchors_min_x, dev_box_anchors_min_y, dev_box_anchors_max_x,
      dev_box_anchors_max_y, dev_sparse_pillar_map, dev_anchor_mask,
      MIN_X_RANGE_, MIN_Y_RANGE_, PILLAR_X_SIZE_, PILLAR_Y_SIZE_, GRID_X_SIZE_,
      GRID_Y_SIZE_, NUM_INDS_FOR_SCAN_);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
