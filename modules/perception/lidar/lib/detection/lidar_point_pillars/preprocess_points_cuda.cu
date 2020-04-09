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

// headers in STL
#include <iostream>

// headers in local files
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/common.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points_cuda.h"

namespace apollo {
namespace perception {
namespace lidar {

__global__ void make_pillar_histo_kernel(
    const float* dev_points, float* dev_pillar_x_in_coors,
    float* dev_pillar_y_in_coors, float* dev_pillar_z_in_coors,
    float* dev_pillar_i_in_coors, int* pillar_count_histo, const int num_points,
    const int max_points_per_pillar, const int GRID_X_SIZE,
    const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float MIN_X_RANGE,
    const float MIN_Y_RANGE, const float MIN_Z_RANGE, const float PILLAR_X_SIZE,
    const float PILLAR_Y_SIZE, const float PILLAR_Z_SIZE,
    const int NUM_BOX_CORNERS) {
  int th_i = threadIdx.x + blockIdx.x * blockDim.x;
  if (th_i >= num_points) {
    return;
  }
  int y_coor = floor((dev_points[th_i * NUM_BOX_CORNERS + 1] - MIN_Y_RANGE) /
                     PILLAR_Y_SIZE);
  int x_coor = floor((dev_points[th_i * NUM_BOX_CORNERS + 0] - MIN_X_RANGE) /
                     PILLAR_X_SIZE);
  int z_coor = floor((dev_points[th_i * NUM_BOX_CORNERS + 2] - MIN_Z_RANGE) /
                     PILLAR_Z_SIZE);

  if (x_coor >= 0 && x_coor < GRID_X_SIZE && y_coor >= 0 &&
      y_coor < GRID_Y_SIZE && z_coor >= 0 && z_coor < GRID_Z_SIZE) {
    int count =
        atomicAdd(&pillar_count_histo[y_coor * GRID_X_SIZE + x_coor], 1);
    if (count < max_points_per_pillar) {
      int ind = y_coor * GRID_X_SIZE * max_points_per_pillar +
                x_coor * max_points_per_pillar + count;
      dev_pillar_x_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 0];
      dev_pillar_y_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 1];
      dev_pillar_z_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 2];
      dev_pillar_i_in_coors[ind] = dev_points[th_i * NUM_BOX_CORNERS + 3];
    }
  }
}

__global__ void make_pillar_index_kernel(
    int* dev_pillar_count_histo, int* dev_counter, int* dev_pillar_count,
    int* dev_x_coors, int* dev_y_coors, float* dev_x_coors_for_sub,
    float* dev_y_coors_for_sub, float* dev_num_points_per_pillar,
    int* dev_sparse_pillar_map, const int max_pillars,
    const int max_points_per_pillar, const int GRID_X_SIZE,
    const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
    const int NUM_INDS_FOR_SCAN) {
  int x = blockIdx.x;
  int y = threadIdx.x;
  int num_points_at_this_pillar = dev_pillar_count_histo[y * GRID_X_SIZE + x];
  if (num_points_at_this_pillar == 0) {
    return;
  }

  int count = atomicAdd(dev_counter, 1);
  if (count < max_pillars) {
    atomicAdd(dev_pillar_count, 1);
    if (num_points_at_this_pillar >= max_points_per_pillar) {
      dev_num_points_per_pillar[count] = max_points_per_pillar;
    } else {
      dev_num_points_per_pillar[count] = num_points_at_this_pillar;
    }
    dev_x_coors[count] = x;
    dev_y_coors[count] = y;

    // TODO(...): Need to be modified after making properly trained weight
    // Will be modified in ver 1.1
    // x_offset = self.vx / 2 + pc_range[0]
    // y_offset = self.vy / 2 + pc_range[1]
    // x_sub = coors_x.unsqueeze(1) * 0.16 + x_offset
    // y_sub = coors_y.unsqueeze(1) * 0.16 + y_offset
    dev_x_coors_for_sub[count] = x * PILLAR_X_SIZE + 0.1f;
    dev_y_coors_for_sub[count] = y * PILLAR_Y_SIZE + -39.9f;
    dev_sparse_pillar_map[y * NUM_INDS_FOR_SCAN + x] = 1;
  }
}

__global__ void make_pillar_feature_kernel(
    float* dev_pillar_x_in_coors, float* dev_pillar_y_in_coors,
    float* dev_pillar_z_in_coors, float* dev_pillar_i_in_coors,
    float* dev_pillar_x, float* dev_pillar_y, float* dev_pillar_z,
    float* dev_pillar_i, int* dev_x_coors, int* dev_y_coors,
    float* dev_num_points_per_pillar, const int max_points,
    const int GRID_X_SIZE) {
  int ith_pillar = blockIdx.x;
  int num_points_at_this_pillar = dev_num_points_per_pillar[ith_pillar];
  int ith_point = threadIdx.x;
  if (ith_point >= num_points_at_this_pillar) {
    return;
  }
  int x_ind = dev_x_coors[ith_pillar];
  int y_ind = dev_y_coors[ith_pillar];
  int pillar_ind = ith_pillar * max_points + ith_point;
  int coors_ind =
      y_ind * GRID_X_SIZE * max_points + x_ind * max_points + ith_point;
  dev_pillar_x[pillar_ind] = dev_pillar_x_in_coors[coors_ind];
  dev_pillar_y[pillar_ind] = dev_pillar_y_in_coors[coors_ind];
  dev_pillar_z[pillar_ind] = dev_pillar_z_in_coors[coors_ind];
  dev_pillar_i[pillar_ind] = dev_pillar_i_in_coors[coors_ind];
}

__global__ void make_extra_network_input_kernel(
    float* dev_x_coors_for_sub, float* dev_y_coors_for_sub,
    float* dev_num_points_per_pillar, float* dev_x_coors_for_sub_shaped,
    float* dev_y_coors_for_sub_shaped, float* dev_pillar_feature_mask,
    const int MAX_NUM_POINTS_PER_PILLAR) {
  int ith_pillar = blockIdx.x;
  int ith_point = threadIdx.x;
  float x = dev_x_coors_for_sub[ith_pillar];
  float y = dev_y_coors_for_sub[ith_pillar];
  int num_points_for_a_pillar = dev_num_points_per_pillar[ith_pillar];
  int ind = ith_pillar * MAX_NUM_POINTS_PER_PILLAR + ith_point;
  dev_x_coors_for_sub_shaped[ind] = x;
  dev_y_coors_for_sub_shaped[ind] = y;

  if (ith_point < num_points_for_a_pillar) {
    dev_pillar_feature_mask[ind] = 1.0;
  } else {
    dev_pillar_feature_mask[ind] = 0.0;
  }
}

PreprocessPointsCuda::PreprocessPointsCuda(
    const int NUM_THREADS, const int MAX_NUM_PILLARS,
    const int MAX_POINTS_PER_PILLAR, const int NUM_INDS_FOR_SCAN,
    const int GRID_X_SIZE, const int GRID_Y_SIZE, const int GRID_Z_SIZE,
    const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
    const float PILLAR_Z_SIZE, const float MIN_X_RANGE, const float MIN_Y_RANGE,
    const float MIN_Z_RANGE, const int NUM_BOX_CORNERS)
    : NUM_THREADS_(NUM_THREADS),
      MAX_NUM_PILLARS_(MAX_NUM_PILLARS),
      MAX_NUM_POINTS_PER_PILLAR_(MAX_POINTS_PER_PILLAR),
      NUM_INDS_FOR_SCAN_(NUM_INDS_FOR_SCAN),
      GRID_X_SIZE_(GRID_X_SIZE),
      GRID_Y_SIZE_(GRID_Y_SIZE),
      GRID_Z_SIZE_(GRID_Z_SIZE),
      PILLAR_X_SIZE_(PILLAR_X_SIZE),
      PILLAR_Y_SIZE_(PILLAR_Y_SIZE),
      PILLAR_Z_SIZE_(PILLAR_Z_SIZE),
      MIN_X_RANGE_(MIN_X_RANGE),
      MIN_Y_RANGE_(MIN_Y_RANGE),
      MIN_Z_RANGE_(MIN_Z_RANGE),
      NUM_BOX_CORNERS_(NUM_BOX_CORNERS) {
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_x_in_coors_),
                       GRID_Y_SIZE_ * GRID_X_SIZE_ *
                           MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_y_in_coors_),
                       GRID_Y_SIZE_ * GRID_X_SIZE_ *
                           MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_z_in_coors_),
                       GRID_Y_SIZE_ * GRID_X_SIZE_ *
                           MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_i_in_coors_),
                       GRID_Y_SIZE_ * GRID_X_SIZE_ *
                           MAX_NUM_POINTS_PER_PILLAR_ * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_count_histo_),
                       GRID_Y_SIZE_ * GRID_X_SIZE_ * sizeof(int)));

  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_counter_), sizeof(int)));
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_pillar_count_), sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_x_coors_for_sub_),
                       MAX_NUM_PILLARS_ * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_y_coors_for_sub_),
                       MAX_NUM_PILLARS_ * sizeof(float)));
}

PreprocessPointsCuda::~PreprocessPointsCuda() {
  GPU_CHECK(cudaFree(dev_pillar_x_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_y_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_z_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_i_in_coors_));
  GPU_CHECK(cudaFree(dev_pillar_count_histo_));

  GPU_CHECK(cudaFree(dev_counter_));
  GPU_CHECK(cudaFree(dev_pillar_count_));
  GPU_CHECK(cudaFree(dev_x_coors_for_sub_));
  GPU_CHECK(cudaFree(dev_y_coors_for_sub_));
}

void PreprocessPointsCuda::doPreprocessPointsCuda(
    const float* dev_points, const int in_num_points, int* dev_x_coors,
    int* dev_y_coors, float* dev_num_points_per_pillar, float* dev_pillar_x,
    float* dev_pillar_y, float* dev_pillar_z, float* dev_pillar_i,
    float* dev_x_coors_for_sub_shaped, float* dev_y_coors_for_sub_shaped,
    float* dev_pillar_feature_mask, int* dev_sparse_pillar_map,
    int* host_pillar_count) {
  GPU_CHECK(cudaMemset(dev_pillar_count_histo_, 0,
                       GRID_Y_SIZE_ * GRID_X_SIZE_ * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_counter_, 0, sizeof(int)));
  GPU_CHECK(cudaMemset(dev_pillar_count_, 0, sizeof(int)));

  int num_block = DIVUP(in_num_points, NUM_THREADS_);
  make_pillar_histo_kernel<<<num_block, NUM_THREADS_>>>(
      dev_points, dev_pillar_x_in_coors_, dev_pillar_y_in_coors_,
      dev_pillar_z_in_coors_, dev_pillar_i_in_coors_, dev_pillar_count_histo_,
      in_num_points, MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_, GRID_Y_SIZE_,
      GRID_Z_SIZE_, MIN_X_RANGE_, MIN_Y_RANGE_, MIN_Z_RANGE_, PILLAR_X_SIZE_,
      PILLAR_Y_SIZE_, PILLAR_Z_SIZE_, NUM_BOX_CORNERS_);

  make_pillar_index_kernel<<<GRID_X_SIZE_, GRID_Y_SIZE_>>>(
      dev_pillar_count_histo_, dev_counter_, dev_pillar_count_, dev_x_coors,
      dev_y_coors, dev_x_coors_for_sub_, dev_y_coors_for_sub_,
      dev_num_points_per_pillar, dev_sparse_pillar_map, MAX_NUM_PILLARS_,
      MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_, PILLAR_X_SIZE_, PILLAR_Y_SIZE_,
      NUM_INDS_FOR_SCAN_);

  GPU_CHECK(cudaMemcpy(host_pillar_count, dev_pillar_count_, sizeof(int),
                       cudaMemcpyDeviceToHost));
  make_pillar_feature_kernel<<<host_pillar_count[0],
                               MAX_NUM_POINTS_PER_PILLAR_>>>(
      dev_pillar_x_in_coors_, dev_pillar_y_in_coors_, dev_pillar_z_in_coors_,
      dev_pillar_i_in_coors_, dev_pillar_x, dev_pillar_y, dev_pillar_z,
      dev_pillar_i, dev_x_coors, dev_y_coors, dev_num_points_per_pillar,
      MAX_NUM_POINTS_PER_PILLAR_, GRID_X_SIZE_);

  make_extra_network_input_kernel<<<MAX_NUM_PILLARS_,
                                    MAX_NUM_POINTS_PER_PILLAR_>>>(
      dev_x_coors_for_sub_, dev_y_coors_for_sub_, dev_num_points_per_pillar,
      dev_x_coors_for_sub_shaped, dev_y_coors_for_sub_shaped,
      dev_pillar_feature_mask, MAX_NUM_POINTS_PER_PILLAR_);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
