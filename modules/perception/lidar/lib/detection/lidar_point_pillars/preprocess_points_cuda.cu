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
    const int max_points_per_pillar, const int grid_x_size,
    const int grid_y_size, const int grid_z_size, const float min_x_range,
    const float min_y_range, const float min_z_range, const float pillar_x_size,
    const float pillar_y_size, const float pillar_z_size,
    const int num_box_corners) {
  int th_i = threadIdx.x + blockIdx.x * blockDim.x;
  if (th_i >= num_points) {
    return;
  }
  int y_coor = floor((dev_points[th_i * num_box_corners + 1] - min_y_range) /
                     pillar_y_size);
  int x_coor = floor((dev_points[th_i * num_box_corners + 0] - min_x_range) /
                     pillar_x_size);
  int z_coor = floor((dev_points[th_i * num_box_corners + 2] - min_z_range) /
                     pillar_z_size);

  if (x_coor >= 0 && x_coor < grid_x_size && y_coor >= 0 &&
      y_coor < grid_y_size && z_coor >= 0 && z_coor < grid_z_size) {
    int count =
        atomicAdd(&pillar_count_histo[y_coor * grid_x_size + x_coor], 1);
    if (count < max_points_per_pillar) {
      int ind = y_coor * grid_x_size * max_points_per_pillar +
                x_coor * max_points_per_pillar + count;
      dev_pillar_x_in_coors[ind] = dev_points[th_i * num_box_corners + 0];
      dev_pillar_y_in_coors[ind] = dev_points[th_i * num_box_corners + 1];
      dev_pillar_z_in_coors[ind] = dev_points[th_i * num_box_corners + 2];
      dev_pillar_i_in_coors[ind] = dev_points[th_i * num_box_corners + 3];
    }
  }
}

__global__ void make_pillar_index_kernel(
    int* dev_pillar_count_histo, int* dev_counter, int* dev_pillar_count,
    int* dev_x_coors, int* dev_y_coors, float* dev_x_coors_for_sub,
    float* dev_y_coors_for_sub, float* dev_num_points_per_pillar,
    int* dev_sparse_pillar_map, const int max_pillars,
    const int max_points_per_pillar, const int grid_x_size,
    const float pillar_x_size, const float pillar_y_size,
    const int num_inds_for_scan) {
  int x = blockIdx.x;
  int y = threadIdx.x;
  int num_points_at_this_pillar = dev_pillar_count_histo[y * grid_x_size + x];
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
    dev_x_coors_for_sub[count] = x * pillar_x_size + 0.1f;
    dev_y_coors_for_sub[count] = y * pillar_y_size + -39.9f;
    dev_sparse_pillar_map[y * num_inds_for_scan + x] = 1;
  }
}

__global__ void make_pillar_feature_kernel(
    float* dev_pillar_x_in_coors, float* dev_pillar_y_in_coors,
    float* dev_pillar_z_in_coors, float* dev_pillar_i_in_coors,
    float* dev_pillar_x, float* dev_pillar_y, float* dev_pillar_z,
    float* dev_pillar_i, int* dev_x_coors, int* dev_y_coors,
    float* dev_num_points_per_pillar, const int max_points,
    const int grid_x_size) {
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
      y_ind * grid_x_size * max_points + x_ind * max_points + ith_point;
  dev_pillar_x[pillar_ind] = dev_pillar_x_in_coors[coors_ind];
  dev_pillar_y[pillar_ind] = dev_pillar_y_in_coors[coors_ind];
  dev_pillar_z[pillar_ind] = dev_pillar_z_in_coors[coors_ind];
  dev_pillar_i[pillar_ind] = dev_pillar_i_in_coors[coors_ind];
}

__global__ void make_extra_network_input_kernel(
    float* dev_x_coors_for_sub, float* dev_y_coors_for_sub,
    float* dev_num_points_per_pillar, float* dev_x_coors_for_sub_shaped,
    float* dev_y_coors_for_sub_shaped, float* dev_pillar_feature_mask,
    const int max_num_points_per_pillar) {
  int ith_pillar = blockIdx.x;
  int ith_point = threadIdx.x;
  float x = dev_x_coors_for_sub[ith_pillar];
  float y = dev_y_coors_for_sub[ith_pillar];
  int num_points_for_a_pillar = dev_num_points_per_pillar[ith_pillar];
  int ind = ith_pillar * max_num_points_per_pillar + ith_point;
  dev_x_coors_for_sub_shaped[ind] = x;
  dev_y_coors_for_sub_shaped[ind] = y;

  if (ith_point < num_points_for_a_pillar) {
    dev_pillar_feature_mask[ind] = 1.0;
  } else {
    dev_pillar_feature_mask[ind] = 0.0;
  }
}

PreprocessPointsCuda::PreprocessPointsCuda(
    const int num_threads, const int max_num_pillars,
    const int max_points_per_pillar, const int num_inds_for_scan,
    const int grid_x_size, const int grid_y_size, const int grid_z_size,
    const float pillar_x_size, const float pillar_y_size,
    const float pillar_z_size, const float min_x_range, const float min_y_range,
    const float min_z_range, const int num_box_corners)
    : kNumThreads(num_threads),
      kMaxNumPillars(max_num_pillars),
      kMaxNumPointsPerPillar(max_points_per_pillar),
      kNumIndsForScan(num_inds_for_scan),
      kGridXSize(grid_x_size),
      kGridYSize(grid_y_size),
      kGridZSize(grid_z_size),
      kPillarXSize(pillar_x_size),
      kPillarYSize(pillar_y_size),
      kPillarZSize(pillar_z_size),
      kMinXRange(min_x_range),
      kMinYRange(min_y_range),
      kMinZRange(min_z_range),
      kNumBoxCorners(num_box_corners) {
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_x_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_y_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_z_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_i_in_coors_),
                       kGridYSize * kGridXSize *
                           kMaxNumPointsPerPillar * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_pillar_count_histo_),
                       kGridYSize * kGridXSize * sizeof(int)));

  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_counter_), sizeof(int)));
  GPU_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&dev_pillar_count_), sizeof(int)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_x_coors_for_sub_),
                       kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_y_coors_for_sub_),
                       kMaxNumPillars * sizeof(float)));
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

void PreprocessPointsCuda::DoPreprocessPointsCuda(
    const float* dev_points, const int in_num_points, int* dev_x_coors,
    int* dev_y_coors, float* dev_num_points_per_pillar, float* dev_pillar_x,
    float* dev_pillar_y, float* dev_pillar_z, float* dev_pillar_i,
    float* dev_x_coors_for_sub_shaped, float* dev_y_coors_for_sub_shaped,
    float* dev_pillar_feature_mask, int* dev_sparse_pillar_map,
    int* host_pillar_count) {
  GPU_CHECK(cudaMemset(dev_pillar_count_histo_, 0,
                       kGridYSize * kGridXSize * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_counter_, 0, sizeof(int)));
  GPU_CHECK(cudaMemset(dev_pillar_count_, 0, sizeof(int)));

  int num_block = DIVUP(in_num_points, kNumThreads);
  make_pillar_histo_kernel<<<num_block, kNumThreads>>>(
      dev_points, dev_pillar_x_in_coors_, dev_pillar_y_in_coors_,
      dev_pillar_z_in_coors_, dev_pillar_i_in_coors_, dev_pillar_count_histo_,
      in_num_points, kMaxNumPointsPerPillar, kGridXSize, kGridYSize,
      kGridZSize, kMinXRange, kMinYRange, kMinZRange, kPillarXSize,
      kPillarYSize, kPillarZSize, kNumBoxCorners);

  make_pillar_index_kernel<<<kGridXSize, kGridYSize>>>(
      dev_pillar_count_histo_, dev_counter_, dev_pillar_count_, dev_x_coors,
      dev_y_coors, dev_x_coors_for_sub_, dev_y_coors_for_sub_,
      dev_num_points_per_pillar, dev_sparse_pillar_map, kMaxNumPillars,
      kMaxNumPointsPerPillar, kGridXSize, kPillarXSize, kPillarYSize,
      kNumIndsForScan);

  GPU_CHECK(cudaMemcpy(host_pillar_count, dev_pillar_count_, sizeof(int),
                       cudaMemcpyDeviceToHost));
  make_pillar_feature_kernel<<<host_pillar_count[0],
                               kMaxNumPointsPerPillar>>>(
      dev_pillar_x_in_coors_, dev_pillar_y_in_coors_, dev_pillar_z_in_coors_,
      dev_pillar_i_in_coors_, dev_pillar_x, dev_pillar_y, dev_pillar_z,
      dev_pillar_i, dev_x_coors, dev_y_coors, dev_num_points_per_pillar,
      kMaxNumPointsPerPillar, kGridXSize);

  make_extra_network_input_kernel<<<kMaxNumPillars,
                                    kMaxNumPointsPerPillar>>>(
      dev_x_coors_for_sub_, dev_y_coors_for_sub_, dev_num_points_per_pillar,
      dev_x_coors_for_sub_shaped, dev_y_coors_for_sub_shaped,
      dev_pillar_feature_mask, kMaxNumPointsPerPillar);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
