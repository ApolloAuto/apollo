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
#include <cmath>
#include <iostream>

// headers in local files
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/preprocess_points.h"

namespace apollo {
namespace perception {
namespace lidar {

PreprocessPoints::PreprocessPoints(
    const int max_num_pillars, const int max_points_per_pillar,
    const int grid_x_size, const int grid_y_size, const int grid_z_size,
    const float pillar_x_size, const float pillar_y_size,
    const float pillar_z_size, const float min_x_range, const float min_y_range,
    const float min_z_range, const int num_inds_for_scan,
    const int num_box_corners)
    : kMaxNumPillars(max_num_pillars),
      kMaxNumPointsPerPillar(max_points_per_pillar),
      kGridXSize(grid_x_size),
      kGridYSize(grid_y_size),
      kGridZSize(grid_z_size),
      kPillarXSize(pillar_x_size),
      kPillarYSize(pillar_y_size),
      kPillarZSize(pillar_z_size),
      kMinXRange(min_x_range),
      kMinYRange(min_y_range),
      kMinZRange(min_z_range),
      kNumIndsForScan(num_inds_for_scan),
      kNumBoxCorners(num_box_corners) {}

void PreprocessPoints::InitializeVariables(int* coor_to_pillaridx,
                                           float* sparse_pillar_map,
                                           float* pillar_x, float* pillar_y,
                                           float* pillar_z, float* pillar_i,
                                           float* x_coors_for_sub_shaped,
                                           float* y_coors_for_sub_shaped) {
  for (int i = 0; i < kGridYSize; i++) {
    for (int j = 0; j < kGridXSize; j++) {
      coor_to_pillaridx[i * kGridXSize + j] = -1;
    }
  }

  for (int i = 0; i < kNumIndsForScan; i++) {
    for (int j = 0; j < kNumIndsForScan; j++) {
      sparse_pillar_map[i * kNumIndsForScan + j] = 0;
    }
  }

  for (int i = 0; i < kMaxNumPillars * kMaxNumPointsPerPillar; i++) {
    pillar_x[i] = 0;
    pillar_y[i] = 0;
    pillar_z[i] = 0;
    pillar_i[i] = 0;
    x_coors_for_sub_shaped[i] = 0;
    y_coors_for_sub_shaped[i] = 0;
  }
}

void PreprocessPoints::Preprocess(
    const float* in_points_array, int in_num_points, int* x_coors, int* y_coors,
    float* num_points_per_pillar, float* pillar_x, float* pillar_y,
    float* pillar_z, float* pillar_i, float* x_coors_for_sub_shaped,
    float* y_coors_for_sub_shaped, float* pillar_feature_mask,
    float* sparse_pillar_map, int* host_pillar_count) {
  int pillar_count = 0;
  float x_coors_for_sub[kMaxNumPillars];
  float y_coors_for_sub[kMaxNumPillars];
  x_coors_for_sub[0] = 0;
  y_coors_for_sub[0] = 0;
  // init variables
  int coor_to_pillaridx[kGridYSize * kGridXSize];
  InitializeVariables(coor_to_pillaridx, sparse_pillar_map, pillar_x, pillar_y,
                      pillar_z, pillar_i, x_coors_for_sub_shaped,
                      y_coors_for_sub_shaped);
  for (int i = 0; i < in_num_points; i++) {
    int x_coor =
        std::floor((in_points_array[i * kNumBoxCorners + 0] - kMinXRange) /
                   kPillarXSize);
    int y_coor =
        std::floor((in_points_array[i * kNumBoxCorners + 1] - kMinYRange) /
                   kPillarYSize);
    int z_coor =
        std::floor((in_points_array[i * kNumBoxCorners + 2] - kMinZRange) /
                   kPillarZSize);
    if (x_coor < 0 || x_coor >= kGridXSize || y_coor < 0 ||
        y_coor >= kGridYSize || z_coor < 0 || z_coor >= kGridZSize) {
      continue;
    }
    // reverse index
    int pillar_index = coor_to_pillaridx[y_coor * kGridXSize + x_coor];
    if (pillar_index == -1) {
      pillar_index = pillar_count;
      if (pillar_count >= kMaxNumPillars) {
        break;
      }
      pillar_count += 1;
      coor_to_pillaridx[y_coor * kGridXSize + x_coor] = pillar_index;

      y_coors[pillar_index] = std::floor(y_coor);
      x_coors[pillar_index] = std::floor(x_coor);

      // float y_offset = kPillarYSize/ 2 + kMinYRange;
      // float x_offset = kPillarXSize/ 2 + kMinXRange;
      // TODO(...): Need to be modified after proper training code
      // Will be modified in ver 1.1
      y_coors_for_sub[pillar_index] =
          std::floor(y_coor) * kPillarYSize + -39.9f;
      x_coors_for_sub[pillar_index] =
          std::floor(x_coor) * kPillarXSize + 0.1f;

      sparse_pillar_map[y_coor * kNumIndsForScan + x_coor] = 1;
    }
    int num = num_points_per_pillar[pillar_index];
    if (num < kMaxNumPointsPerPillar) {
      pillar_x[pillar_index * kMaxNumPointsPerPillar + num] =
          in_points_array[i * kNumBoxCorners + 0];
      pillar_y[pillar_index * kMaxNumPointsPerPillar + num] =
          in_points_array[i * kNumBoxCorners + 1];
      pillar_z[pillar_index * kMaxNumPointsPerPillar + num] =
          in_points_array[i * kNumBoxCorners + 2];
      pillar_i[pillar_index * kMaxNumPointsPerPillar + num] =
          in_points_array[i * kNumBoxCorners + 3];
      num_points_per_pillar[pillar_index] += 1;
    }
  }

  for (int i = 0; i < kMaxNumPillars; i++) {
    float x = x_coors_for_sub[i];
    float y = y_coors_for_sub[i];
    int num_points_for_a_pillar = num_points_per_pillar[i];
    for (int j = 0; j < kMaxNumPointsPerPillar; j++) {
      x_coors_for_sub_shaped[i * kMaxNumPointsPerPillar + j] = x;
      y_coors_for_sub_shaped[i * kMaxNumPointsPerPillar + j] = y;
      if (j < num_points_for_a_pillar) {
        pillar_feature_mask[i * kMaxNumPointsPerPillar + j] = 1.0f;
      } else {
        pillar_feature_mask[i * kMaxNumPointsPerPillar + j] = 0.0f;
      }
    }
  }
  host_pillar_count[0] = pillar_count;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
