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
    const int num_point_feature, const int grid_x_size, const int grid_y_size,
    const int grid_z_size, const float pillar_x_size, const float pillar_y_size,
    const float pillar_z_size, const float min_x_range, const float min_y_range,
    const float min_z_range, const int num_inds_for_scan)
    : max_num_pillars_(max_num_pillars),
      max_num_points_per_pillar_(max_points_per_pillar),
      num_point_feature_(num_point_feature),
      grid_x_size_(grid_x_size),
      grid_y_size_(grid_y_size),
      grid_z_size_(grid_z_size),
      pillar_x_size_(pillar_x_size),
      pillar_y_size_(pillar_y_size),
      pillar_z_size_(pillar_z_size),
      min_x_range_(min_x_range),
      min_y_range_(min_y_range),
      min_z_range_(min_z_range),
      num_inds_for_scan_(num_inds_for_scan) {}

void PreprocessPoints::InitializeVariables(int* coor_to_pillaridx,
                                           float* sparse_pillar_map,
                                           float* pillar_point_feature,
                                           float* pillar_coors) {
  for (int i = 0; i < grid_y_size_; ++i) {
    for (int j = 0; j < grid_x_size_; ++j) {
      coor_to_pillaridx[i * grid_x_size_ + j] = -1;
    }
  }

  for (int i = 0; i < num_inds_for_scan_; ++i) {
    for (int j = 0; j < num_inds_for_scan_; ++j) {
      sparse_pillar_map[i * num_inds_for_scan_ + j] = 0;
    }
  }

  for (int i = 0;
       i < max_num_pillars_ * max_num_points_per_pillar_ * num_point_feature_;
       ++i) {
    pillar_point_feature[i] = 0;
  }

  for (int i = 0; i < max_num_pillars_ * 4; ++i) {
    pillar_coors[i] = 0;
  }
}

void PreprocessPoints::Preprocess(const float* in_points_array,
                                  int in_num_points, int* x_coors, int* y_coors,
                                  float* num_points_per_pillar,
                                  float* pillar_point_feature,
                                  float* pillar_coors, float* sparse_pillar_map,
                                  int* host_pillar_count) {
  int pillar_count = 0;
  // init variables
  int* coor_to_pillaridx = new int[grid_y_size_ * grid_x_size_];
  InitializeVariables(coor_to_pillaridx, sparse_pillar_map,
                      pillar_point_feature, pillar_coors);
  for (int i = 0; i < in_num_points; ++i) {
    int x_coor = std::floor(
        (in_points_array[i * num_point_feature_ + 0] - min_x_range_) /
        pillar_x_size_);
    int y_coor = std::floor(
        (in_points_array[i * num_point_feature_ + 1] - min_y_range_) /
        pillar_y_size_);
    int z_coor = std::floor(
        (in_points_array[i * num_point_feature_ + 2] - min_z_range_) /
        pillar_z_size_);
    if (x_coor < 0 || x_coor >= grid_x_size_ || y_coor < 0 ||
        y_coor >= grid_y_size_ || z_coor < 0 || z_coor >= grid_z_size_) {
      continue;
    }
    // reverse index
    int pillar_index = coor_to_pillaridx[y_coor * grid_x_size_ + x_coor];
    if (pillar_index == -1) {
      pillar_index = pillar_count;
      if (pillar_count >= max_num_pillars_) {
        break;
      }
      pillar_count += 1;
      coor_to_pillaridx[y_coor * grid_x_size_ + x_coor] = pillar_index;

      y_coors[pillar_index] = std::floor(y_coor);
      x_coors[pillar_index] = std::floor(x_coor);

      sparse_pillar_map[y_coor * num_inds_for_scan_ + x_coor] = 1;
    }
    int num = num_points_per_pillar[pillar_index];
    if (num < max_num_points_per_pillar_) {
      for (int j = 0; j < num_point_feature_; ++j) {
        pillar_point_feature[pillar_index * max_num_points_per_pillar_ *
                                 num_point_feature_ +
                             num * num_point_feature_ + j] =
            in_points_array[i * num_point_feature_ + j];
      }
      num_points_per_pillar[pillar_index] += 1;
    }
  }

  for (int i = 0; i < max_num_pillars_; ++i) {
    float x = 0;
    float y = 0;
    if (i < pillar_count) {
      x = static_cast<float>(x_coors[i]);
      y = static_cast<float>(y_coors[i]);
    }
    pillar_coors[i * 4 + 0] = 0;  // batch idx, but currently it is useless
    pillar_coors[i * 4 + 1] = 0;  // z
    pillar_coors[i * 4 + 2] = y;
    pillar_coors[i * 4 + 3] = x;
  }
  host_pillar_count[0] = pillar_count;

  delete[] coor_to_pillaridx;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
