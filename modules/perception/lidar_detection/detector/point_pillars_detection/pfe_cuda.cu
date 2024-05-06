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

#include "modules/perception/lidar_detection/detector/point_pillars_detection/pfe_cuda.h"

namespace apollo {
namespace perception {
namespace lidar {

__global__ void gather_point_feature_kernel(
    float* dev_pillar_point_feature, float* dev_num_points_per_pillar,
    float* dev_pillar_coors, float* dev_pfe_gather_feature,
    int max_num_points_per_pillar, int num_point_feature,
    int num_gather_point_feature, float pillar_x_size, float pillar_y_size,
    float min_x_range, float min_y_range) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  float point_mean[3];
  for (int point_id = 0; point_id < dev_num_points_per_pillar[tid];
       ++point_id) {
    for (int i = 0; i < 3; ++i) {
      int feature_id = i + point_id * num_point_feature +
                       tid * max_num_points_per_pillar * num_point_feature;
      point_mean[i] += dev_pillar_point_feature[feature_id];
    }
  }
  for (int i = 0; i < 3; ++i) {
    point_mean[i] = point_mean[i] / dev_num_points_per_pillar[tid];
  }

  float x_offset = pillar_x_size / 2 + min_x_range;
  float y_offset = pillar_y_size / 2 + min_y_range;
  for (int point_id = 0; point_id < dev_num_points_per_pillar[tid];
       ++point_id) {
    int point_head_id = point_id * num_point_feature +
                        tid * max_num_points_per_pillar * num_point_feature;
    int pfe_gather_head_id = point_id * num_gather_point_feature +
        tid * max_num_points_per_pillar * num_gather_point_feature;
    float point_x = dev_pillar_point_feature[point_head_id];
    float point_y = dev_pillar_point_feature[point_head_id + 1];
    dev_pfe_gather_feature[pfe_gather_head_id] =
        sqrt(point_x * point_x + point_y * point_y);
    for (int i = 2; i < num_point_feature; ++i) {
      dev_pfe_gather_feature[pfe_gather_head_id + i - 1] =
          dev_pillar_point_feature[point_head_id + i];
    }
    for (int i = 4; i < 7; ++i) {
      dev_pfe_gather_feature[pfe_gather_head_id + i] =
          dev_pillar_point_feature[point_head_id + i - 4] - point_mean[i - 4];
    }
    dev_pfe_gather_feature[pfe_gather_head_id + 7] =
        dev_pillar_point_feature[point_head_id] -
        (dev_pillar_coors[tid * 4 + 3] * pillar_x_size + x_offset);
    dev_pfe_gather_feature[pfe_gather_head_id + 8] =
        dev_pillar_point_feature[point_head_id + 1] -
        (dev_pillar_coors[tid * 4 + 2] * pillar_y_size + y_offset);
  }
}

PfeCuda::PfeCuda(int max_num_pillars, int max_num_points_per_pillar,
                 int num_point_feature, int num_gather_point_feature,
                 float pillar_x_size, float pillar_y_size, float min_x_range,
                 float min_y_range, int num_threads)
    : max_num_pillars_(max_num_pillars),
      max_num_points_per_pillar_(max_num_points_per_pillar),
      num_point_feature_(num_point_feature),
      num_gather_point_feature_(num_gather_point_feature),
      pillar_x_size_(pillar_x_size),
      pillar_y_size_(pillar_y_size),
      min_x_range_(min_x_range),
      min_y_range_(min_y_range),
      num_threads_(num_threads) {}

void PfeCuda::GatherPointFeature(float* dev_pillar_point_feature,
                                 float* dev_num_points_per_pillar,
                                 float* dev_pillar_coors,
                                 float* dev_pfe_gather_feature) {
  const int num_blocks = DIVUP(max_num_pillars_, num_threads_);
  gather_point_feature_kernel<<<num_blocks, num_threads_>>>(
      dev_pillar_point_feature, dev_num_points_per_pillar, dev_pillar_coors,
      dev_pfe_gather_feature, max_num_points_per_pillar_, num_point_feature_,
      num_gather_point_feature_, pillar_x_size_, pillar_y_size_, min_x_range_,
      min_y_range_);
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
