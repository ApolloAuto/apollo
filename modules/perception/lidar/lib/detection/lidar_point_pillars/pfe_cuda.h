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

#pragma once

#include "modules/perception/lidar/lib/detection/lidar_point_pillars/common.h"

namespace apollo {
namespace perception {
namespace lidar {

class PfeCuda {
 public:
  PfeCuda(int max_num_pillars, int max_num_points_per_pillar,
          int num_point_feature, int num_gather_point_feature,
          float pillar_x_size, float pillar_y_size, float min_x_range,
          float min_y_range, int num_threads);
  ~PfeCuda() = default;

  void GatherPointFeature(float* dev_pillar_point_feature,
                          float* dev_num_points_per_pillar,
                          float* dev_pillar_coors,
                          float* dev_pfe_gather_feature);

 private:
  int max_num_pillars_;
  int max_num_points_per_pillar_;
  int num_point_feature_;
  int num_gather_point_feature_;
  float pillar_x_size_;
  float pillar_y_size_;
  float min_x_range_;
  float min_y_range_;
  int num_threads_;
};
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
