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

#include "modules/perception/lidar_detection/detector/point_pillars_detection/common.h"

namespace apollo {
namespace perception {
namespace lidar {

class PfeCuda {
 public:
  /**
   * @brief Construct a new Pfe Cuda object
   * 
   * @param max_num_pillars max number of pillars
   * @param max_num_points_per_pillar max number of points per pillar
   * @param num_point_feature number of features of points
   * @param num_gather_point_feature number of gathered feature of points
   * @param pillar_x_size size of pillar along x axis
   * @param pillar_y_size size of pillar along y axis
   * @param min_x_range min x range of point cloud
   * @param min_y_range min y range of point cloud
   * @param num_threads number of threads
   */
  PfeCuda(int max_num_pillars, int max_num_points_per_pillar,
          int num_point_feature, int num_gather_point_feature,
          float pillar_x_size, float pillar_y_size, float min_x_range,
          float min_y_range, int num_threads);

  /**
   * @brief Destroy the Pfe Cuda object
   * 
   */
  ~PfeCuda() = default;

  /**
   * @brief gather the point feature
   * 
   * @param dev_pillar_point_feature pillar point feature
   * @param dev_num_points_per_pillar number of points per pillar
   * @param dev_pillar_coors pillar coordinates
   * @param dev_pfe_gather_feature gathered point feature
   */
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
