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

/**
 * @file preprocess_points.h
 * @brief CPU version of preprocess points
 * @author Kosuke Murakami
 * @date 2019/02/26
 */

#pragma once

namespace apollo {
namespace perception {
namespace lidar {

class PreprocessPoints {
 private:
  friend class TestClass;
  const int max_num_pillars_;
  const int max_num_points_per_pillar_;
  const int num_point_feature_;
  const int grid_x_size_;
  const int grid_y_size_;
  const int grid_z_size_;
  const float pillar_x_size_;
  const float pillar_y_size_;
  const float pillar_z_size_;
  const float min_x_range_;
  const float min_y_range_;
  const float min_z_range_;
  const int num_inds_for_scan_;

 public:
  /**
   * @brief Constructor
   * @param[in] max_num_pillars Maximum number of pillars
   * @param[in] max_points_per_pillar Maximum number of points per pillar
   * @param[in] num_point_feature Number of features in a point
   * @param[in] grid_x_size Number of pillars in x-coordinate
   * @param[in] grid_y_size Number of pillars in y-coordinate
   * @param[in] grid_z_size Number of pillars in z-coordinate
   * @param[in] pillar_x_size Size of x-dimension for a pillar
   * @param[in] pillar_y_size Size of y-dimension for a pillar
   * @param[in] pillar_z_size Size of z-dimension for a pillar
   * @param[in] min_x_range Minimum x value for pointcloud
   * @param[in] min_y_range Minimum y value for pointcloud
   * @param[in] min_z_range Minimum z value for pointcloud
   * @param[in] num_inds_for_scan Number of indexes for scan(cumsum)
   * @details Captital variables never change after the compile
   */
  PreprocessPoints(const int max_num_pillars, const int max_points_per_pillar,
                   const int num_point_feature, const int grid_x_size,
                   const int grid_y_size, const int grid_z_size,
                   const float pillar_x_size, const float pillar_y_size,
                   const float pillar_z_size, const float min_x_range,
                   const float min_y_range, const float min_z_range,
                   const int num_inds_for_scan);

  /**
   * @brief CPU preprocessing for input pointcloud
   * @param[in] in_points_array Pointcloud array
   * @param[in] in_num_points The number of points
   * @param[in] x_coors X-coordinate indexes for corresponding pillars
   * @param[in] y_coors Y-coordinate indexes for corresponding pillars
   * @param[in] num_points_per_pillar Number of points in corresponding pillars
   * @param[in] pillar_point_feature Values for features of points in each
   * pillar
   * @param[in] pillar_coors Array for coors of pillars
   * @param[in] sparse_pillar_map Grid map representation for pillar-occupancy
   * @param[in] host_pillar_count The numnber of valid pillars for the input
   * pointcloud
   * @details Convert pointcloud to pillar representation
   */
  void Preprocess(const float* in_points_array, int in_num_points, int* x_coors,
                  int* y_coors, float* num_points_per_pillar,
                  float* pillar_point_feature, float* pillar_coors,
                  float* sparse_pillar_map, int* host_pillar_count);

  /**
   * @brief Initializing variables for preprocessing
   * @param[in] coor_to_pillaridx Map for converting one set of coordinate to a
   * pillar
   * @param[in] sparse_pillar_map Grid map representation for pillar-occupancy
   * @param[in] pillar_point_feature Values for features of points in each
   * pillar
   * @param[in] pillar_coors Array for coors of pillars
   * @details Initializeing input arguments with certain values
   */
  void InitializeVariables(int* coor_to_pillaridx, float* sparse_pillar_map,
                           float* pillar_point_feature, float* pillar_coors);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
