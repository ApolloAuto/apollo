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
  const int MAX_NUM_PILLARS_;
  const int MAX_NUM_POINTS_PER_PILLAR_;
  const int GRID_X_SIZE_;
  const int GRID_Y_SIZE_;
  const int GRID_Z_SIZE_;
  const float PILLAR_X_SIZE_;
  const float PILLAR_Y_SIZE_;
  const float PILLAR_Z_SIZE_;
  const float MIN_X_RANGE_;
  const float MIN_Y_RANGE_;
  const float MIN_Z_RANGE_;
  const int NUM_INDS_FOR_SCAN_;
  const int NUM_BOX_CORNERS_;

 public:
  /**
   * @brief Constructor
   * @param[in] MAX_NUM_PILLARS Maximum number of pillars
   * @param[in] MAX_POINTS_PER_PILLAR Maximum number of points per pillar
   * @param[in] GRID_X_SIZE Number of pillars in x-coordinate
   * @param[in] GRID_Y_SIZE Number of pillars in y-coordinate
   * @param[in] GRID_Z_SIZE Number of pillars in z-coordinate
   * @param[in] PILLAR_X_SIZE Size of x-dimension for a pillar
   * @param[in] PILLAR_Y_SIZE Size of y-dimension for a pillar
   * @param[in] PILLAR_Z_SIZE Size of z-dimension for a pillar
   * @param[in] MIN_X_RANGE Minimum x value for pointcloud
   * @param[in] MIN_Y_RANGE Minimum y value for pointcloud
   * @param[in] MIN_Z_RANGE Minimum z value for pointcloud
   * @param[in] NUM_INDS_FOR_SCAN Number of indexes for scan(cumsum)
   * @param[in] NUM_BOX_CORNERS Number of box's corner
   * @details Captital variables never change after the compile
   */
  PreprocessPoints(const int MAX_NUM_PILLARS, const int MAX_POINTS_PER_PILLAR,
                   const int GRID_X_SIZE, const int GRID_Y_SIZE,
                   const int GRID_Z_SIZE, const float PILLAR_X_SIZE,
                   const float PILLAR_Y_SIZE, const float PILLAR_Z_SIZE,
                   const float MIN_X_RANGE, const float MIN_Y_RANGE,
                   const float MIN_Z_RANGE, const int NUM_INDS_FOR_SCAN,
                   const int NUM_BOX_CORNERS);

  /**
   * @brief CPU preprocessing for input pointcloud
   * @param[in] in_points_array Pointcloud array
   * @param[in] in_num_points The number of points
   * @param[in] x_coors X-coordinate indexes for corresponding pillars
   * @param[in] y_coors Y-coordinate indexes for corresponding pillars
   * @param[in] num_points_per_pillar Number of points in corresponding pillars
   * @param[in] pillar_x X-coordinate values for points in each pillar
   * @param[in] pillar_y Y-coordinate values for points in each pillar
   * @param[in] pillar_z Z-coordinate values for points in each pillar
   * @param[in] pillar_i Intensity values for points in each pillar
   * @param[in] x_coors_for_sub_shaped Array for x substraction in the network
   * @param[in] y_coors_for_sub_shaped Array for y substraction in the network
   * @param[in] pillar_feature_mask Mask to make pillars' feature zero where no
   * points in the pillars
   * @param[in] sparse_pillar_map Grid map representation for pillar-occupancy
   * @param[in] host_pillar_count The numnber of valid pillars for the input
   * pointcloud
   * @details Convert pointcloud to pillar representation
   */
  void preprocess(const float* in_points_array, int in_num_points, int* x_coors,
                  int* y_coors, float* num_points_per_pillar, float* pillar_x,
                  float* pillar_y, float* pillar_z, float* pillar_i,
                  float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped,
                  float* pillar_feature_mask, float* sparse_pillar_map,
                  int* host_pillar_count);

  /**
   * @brief Initializing variables for preprocessing
   * @param[in] coor_to_pillaridx Map for converting one set of coordinate to a
   * pillar
   * @param[in] sparse_pillar_map Grid map representation for pillar-occupancy
   * @param[in] pillar_x X-coordinate values for points in each pillar
   * @param[in] pillar_y Y-coordinate values for points in each pillar
   * @param[in] pillar_z Z-coordinate values for points in each pillar
   * @param[in] pillar_i Intensity values for points in each pillar
   * @param[in] x_coors_for_sub_shaped Array for x substraction in the network
   * @param[in] y_coors_for_sub_shaped Array for y substraction in the network
   * @details Initializeing input arguments with certain values
   */
  void initializeVariables(int* coor_to_pillaridx, float* sparse_pillar_map,
                           float* pillar_x, float* pillar_y, float* pillar_z,
                           float* pillar_i, float* x_coors_for_sub_shaped,
                           float* y_coors_for_sub_shaped);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
