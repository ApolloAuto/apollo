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
 * @file anchor_mask_cuda.h
 * @brief Make anchor mask for filtering output
 * @author Kosuke Murakami
 * @date 2019/02/26
 */

#pragma once

namespace apollo {
namespace perception {
namespace lidar {

class AnchorMaskCuda {
 private:
  const int NUM_INDS_FOR_SCAN_;
  const int NUM_ANCHOR_X_INDS_;
  const int NUM_ANCHOR_Y_INDS_;
  const int NUM_ANCHOR_R_INDS_;
  const float MIN_X_RANGE_;
  const float MIN_Y_RANGE_;
  const float PILLAR_X_SIZE_;
  const float PILLAR_Y_SIZE_;
  const int GRID_X_SIZE_;
  const int GRID_Y_SIZE_;

 public:
  /**
   * @brief Constructor
   * @param[in] NUM_INDS_FOR_SCAN Number of indexes for scan(cumsum)
   * @param[in] NUM_ANCHOR_X_INDS Number of x-indexes for anchors
   * @param[in] NUM_ANCHOR_Y_INDS Number of y-indexes for anchors
   * @param[in] NUM_ANCHOR_R_INDS Number of rotation-indexes for anchors
   * @param[in] MIN_X_RANGE Minimum x value for pointcloud
   * @param[in] MIN_Y_RANGE Minimum y value for pointcloud
   * @param[in] PILLAR_X_SIZE Size of x-dimension for a pillar
   * @param[in] PILLAR_Y_SIZE Size of y-dimension for a pillar
   * @param[in] GRID_X_SIZE Number of pillars in x-coordinate
   * @param[in] GRID_Y_SIZE Number of pillars in y-coordinate
   * @details Captital variables never change after the compile
   */
  AnchorMaskCuda(const int NUM_INDS_FOR_SCAN, const int NUM_ANCHOR_X_INDS,
                 const int NUM_ANCHOR_Y_INDS, const int NUM_ANCHOR_R_INDS,
                 const float MIN_X_RANGE, const float MIN_Y_RANGE,
                 const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
                 const int GRID_X_SIZE, const int GRID_Y_SIZE);

  /**
   * @brief call cuda code for making anchor mask
   * @param[in] dev_sparse_pillar_map Grid map representation for pillar
   * occupancy
   * @param[in] dev_cumsum_along_x Array for storing cumsum-ed
   * dev_sparse_pillar_map values
   * @param[in] dev_cumsum_along_y Array for storing cumsum-ed
   * dev_cumsum_along_y values
   * @param[in] dev_box_anchors_min_x Array for storing min x value for each
   * anchor
   * @param[in] dev_box_anchors_min_y Array for storing min y value for each
   * anchor
   * @param[in] dev_box_anchors_max_x Array for storing max x value for each
   * anchor
   * @param[in] dev_box_anchors_max_y Array for storing max y value for each
   * anchor
   * @param[in] dev_box_anchors_max_y Array for storing max y value for each
   * anchor
   * @param[out] dev_anchor_mask Anchor mask for filtering the network output
   * @details dev_* means device memory. Make a mask for filtering pillar
   * occupancy area
   */
  void doAnchorMaskCuda(int* dev_sparse_pillar_map, int* dev_cumsum_along_x,
                        int* dev_cumsum_along_y,
                        const float* dev_box_anchors_min_x,
                        const float* dev_box_anchors_min_y,
                        const float* dev_box_anchors_max_x,
                        const float* dev_box_anchors_max_y,
                        int* dev_anchor_mask);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
