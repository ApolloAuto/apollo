/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
namespace radar4d {

class AnchorMaskCuda {
 private:
  const int num_threads_;
  const int num_inds_for_scan_;
  const int num_anchor_;
  const float min_x_range_;
  const float min_y_range_;
  const float pillar_x_size_;
  const float pillar_y_size_;
  const int grid_x_size_;
  const int grid_y_size_;

 public:
  /**
   * @brief Constructor
   * @param[in] num_threads Number of threads per block
   * @param[in] num_inds_for_scan Number of indexes for scan(cumsum)
   * @param[in] num_anchor Number of anchors in total
   * @param[in] min_x_range Minimum x value for point cloud
   * @param[in] min_y_range Minimum y value for point cloud
   * @param[in] pillar_x_size Size of x-dimension for a pillar
   * @param[in] pillar_y_size Size of y-dimension for a pillar
   * @param[in] grid_x_size Number of pillars in x-coordinate
   * @param[in] grid_y_size Number of pillars in y-coordinate
   * @details Captital variables never change after the compile
   */
  AnchorMaskCuda(const int num_threads, const int num_inds_for_scan,
                 const int num_anchor, const float min_x_range,
                 const float min_y_range, const float pillar_x_size,
                 const float pillar_y_size, const int grid_x_size,
                 const int grid_y_size);

  /**
   * @brief call cuda code for making anchor mask
   * @param[in] dev_sparse_pillar_map
   *   Grid map representation for pillar occupancy
   * @param[in] dev_cumsum_along_x
   *   Array for storing cumsum-ed dev_sparse_pillar_map values
   * @param[in] dev_cumsum_along_y
   *   Array for storing cumsum-ed dev_cumsum_along_y values
   * @param[in] dev_box_anchors_min_x
   *   Array for storing min x value for each anchor
   * @param[in] dev_box_anchors_min_y
   *   Array for storing min y value for each anchor
   * @param[in] dev_box_anchors_max_x
   *   Array for storing max x value for each anchor
   * @param[in] dev_box_anchors_max_y
   *   Array for storing max y value for each anchor
   * @param[in] dev_box_anchors_max_y
   *   Array for storing max y value for each anchor
   * @param[out] dev_anchor_mask Anchor mask for filtering the network output
   * @details dev_* means device memory. Make a mask for filtering pillar
   * occupancy area
   */
  void DoAnchorMaskCuda(int* dev_sparse_pillar_map, int* dev_cumsum_along_x,
                        int* dev_cumsum_along_y,
                        const float* dev_box_anchors_min_x,
                        const float* dev_box_anchors_min_y,
                        const float* dev_box_anchors_max_x,
                        const float* dev_box_anchors_max_y,
                        int* dev_anchor_mask);
};

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
