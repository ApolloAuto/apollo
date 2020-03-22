/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <map>
#include <vector>

#include "Eigen/Cholesky"
#include "Eigen/Dense"
#include "pcl/common/common.h"
#include "pcl/filters/boost.h"

#include "modules/localization/ndt/ndt_locator/ndt_voxel_grid_covariance.h"

namespace apollo {
namespace localization {
namespace ndt {

template <typename PointT>
void VoxelGridCovariance<PointT>::SetMap(const std::vector<Leaf>& map_leaves,
                                         PointCloudPtr output) {
  voxel_centroids_leaf_indices_.clear();

  // Has the input dataset been set already
  if (!input_) {
    AWARN << "No input dataset given. ";
    output->width = output->height = 0;
    output->points.clear();
    return;
  }

  // Copy the header + allocate enough space for points
  output->height = 1;
  output->is_dense = true;
  output->points.clear();

  // Get the minimum and maximum dimensions
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D<PointT>(*input_, min_p, max_p);

  Eigen::Vector4f left_top = Eigen::Vector4f::Zero();
  left_top.block<3, 1>(0, 0) = map_left_top_corner_.cast<float>();
  min_p -= left_top;
  max_p -= left_top;

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int>(min_p[0] * inverse_leaf_size_[0]);
  max_b_[0] = static_cast<int>(max_p[0] * inverse_leaf_size_[0]);
  min_b_[1] = static_cast<int>(min_p[1] * inverse_leaf_size_[1]);
  max_b_[1] = static_cast<int>(max_p[1] * inverse_leaf_size_[1]);
  min_b_[2] = static_cast<int>(min_p[2] * inverse_leaf_size_[2]);
  max_b_[2] = static_cast<int>(max_p[2] * inverse_leaf_size_[2]);

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
  div_b_[3] = 0;

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

  // Clear the leaves
  leaves_.clear();

  output->points.reserve(map_leaves.size());
  voxel_centroids_leaf_indices_.reserve(leaves_.size());

  for (unsigned int i = 0; i < map_leaves.size(); ++i) {
    const Leaf& cell_leaf = map_leaves[i];
    Eigen::Vector3d local_mean = cell_leaf.mean_ - map_left_top_corner_;
    int ijk0 =
        static_cast<int>(local_mean(0) * inverse_leaf_size_[0]) - min_b_[0];
    int ijk1 =
        static_cast<int>(local_mean(1) * inverse_leaf_size_[1]) - min_b_[1];
    int ijk2 =
        static_cast<int>(local_mean(2) * inverse_leaf_size_[2]) - min_b_[2];

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    Leaf& leaf = leaves_[idx];
    leaf = cell_leaf;

    if (cell_leaf.nr_points_ >= min_points_per_voxel_) {
      output->push_back(PointT());
      output->points.back().x = static_cast<float>(leaf.mean_[0]);
      output->points.back().y = static_cast<float>(leaf.mean_[1]);
      output->points.back().z = static_cast<float>(leaf.mean_[2]);
      voxel_centroids_leaf_indices_.push_back(idx);
    }
  }
  output->width = static_cast<uint32_t>(output->points.size());
}

template <typename PointT>
int VoxelGridCovariance<PointT>::RadiusSearch(
    const PointT& point, double radius, std::vector<LeafConstPtr>* k_leaves,
    std::vector<float>* k_sqr_distances, unsigned int max_nn) {
  k_leaves->clear();

  // Find neighbors within radius in the occupied voxel centroid cloud
  std::vector<int> k_indices;
  int k =
      kdtree_.radiusSearch(point, radius, k_indices, *k_sqr_distances, max_nn);

  // Find leaves corresponding to neighbors
  k_leaves->reserve(k);
  for (std::vector<int>::iterator iter = k_indices.begin();
       iter != k_indices.end(); iter++) {
    k_leaves->push_back(&leaves_[voxel_centroids_leaf_indices_[*iter]]);
  }
  return k;
}

template <typename PointT>
void VoxelGridCovariance<PointT>::GetDisplayCloud(
    pcl::PointCloud<pcl::PointXYZ>* cell_cloud) {
  cell_cloud->clear();

  int pnt_per_cell = 100;
  boost::mt19937 rng;
  boost::normal_distribution<> nd(0.0, leaf_size_.head(3).norm());
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<>>
      var_nor(rng, nd);

  Eigen::LLT<Eigen::Matrix3d> llt_of_cov;
  Eigen::Matrix3d cholesky_decomp;
  Eigen::Vector3d cell_mean;
  Eigen::Vector3d rand_point;
  Eigen::Vector3d dist_point;

  // Generate points for each occupied voxel with sufficient points.
  for (typename std::map<size_t, Leaf>::iterator it = leaves_.begin();
       it != leaves_.end(); ++it) {
    Leaf& leaf = it->second;

    if (leaf.nr_points_ >= min_points_per_voxel_) {
      cell_mean = leaf.mean_;
      Eigen::Matrix3d cov = leaf.icov_.inverse();
      llt_of_cov.compute(cov);
      cholesky_decomp = llt_of_cov.matrixL();

      // Random points generated by sampling the normal distribution given
      // by voxel mean and covariance matrix
      for (int i = 0; i < pnt_per_cell; i++) {
        rand_point = Eigen::Vector3d(var_nor(), var_nor(), var_nor());
        dist_point = cell_mean + cholesky_decomp * rand_point;
        cell_cloud->push_back(pcl::PointXYZ(static_cast<float>(dist_point(0)),
                                            static_cast<float>(dist_point(1)),
                                            static_cast<float>(dist_point(2))));
      }
    }
  }
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
