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

#pragma once

#include <map>
#include <vector>

#include "pcl/filters/boost.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_types.h"

#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"

namespace apollo {
namespace localization {
namespace ndt {

/**@brief Simple structure to hold a centroid, covarince and the number of
 * points in a leaf. */
struct Leaf {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Leaf()
      : nr_points_(0),
        mean_(Eigen::Vector3d::Zero()),
        icov_(Eigen::Matrix3d::Zero()) {}

  /**@brief Get the number of points contained by this voxel. */
  int GetPointCount() const { return nr_points_; }

  /**@brief Get the voxel centroid. */
  Eigen::Vector3d GetMean() const { return mean_; }

  /**@brief Get the inverse of the voxel covariance. */
  Eigen::Matrix3d GetInverseCov() const { return icov_; }

  /**@brief Number of points contained by voxel. */
  int nr_points_;
  /**@brief 3D voxel centroid. */
  Eigen::Vector3d mean_;
  /**@brief Inverse of voxel covariance matrix. */
  Eigen::Matrix3d icov_;
};
/**@brief Pointer to VoxelGridCovariance leaf structure */
typedef Leaf *LeafPtr;
/**@brief Const pointer to VoxelGridCovariance leaf structure */
typedef const Leaf *LeafConstPtr;

/**@brief A searchable voxel structure containing the mean and covariance of the
 * data. */
template <typename PointT>
class VoxelGridCovariance {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef boost::shared_ptr<PointCloud> PointCloudPtr;
  typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
  PointCloudConstPtr input_;

  Eigen::Vector4f leaf_size_;
  Eigen::Array4f inverse_leaf_size_;

  Eigen::Vector4i min_b_;
  Eigen::Vector4i max_b_;

  Eigen::Vector4i div_b_;
  Eigen::Vector4i divb_mul_;

 public:
  /**@brief Constructor. */
  VoxelGridCovariance()
      : min_points_per_voxel_(6),
        leaves_(),
        voxel_centroids_(),
        voxel_centroids_leaf_indices_(),
        kdtree_() {
    leaf_size_.setZero();
    min_b_.setZero();
    max_b_.setZero();
  }

  /**@brief Provide a pointer to the input dataset. */
  void SetInputCloud(const PointCloudConstPtr &cloud) { input_ = cloud; }

  /**@brief Set the minimum number of points required for a cell to be used
   * (must be 3 or greater for covariance calculation). */
  inline void SetMinPointPerVoxel(int min_points_per_voxel) {
    if (min_points_per_voxel > 2) {
      min_points_per_voxel_ = min_points_per_voxel;
    } else {
      AWARN << "Covariance calculation requires at least 3 "
            << "points, setting Min Point per Voxel to 3 ";
      min_points_per_voxel_ = 3;
    }
  }

  /**@brief Get the minimum number of points required for a cell to be used.*/
  inline int GetMinPointPerVoxel() { return min_points_per_voxel_; }

  /**@brief Initializes voxel structure. */
  inline void filter(const std::vector<Leaf> &cell_leaf,
                     bool searchable = true) {
    voxel_centroids_ = PointCloudPtr(new PointCloud);
    SetMap(cell_leaf, voxel_centroids_);
    if (voxel_centroids_->size() > 0) {
      kdtree_.setInputCloud(voxel_centroids_);
    }
  }

  void SetMap(const std::vector<Leaf> &map_leaves, PointCloudPtr output);

  /**@brief Get the voxel containing point p. */
  inline LeafConstPtr GetLeaf(int index) {
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(index);
    if (leaf_iter == leaves_.end()) {
      return nullptr;
    }
    LeafConstPtr ret(&(leaf_iter->second));
    return ret;
  }

  /**@brief Get the voxel containing point p.
   * \param[in] p the point to get the leaf structure at
   * \return const pointer to leaf structure
   */
  inline LeafConstPtr GetLeaf(PointT *p) {
    // Generate index associated with p
    int ijk0 = static_cast<int>((p->x - map_left_top_corner_(0)) *
                                inverse_leaf_size_[0]) -
               min_b_[0];
    int ijk1 = static_cast<int>((p->y - map_left_top_corner_(1)) *
                                inverse_leaf_size_[1]) -
               min_b_[1];
    int ijk2 = static_cast<int>((p->z - map_left_top_corner_(2)) *
                                inverse_leaf_size_[2]) -
               min_b_[2];

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter == leaves_.end()) {
      return nullptr;
    }
    // If such a leaf exists return the pointer to the leaf structure
    LeafConstPtr ret(&(leaf_iter->second));
    return ret;
  }

  /**@brief Get the voxel containing point p.* \return const pointer to leaf
   * structure */
  inline LeafConstPtr GetLeaf(Eigen::Vector3f *p) {
    // Generate index associated with p
    int ijk0 = static_cast<int>((p->x() - map_left_top_corner_(0)) *
                                inverse_leaf_size_[0]) -
               min_b_[0];
    int ijk1 = static_cast<int>((p->y() - map_left_top_corner_(1)) *
                                inverse_leaf_size_[1]) -
               min_b_[1];
    int ijk2 = static_cast<int>((p->z() - map_left_top_corner_(2)) *
                                inverse_leaf_size_[2]) -
               min_b_[2];

    // Compute the centroid leaf index
    int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

    // Find leaf associated with index
    typename std::map<size_t, Leaf>::iterator leaf_iter = leaves_.find(idx);
    if (leaf_iter == leaves_.end()) {
      return nullptr;
    }
    // If such a leaf exists return the pointer to the leaf structure
    LeafConstPtr ret(&(leaf_iter->second));
    return ret;
  }

  /**@brief Get the leaf structure map. */
  inline const std::map<size_t, Leaf> &GetLeaves() { return leaves_; }

  /**@brief Get a pointcloud containing the voxel centroids. */
  inline PointCloudPtr GetCentroids() { return voxel_centroids_; }

  /**@brief Search for all the nearest occupied voxels of the query point in a
   * given radius. */
  int RadiusSearch(const PointT &point, double radius,
                   std::vector<LeafConstPtr> *k_leaves,
                   std::vector<float> *k_sqr_distances,
                   unsigned int max_nn = 0);

  void GetDisplayCloud(pcl::PointCloud<pcl::PointXYZ> *cell_cloud);

  inline void SetMapLeftTopCorner(const Eigen::Vector3d &left_top_corner) {
    map_left_top_corner_ = left_top_corner;
  }

  inline void SetVoxelGridResolution(float lx, float ly, float lz) {
    leaf_size_[0] = lx;
    leaf_size_[1] = ly;
    leaf_size_[2] = lz;
    // Avoid division errors
    if (leaf_size_[3] == 0) {
      leaf_size_[3] = 1;
    }  // Use multiplications instead of divisions
    inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
  }

 protected:
  /**@brief Minimum points contained with in a voxel to allow it to be usable.
   */
  int min_points_per_voxel_;

  /**@brief Voxel structure containing all leaf nodes (includes voxels with
   * less than a sufficient number of points). */
  std::map<size_t, Leaf> leaves_;

  /**@brief Point cloud containing centroids of voxels containing at least
   * minimum number of points. */
  PointCloudPtr voxel_centroids_;

  /**@brief Indices of leaf structurs associated with each point. */
  std::vector<int> voxel_centroids_leaf_indices_;

  /**@brief KdTree generated using voxel_centroids_ (used for searching). */
  pcl::KdTreeFLANN<PointT> kdtree_;

  /**@brief Left top corner. */
  Eigen::Vector3d map_left_top_corner_;
};

}  // namespace ndt
}  // namespace localization
}  // namespace apollo

#include "modules/localization/ndt/ndt_locator/ndt_voxel_grid_covariance.hpp"
