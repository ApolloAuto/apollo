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

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace algorithm {

// @brief: define of the conditional clustering class
template <typename PointT>
class ConditionClustering {
 public:
  using IndicesClusters = std::vector<base::PointIndices>;

  ConditionClustering() = default;
  explicit ConditionClustering(bool extract_removed_clusters = false)
      : min_cluster_size_(1),
        max_cluster_size_(std::numeric_limits<int>::max()),
        extract_removed_clusters_(extract_removed_clusters),
        small_clusters_(new IndicesClusters),
        large_clusters_(new IndicesClusters) {}

  virtual ~ConditionClustering() = default;

  inline void set_cloud(
      typename std::shared_ptr<base::PointCloud<PointT>> cloud) {
    cloud_ = cloud;
  }

  inline void set_candidate_param(void* param) { candidate_param_ = param; }

  inline void set_candidate_function(
      int (*candidate_function)(const PointT&, void*, std::vector<int>*)) {
    candidate_function_ = candidate_function;
  }

  inline void set_condition_param(void* param) { condition_param_ = param; }

  inline void set_condition_function(bool (*condition_function)(const PointT&,
                                                                const PointT&,
                                                                void* param)) {
    condition_function_ = condition_function;
  }

  inline void set_min_cluster_size(size_t size) { min_cluster_size_ = size; }

  inline void set_max_cluster_size(size_t size) { max_cluster_size_ = size; }

  // main interface of ConditionClustering class to segment.
  void Segment(IndicesClusters* xy_clusters);

 private:
  typename std::shared_ptr<base::PointCloud<PointT>> cloud_;
  size_t min_cluster_size_ = 0;
  size_t max_cluster_size_ = 0;
  bool extract_removed_clusters_ = true;
  std::shared_ptr<IndicesClusters> small_clusters_;
  std::shared_ptr<IndicesClusters> large_clusters_;
  bool (*condition_function_)(const PointT&, const PointT&,
                              void* param) = nullptr;
  int (*candidate_function_)(const PointT&, void*,
                             std::vector<int>* nn_indices_ptr) = nullptr;
  void* candidate_param_ = nullptr;
  void* condition_param_ = nullptr;
};  // class ConditionClustering

template <typename PointT>
void ConditionClustering<PointT>::Segment(IndicesClusters* xy_clusters) {
  std::vector<int> nn_indices;
  nn_indices.reserve(200);
  std::vector<bool> processed(cloud_->size(), false);
  // Process all points indexed by indices_
  for (std::size_t iii = 0; iii < cloud_->size(); ++iii) {
    if (processed[iii]) {
      continue;
    }
    // Set up a new growing cluster
    std::vector<int> current_cluster;
    current_cluster.reserve(200);
    std::size_t cii = 0;
    current_cluster.push_back(static_cast<int>(iii));
    processed[iii] = true;
    // Process the current cluster
    // (it can be growing in size as it is being processed)
    while (cii < current_cluster.size()) {
      nn_indices.clear();
      if (candidate_function_(cloud_->at(current_cluster[cii]),
                              candidate_param_, &nn_indices) < 1) {
        cii++;
        continue;
      }
      for (std::size_t nii = 1; nii < nn_indices.size(); ++nii) {
        if (nn_indices[nii] < 0 ||
            nn_indices[nii] >= static_cast<int>(processed.size()) ||
            processed[nn_indices[nii]]) {
          continue;
        }
        if (condition_function_(cloud_->at(current_cluster[cii]),
                                cloud_->at(nn_indices[nii]),
                                condition_param_)) {
          current_cluster.push_back(nn_indices[nii]);
          processed[nn_indices[nii]] = true;
        }
      }
      cii++;
    }

    if (extract_removed_clusters_ ||
        (current_cluster.size() >= min_cluster_size_ &&
         current_cluster.size() <= max_cluster_size_)) {
      base::PointIndices pi;
      pi.indices.resize(current_cluster.size());
      for (int ii = 0; ii < static_cast<int>(current_cluster.size()); ++ii) {
        pi.indices[ii] = current_cluster[ii];
      }

      if (extract_removed_clusters_ &&
          current_cluster.size() < min_cluster_size_) {
        small_clusters_->push_back(pi);
      } else if (extract_removed_clusters_ &&
                 current_cluster.size() > max_cluster_size_) {
        large_clusters_->push_back(pi);
      } else {
        xy_clusters->push_back(pi);
      }
    }
  }
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
