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

#include <string>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_cluster.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_label_image.h"

namespace apollo {
namespace perception {
namespace lidar {

class SppClusterList {
 public:
  SppClusterList() { clusters_.reserve(kDefaultReserveSize); }
  // @brief: initialize cluster list
  // @param [in]: size
  // @param [in]: sensor_name
  void Init(size_t size, const std::string& sensor_name = "velodyne64");
  // @brief: reset cluster list
  inline void Reset() { clusters_.clear(); }
  // @brief: resize cluster list
  // @param [in]: size
  void resize(size_t size);
  // @brief: add an 3d point sample
  // @param [in]: cluster id
  // @param [in]: 3d point
  // @param [in]: point height above ground
  // @param [in]: point id
  void AddPointSample(size_t cluster_id, const base::PointF& point,
                      float height, uint32_t point_id);
  // @brief: get clusters data
  // @return: clusters
  inline std::vector<SppClusterPtr>& clusters() { return clusters_; }
  // @brief: get clusters data, const version
  // @return: clusters
  inline const std::vector<SppClusterPtr>& clusters() const {
    return clusters_;
  }
  // @brief: get clusters size
  // @return: cluster size
  inline size_t size() const { return clusters_.size(); }
  // @brief: get cluster pointer
  // @return: cluster pointer
  inline SppClusterPtr& operator[](int id) { return clusters_[id]; }
  // @brief: get cluster pointer, const version
  // @return: cluster pointer
  inline const SppClusterPtr& operator[](int id) const { return clusters_[id]; }
  // @brief: merge elements from another cluster list and clear it
  // @param [in]: another cluster list
  void Merge(SppClusterList* rhs);
  // @brief: cut along height axis to split cluster
  // @param [in]: max gap value in two connected component
  // @param [in]: start cluster id
  // @return: cut number in all clusters
  size_t HeightCut(float max_gap, size_t start_id = 0);
  // @brief: compute height and split clusters
  // @param [in]: cluster id
  // @param [in]: max gap value in two connected component
  // @return: true if successfully split
  bool ComputeHeightAndSplitCluster(size_t id, float max_gap);
  // @brief: remove empty cluster from clusters
  void RemoveEmptyClusters();
  // @brief: remove cluster given id
  void EraseCluster(size_t id);
  // @brief: assign clusters from label image
  SppClusterList& operator=(const SppLabelImage& rhs) {
    clusters_ = rhs.GetClusters();
    return *this;
  }

 private:
  static const size_t kDefaultReserveSize = 500;

 private:
  std::vector<SppClusterPtr> clusters_;
  std::string sensor_name_;
  DISALLOW_COPY_AND_ASSIGN(SppClusterList);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
