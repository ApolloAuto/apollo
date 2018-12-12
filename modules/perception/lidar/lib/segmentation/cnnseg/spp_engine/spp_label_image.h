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

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include "cyber/common/macros.h"
#include "modules/perception/common/i_lib/core/i_alloc.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/spp_engine/spp_cluster.h"

namespace apollo {
namespace perception {
namespace lidar {

class SppLabelImage {
 public:
  SppLabelImage() { clusters_.reserve(kDefaultReserveSize); }
  ~SppLabelImage() {
    if (labels_) {
      common::IFree2(&labels_);
    }
    if (range_mask_) {
      common::IFree2(&range_mask_);
    }
  }
  // @brief: initialize label image
  // @param [in]: image width
  // @param [in]: image height
  // @param [in]: sensor name
  void Init(size_t width, size_t height,
            const std::string& sensor_name = "velodyne64");
  // @brief: initialize range mask of label image
  // @param [in]: range
  // @param [in]: boundary distance for mask
  void InitRangeMask(float range, float boundary_distance);
  // @brief: transform clusters to label image
  void CollectClusterFromSppLabelImage();
  // @brief: transform label image to clusters
  void ProjectClusterToSppLabelImage();
  // @brief: filter clusters, given confidence map
  // @param [in]: confidence_map of the same size
  // @param [in]: confidence threshold
  void FilterClusters(const float* confidence_map, float threshold);
  // @brief: filter clusters, given confidence and category map
  // @param [in]: confidence_map of the same size
  // @param [in]: category_map of the same size
  // @param [in]: confidence threshold
  // @param [in]: category threshold
  void FilterClusters(const float* confidence_map, const float* category_map,
                      float confidence_threshold, float category_threshold);
  // @brief: calculate class for each cluster, given class map
  // @param [in]: class_map of the same size
  // @param [in]: class number
  void CalculateClusterClass(const float* class_map, size_t class_num);
  // @brief: calculate heading (yaw) for each cluster, given heading map
  // @param [in]: heading_map of the same size
  void CalculateClusterHeading(const float* heading_map);
  // @brief: calculate top_z for each cluster, given top_z map
  // @param [in]: top_z_map of the same size
  void CalculateClusterTopZ(const float* top_z_map);
  // @brief: get label image
  // @return: label image
  uint16_t** GetSppLabelImage() { return labels_; }
  // @brief: get label image, const version
  // @return: const label image
  const uint16_t* const* GetSppLabelImage() const { return labels_; }
  // @brief: get clusters
  // @return: clusters
  inline std::vector<SppClusterPtr>& GetClusters() { return clusters_; }
  // @brief: get clusters, const version
  // @return: const clusters
  inline const std::vector<SppClusterPtr>& GetClusters() const {
    return clusters_;
  }
  // @brief: get label row pointer
  // @param [in]: row id
  // @return: row pointer
  uint16_t* operator[](size_t id) { return labels_[id]; }
  // @brief: get label row pointer, const version
  // @param [in]: row id
  // @return: row pointer
  const uint16_t* operator[](size_t id) const { return labels_[id]; }
  // @brief: get cluster number
  // @return: cluster number
  size_t GetClusterNum() const { return clusters_.size(); }
  // @brief: get cluster given id
  // @param [in]: cluster id
  // @return: cluster pointer
  SppClusterPtr GetCluster(size_t id) { return clusters_[id]; }
  // @brief: get cluster given id, const version
  // @param [in]: cluster id
  // @return: cluster pointer
  SppClusterConstPtr GetCluster(size_t id) const { return clusters_[id]; }
  // @brief: add a pixel to labeled cluster
  // @param [in]: label id
  // @param [in]: pixel x
  // @param [in]: pixel y
  void AddPixelSample(size_t id, uint16_t x, uint16_t y) {
    return AddPixelSample(id, y * static_cast<uint32_t>(width_) + x);
  }
  // @brief: add a pixel to labeled cluster
  // @param [in]: label id
  // @param [in]: pixel id
  void AddPixelSample(size_t id, uint32_t pixel);
  // @brief: clear clusters
  inline void ClearClusters() { clusters_.clear(); }
  // @brief: resize clusters
  // @param [in]: target cluster size
  void ResizeClusters(size_t size);
  // @brief: reset clusters
  // @param [in]: target cluster size
  void ResetClusters(size_t size);

 private:
  // note the correspondence between label and cluster id is
  // label - 1 == cluster id, label zero is reserved for background
  uint16_t** labels_ = nullptr;
  size_t width_ = 0;
  size_t height_ = 0;
  char** range_mask_ = nullptr;
  std::vector<SppClusterPtr> clusters_;
  std::string sensor_name_;

 private:
  static const size_t kDefaultReserveSize = 500;

  FRIEND_TEST(SppClusterTest, spp_cluster_test);
  DISALLOW_COPY_AND_ASSIGN(SppLabelImage);
};

typedef std::shared_ptr<SppLabelImage> SppLabelImagePtr;
typedef std::shared_ptr<const SppLabelImage> SppLabelImageConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
