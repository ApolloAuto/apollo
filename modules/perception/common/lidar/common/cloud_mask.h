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

#include <vector>

#include "modules/perception/common/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace lidar {

class CloudMask {
 public:
  CloudMask() = default;
  ~CloudMask() = default;

  // @brief: set mask size and initial value
  // @param [in]: size, mask size
  // @param [in]: value, initial value
  inline void Set(size_t size, int init_value) {
    mask_.assign(size, init_value);
  }

  // @brief: fill mask with given value
  // @param [in]: value
  inline void Fill(int value) { std::fill(mask_.begin(), mask_.end(), value); }

  // @brief: get mask data
  // @return: mask data
  inline const std::vector<int>& mask() const { return mask_; }

  // @brief: overloading [] to get mask element
  // @param [in]: id
  // @return: mask element, at id
  inline int& operator[](int id) { return mask_[id]; }

  // @brief: overloading [] to get mask element, const version
  // @param [in]: id
  // @return: mask element, at id
  inline const int& operator[](int id) const { return mask_[id]; }

  // @brief: get size of mask data
  // @return: size
  inline size_t size() const { return mask_.size(); }

  // @brief: clear mask data
  inline void clear() { mask_.clear(); }

  // @brief: get valid (positive) indices count
  // @return: count
  size_t ValidIndicesCount() const;

  // @brief: get valid (positive) indices count
  // @param [in]: indices
  // @return: count
  template <typename IntegerType>
  size_t ValidIndicesCount(const std::vector<IntegerType>& indices) const;

  // @brief: get valid (positive) point cloud from mask
  // @param [in]: source point cloud
  // @param [out]: target point cloud with valid points
  void GetValidCloud(
      const base::AttributePointCloud<base::PointF>& source_cloud,
      base::AttributePointCloud<base::PointF>* target_cloud) const;

  // @brief; get valid indices from mask
  // @param [in]: indices vector
  void GetValidIndices(base::PointIndices* indices);

  // @brief: flip the mask data, positive to zero and zero to one
  // @brief: note, flip twice is not guaranteed to recover the original mask
  void Flip();

  // @brief: add point indices, base::PointIndices version
  // @param [in]: indices
  // @param [in]: value
  void AddIndices(const base::PointIndices& indices, int value = 1);

  // @brief: add point indices, std::vector<IntegerType> version
  // @param [in]: indices
  // @param [in]: value
  template <typename IntegerType>
  void AddIndices(const std::vector<IntegerType>& indices, int value = 1);

  // @brief: add point indices of indices
  // @param [in]: indices
  // @param [in]: indices of indices (first param)
  // @param [in]: value
  void AddIndicesOfIndices(const base::PointIndices& indices,
                           const base::PointIndices& indices_of_indices,
                           int value = 1);

  // @brief: remove point indices, base::PointIndices version
  // @param [in]: indices
  void RemoveIndices(const base::PointIndices& indices);

  // @brief: remove point indices, std::vector<IntegerType> version
  // @param [in]: indices
  template <typename IntegerType>
  void RemoveIndices(const std::vector<IntegerType>& indices);

  // @brief: remove point indices of indices
  // @param [in]: indices
  // @param [in]: indices of indices (first param)
  void RemoveIndicesOfIndices(const base::PointIndices& indices,
                              const base::PointIndices& indices_of_indices);

  // @brief: get and store valid (positive) value to another mask
  // @param [out]: target mask
  void GetValidMask(CloudMask* rhs) const;

  // @brief: reset source value in mask to target value
  // @param [in]: source value
  // @param [in]: target value
  void ResetValue(int source_value, int target_value);

  std::vector<int> GetValidIndices();

 private:
  // @brief mask data
  std::vector<int> mask_;

  // @brief point indices buffer
  mutable std::vector<int> indices_;
};

template <typename IntegerType>
void CloudMask::AddIndices(const std::vector<IntegerType>& indices, int value) {
  for (auto& id : indices) {
    mask_[id] = value;
  }
}

template <typename IntegerType>
void CloudMask::RemoveIndices(const std::vector<IntegerType>& indices) {
  for (auto& id : indices) {
    mask_[id] = 0;
  }
}

template <typename IntegerType>
size_t CloudMask::ValidIndicesCount(
    const std::vector<IntegerType>& indices) const {
  size_t count = 0;
  for (const auto& id : indices) {
    if (mask_[id]) {
      ++count;
    }
  }
  return count;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
