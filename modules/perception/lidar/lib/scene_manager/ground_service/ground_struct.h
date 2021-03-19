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

#include "Eigen/Dense"

namespace apollo {
namespace perception {
namespace lidar {

struct GroundNode {
  Eigen::Vector4f params;
  float confidence = 0.f;
};

class GroundGrid {
 public:
  GroundGrid() = default;
  ~GroundGrid() = default;
  GroundGrid(const GroundGrid& rhs) { *this = rhs; }
  GroundGrid& operator=(const GroundGrid& rhs) {
    rows_ = rhs.rows_;
    cols_ = rhs.cols_;
    size_ = rhs.size_;
    if (data_.size() != rhs.data_.size()) {
      data_.resize(rhs.data_.size());
    }
    memcpy(data_.data(), rhs.data_.data(), sizeof(GroundNode) * size_);
    nodes_.resize(rows_);
    for (int32_t i = 0; i < rows_; ++i) {
      nodes_[i] = data_.data() + i * cols_;
    }
    return *this;
  }
  // @brief: initialize ground grid
  // @param [in]: rows and cols of grid
  void Init(int32_t rows, int32_t cols) {
    data_.clear();
    nodes_.clear();
    rows_ = rows;
    cols_ = cols;
    size_ = rows_ * cols_;
    data_.resize(size_);
    nodes_.resize(rows_);
    for (int i = 0; i < rows_; ++i) {
      nodes_[i] = data_.data() + i * cols_;
    }
    Reset();
  }
  // @brief: reset grid
  void Reset() { memset(data_.data(), 0, sizeof(GroundNode) * data_.size()); }
  // @brief: whether a coordinate is in grid
  // @param [in]: row id r, col id c
  // @return: status
  bool IsInGrid(int32_t r, int32_t c) {
    return (r >= 0 && r < rows_ && c >= 0 && c < cols_);
  }
  // @brief: row pointer accessor
  // @param [in]: row id
  // @return: row pointer
  GroundNode* operator[](int32_t r) { return nodes_[r]; }
  // @brief: row pointer accessor(const version)
  // @param [in]: row id
  // @return: row pointer
  const GroundNode* operator[](int32_t r) const { return nodes_[r]; }
  // @brief: data pointer accessor
  // @return: data pointer
  GroundNode* DataPtr() { return data_.data(); }
  // @brief: data pointer accessor(const version)
  // @return: data pointer
  const GroundNode* DataPtr() const { return data_.data(); }
  // @brief: grid rows accessor
  // @return: rows
  int32_t rows() const { return rows_; }
  // @brief: grid cols accessor
  // @return: cols
  int32_t cols() const { return cols_; }
  // @brief: grid size assessor
  // @return: size
  int32_t size() const { return size_; }

 protected:
  std::vector<GroundNode> data_;
  std::vector<GroundNode*> nodes_;
  int32_t rows_ = 0;
  int32_t cols_ = 0;
  int32_t size_ = 0;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
