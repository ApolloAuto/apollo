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

#include "Eigen/Dense"

namespace apollo {
namespace perception {
namespace common {

template <typename T>
class SecureMat {
 public:
  SecureMat() : height_(0), width_(0) { Reserve(max_height_, max_width_); }

  size_t height() { return height_; }
  size_t width() { return width_; }

  /* @brief: reserve memory of SecureMat
   * @params[IN] reserve_height: height of reserve memory
   * @params[IN] reserve_width: width of reserve memory
   * @return nothing */
  void Reserve(const size_t reserve_height, const size_t reserve_width) {
    max_height_ = (reserve_height > max_height_) ? reserve_height : max_height_;
    max_width_ = (reserve_width > max_width_) ? reserve_width : max_width_;
    mat_.resize(max_height_, max_width_);
  }

  /* @brief: resize memory of SecureMat
   * @params[IN] resize_height: height of resize memory
   * @params[IN] resize_width: width of resize memory
   * @return nothing */
  void Resize(const size_t resize_height, const size_t resize_width) {
    height_ = resize_height;
    width_ = resize_width;
    if (resize_height <= max_height_ && resize_width <= max_width_) {
      return;
    }
    max_height_ = (resize_height > max_height_) ? resize_height : max_height_;
    max_width_ = (resize_width > max_width_) ? resize_width : max_width_;
    mat_.resize(max_height_, max_width_);
  }

  void ToString(std::ostream* out_stream) {
    std::ostream& stream = *out_stream;
    for (size_t row = 0; row < height_; ++row) {
      for (size_t col = 0; col < width_; ++col) {
        stream << mat_(row, col) << "\t";
      }
      stream << "\n";
    }
  }

  inline const T& operator()(const size_t row, const size_t col) const {
    return mat_(row, col);
  }

  inline T& operator()(const size_t row, const size_t col) {
    return mat_(row, col);
  }

 private:
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_;
  size_t max_height_ = 1000;
  size_t max_width_ = 1000;
  size_t height_ = 0;
  size_t width_ = 0;
};  // class SecureMat

}  // namespace common
}  // namespace perception
}  // namespace apollo
