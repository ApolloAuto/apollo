/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/bitmap2d.h"

namespace apollo {
namespace perception {

static const uint64_t all_ones = static_cast<uint64_t>(-1);

void Bitmap2D::Set(const double x, const double min_y, const double max_y) {
  size_t x_id =
      static_cast<size_t>((x - min_p_[dir_major_]) / grid_size_[dir_major_]);

  size_t min_y_id = static_cast<size_t>((min_y - min_p_[op_dir_major_]) /
                                        grid_size_[op_dir_major_]);

  size_t max_y_id = static_cast<size_t>((max_y - min_p_[op_dir_major_]) /
                                        grid_size_[op_dir_major_]);

  Set(x_id, min_y_id, max_y_id);
}

void Bitmap2D::Set(const size_t& x_id, const size_t& min_y_id,
                   const size_t& max_y_id) {
  size_t left_block_id = min_y_id >> 6;  // min_y_id / 64
  size_t left_bit_id = min_y_id & 63;    // min_y_id % 64

  size_t right_block_id = max_y_id >> 6;  // max_y_id / 64
  size_t right_bit_id = max_y_id & 63;    // max_y_id % 64

  auto& blocks = bitmap_[x_id];
  if (left_block_id == right_block_id) {
    SetUint64RangeBits(left_bit_id, right_bit_id, &blocks[left_block_id]);
  } else {
    SetUint64HeadBits(left_bit_id, &blocks[left_block_id]);
    SetUint64TailBits(right_bit_id, &blocks[right_block_id]);

    for (size_t i = left_block_id + 1; i < right_block_id; ++i) {
      blocks[i] = all_ones;
    }
  }
}

inline void Bitmap2D::SetUint64RangeBits(const size_t& head, const size_t& tail,
                                         uint64_t* block) {
  *block |= (all_ones >> head) & (~(all_ones >> tail));
}

inline void Bitmap2D::SetUint64HeadBits(const size_t& head, uint64_t* block) {
  *block |= all_ones >> head;
}

inline void Bitmap2D::SetUint64TailBits(const size_t& tail, uint64_t* block) {
  *block |= (~(all_ones >> tail));
}

bool Bitmap2D::IsExist(const Eigen::Vector2d& p) const {
  if (p.x() < min_p_.x() || p.x() >= max_p_.x()) {
    return false;
  }
  if (p.y() < min_p_.y() || p.y() >= max_p_.y()) {
    return false;
  }
  return true;
}

bool Bitmap2D::Check(const Eigen::Vector2d& p) const {
  Eigen::Matrix<size_t, 2, 1> grid_pt =
      ((p - min_p_).array() / grid_size_.array()).cast<size_t>();

  Eigen::Matrix<size_t, 2, 1> major_grid_pt(grid_pt[dir_major_],
                                            grid_pt[op_dir_major_]);

  size_t x_id = major_grid_pt.x();
  size_t block_id = major_grid_pt.y() >> 6;  // major_grid_pt.y() / 64
  size_t bit_id = major_grid_pt.y() & 63;    // major_grid_pt.y() % 64

  const uint64_t& block = bitmap_[x_id][block_id];

  const uint64_t first_one = static_cast<uint64_t>(1) << 63;
  return block & (first_one >> bit_id);
}

Bitmap2D::Bitmap2D(const Eigen::Vector2d& min_p, const Eigen::Vector2d& max_p,
                   const Eigen::Vector2d& grid_size, DirectionMajor dir_major) {
  dir_major_ = dir_major;
  op_dir_major_ = opposite_direction(dir_major);

  min_p_ = min_p;
  max_p_ = max_p;
  grid_size_ = grid_size;
}

void Bitmap2D::BuildMap() {
  Eigen::Matrix<size_t, 2, 1> dims =
      ((max_p_ - min_p_).array() / grid_size_.array()).cast<size_t>();
  size_t rows = dims[dir_major_];
  size_t cols = (dims[op_dir_major_] >> 6) + 1;

  bitmap_ =
      std::vector<std::vector<uint64_t>>(rows, std::vector<uint64_t>(cols, 0));
}

}  // namespace perception
}  // namespace apollo
