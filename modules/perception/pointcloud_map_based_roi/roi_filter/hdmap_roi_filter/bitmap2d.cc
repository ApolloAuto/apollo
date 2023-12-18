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

#include "modules/perception/pointcloud_map_based_roi/roi_filter/hdmap_roi_filter/bitmap2d.h"

#include "modules/perception/common/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

static constexpr uint64_t kZeroLast = static_cast<uint64_t>(-1) - 1;

// static
inline Bitmap2D::DirectionMajor Bitmap2D::OppositeDirection(
    const Bitmap2D::DirectionMajor dir_major) {
  return static_cast<DirectionMajor>(static_cast<int>(dir_major) ^ 1);
}

inline bool Bitmap2D::CheckBit(const size_t loc, const uint64_t block) const {
  return block & (static_cast<uint64_t>(1) << loc);
}

inline void Bitmap2D::SetBit(const size_t loc, uint64_t* block) {
  (*block) |= (static_cast<uint64_t>(1) << loc);
}

inline void Bitmap2D::ResetBit(const size_t loc, uint64_t* block) {
  (*block) &= (~(static_cast<uint64_t>(1) << loc));
}

inline void Bitmap2D::SetTailBits(const size_t tail_num, uint64_t* block) {
  (*block) |= (~(static_cast<uint64_t>(-1) << tail_num));
}

inline void Bitmap2D::ResetTailBits(const size_t tail_num, uint64_t* block) {
  (*block) &= (static_cast<uint64_t>(-1) << tail_num);
}

inline void Bitmap2D::SetHeadBits(const size_t tail_num, uint64_t* block) {
  (*block) |= (static_cast<uint64_t>(-1) << tail_num);
}

inline void Bitmap2D::ResetHeadBits(const size_t tail_num, uint64_t* block) {
  (*block) &= ~(static_cast<uint64_t>(-1) << tail_num);
}

inline void Bitmap2D::SetRangeBits(const size_t head, const size_t tail,
                                   uint64_t* block) {
  // note: it not work when head = 64
  (*block) |= ((static_cast<uint64_t>(-1) << tail) & (~(kZeroLast << head)));
}

inline void Bitmap2D::ResetRangeBits(const size_t head, const size_t tail,
                                     uint64_t* block) {
  (*block) &= ((~(static_cast<uint64_t>(-1) << tail)) | (kZeroLast << head));
}

void Bitmap2D::Init(const Eigen::Vector2d& min_range,
                    const Eigen::Vector2d& max_range,
                    const Eigen::Vector2d& cell_size) {
  CHECK_GT(cell_size.x(), 0);
  CHECK_GT(cell_size.y(), 0);
  CHECK_GT(max_range.x(), min_range.x() + cell_size.x());
  CHECK_GT(max_range.y(), min_range.y() + cell_size.y());

  min_range_ = min_range;
  max_range_ = max_range;
  cell_size_ = cell_size;

  dims_ =
      ((max_range_ - min_range_).array() / cell_size_.array()).cast<size_t>();
  dims_ += Vec2ui(1, 1);
  map_size_[0] = dims_[0];
  map_size_[1] = (dims_[1] >> 6) + 1;
  bitmap_.resize(map_size_[0] * map_size_[1], 0);
}

void Bitmap2D::SetUp(const DirectionMajor dir_major) {
  dir_major_ = dir_major;
  op_dir_major_ = OppositeDirection(dir_major);
  memset(bitmap_.data(), 0, sizeof(bitmap_[0]) * bitmap_.size());
}

// point to bitmap index;
inline Bitmap2D::Vec3ui Bitmap2D::RealToBitmap(const Eigen::Vector2d& p) const {
  Vec2ui pix = ((p - min_range_).array() / cell_size_.array()).cast<size_t>();
  Vec2ui major_pix(pix[dir_major()], pix[op_dir_major()]);
  Vec3ui bit_p;
  bit_p.x() = major_pix.x();
  bit_p.y() = major_pix.y() >> 6;
  bit_p.z() = major_pix.y() & 63;
  return bit_p;
}

// calc index by bitmap index;
inline int Bitmap2D::Index(const Bitmap2D::Vec3ui& p) const {
  return static_cast<int>(p.x() * map_size_[1] + p.y());
}

// range check
bool Bitmap2D::IsExists(const Eigen::Vector2d& p) const {
  return p.x() >= min_range_.x() && p.x() < max_range_.x() &&
         p.y() >= min_range_.y() && p.y() < max_range_.y();
}

// check
bool Bitmap2D::Check(const Eigen::Vector2d& p) const {
  const Vec3ui bit_p = RealToBitmap(p);
  const int idx = Index(bit_p);
  return CheckBit(bit_p.z(), bitmap_[idx]);
}

// set and reset
void Bitmap2D::Set(const Eigen::Vector2d& p) {
  const Vec3ui bit_p = RealToBitmap(p);
  const int idx = Index(bit_p);
  return SetBit(bit_p.z(), &bitmap_[idx]);
}

void Bitmap2D::Reset(const Eigen::Vector2d& p) {
  const Vec3ui bit_p = RealToBitmap(p);
  const int idx = Index(bit_p);
  return ResetBit(bit_p.z(), &bitmap_[idx]);
}

void Bitmap2D::Set(const double x, const double min_y, const double max_y) {
  Eigen::Vector2d real_left, real_right;
  real_left[op_dir_major()] = min_y;
  real_right[op_dir_major()] = max_y;
  real_left[dir_major()] = real_right[dir_major()] = x;
  const Vec3ui left_bit_p = RealToBitmap(real_left);
  const Vec3ui right_bit_p = RealToBitmap(real_right);
  if (left_bit_p.y() == right_bit_p.y()) {
    const int idx = Index(left_bit_p);
    SetRangeBits(right_bit_p.z(), left_bit_p.z(), &bitmap_[idx]);
    return;
  }
  const size_t left_idx = Index(left_bit_p);
  const size_t right_idx = Index(right_bit_p);
  SetHeadBits(left_bit_p.z(), &bitmap_[left_idx]);
  SetTailBits(right_bit_p.z(), &bitmap_[right_idx]);
  for (size_t i = left_idx + 1; i < right_idx; ++i) {
    bitmap_[i] = static_cast<uint64_t>(-1);
  }
}

void Bitmap2D::Reset(const double x, const double min_y, const double max_y) {
  Eigen::Vector2d real_left, real_right;
  real_left[op_dir_major()] = min_y;
  real_right[op_dir_major()] = max_y;
  real_left[dir_major()] = real_right[dir_major()] = x;
  const Vec3ui left_bit_p = RealToBitmap(real_left);
  const Vec3ui right_bit_p = RealToBitmap(real_right);
  if (left_bit_p.y() == right_bit_p.y()) {
    const int idx = Index(left_bit_p);
    ResetRangeBits(right_bit_p.z() + 1, left_bit_p.z(), &bitmap_[idx]);
    return;
  }
  // set first block and last block
  const size_t left_idx = Index(left_bit_p);
  const size_t right_idx = Index(right_bit_p);
  ResetHeadBits(left_bit_p.z(), &bitmap_[left_idx]);
  ResetTailBits(right_bit_p.z(), &bitmap_[right_idx]);
  for (size_t i = left_idx + 1; i < right_idx; ++i) {
    bitmap_[i] = static_cast<uint64_t>(0);
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
