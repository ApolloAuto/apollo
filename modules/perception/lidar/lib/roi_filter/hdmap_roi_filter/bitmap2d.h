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

#include <boost/format.hpp>
#include <vector>

#include "Eigen/Core"

namespace apollo {
namespace perception {
namespace lidar {

class Bitmap2D {
 public:
  enum class DirectionMajor { XMAJOR = 0, YMAJOR = 1 };
  typedef Eigen::Matrix<size_t, 2, 1> Vec2ui;
  typedef Eigen::Matrix<size_t, 3, 1> Vec3ui;

  Bitmap2D() = default;
  virtual ~Bitmap2D() = default;

  inline DirectionMajor OppositeDirection(const DirectionMajor dir_major);

  // getter and setter
  const Eigen::Vector2d& min_range() const { return min_range_; }
  const Eigen::Vector2d& max_range() const { return max_range_; }
  const Eigen::Vector2d& cell_size() const { return cell_size_; }
  const Vec2ui& map_size() const { return map_size_; }
  const Vec2ui& dims() const { return dims_; }
  const std::vector<uint64_t>& bitmap() const { return bitmap_; }
  int dir_major() const { return static_cast<int>(dir_major_); }
  int op_dir_major() const { return static_cast<int>(op_dir_major_); }

  void Init(const Eigen::Vector2d& min_range, const Eigen::Vector2d& max_range,
            const Eigen::Vector2d& cell_size);

  void SetUp(const DirectionMajor dir_major);

  // the only function for range check
  bool Empty() const { return bitmap_.empty(); }
  bool IsExists(const Eigen::Vector2d& p) const;

  bool Check(const Eigen::Vector2d& p) const;
  void Set(const Eigen::Vector2d& p);
  void Reset(const Eigen::Vector2d& p);

  // x for major x, min_y <= valid_y <= max_y
  void Set(const double x, const double min_y, const double max_y);
  void Reset(const double x, const double min_y, const double max_y);

  // output some meta data
  friend std::ostream& operator<<(std::ostream& out, const Bitmap2D& bitmap) {
    out << boost::format(
               "min_range: %lf %lf; max_range: %lf %lf;"
               "cell_size: %lf %lf; dims: %d %d; dir_major: %s") %
               bitmap.min_range_.x() % bitmap.min_range_.y() %
               bitmap.max_range_.x() % bitmap.max_range_.y() %
               bitmap.cell_size_.x() % bitmap.cell_size_.y() %
               bitmap.dims_.x() % bitmap.dims_.y() %
               (bitmap.dir_major_ == DirectionMajor::XMAJOR ? "x" : "y");
    return out;
  }

 private:
  // closed range
  inline bool CheckBit(const size_t loc, const uint64_t block) const;
  inline void SetBit(const size_t loc, uint64_t* block);
  inline void ResetBit(const size_t loc, uint64_t* block);

  inline void SetTailBits(const size_t tail_num, uint64_t* block);
  inline void ResetTailBits(const size_t tail_num, uint64_t* block);

  inline void SetHeadBits(const size_t tail_num, uint64_t* block);
  inline void ResetHeadBits(const size_t tail_num, uint64_t* block);

  inline void SetRangeBits(const size_t head, const size_t tail,
                           uint64_t* block);

  inline void ResetRangeBits(const size_t head, const size_t tail,
                             uint64_t* block);

  inline Vec3ui RealToBitmap(const Eigen::Vector2d& p) const;
  inline int Index(const Vec3ui& p) const;

  Eigen::Vector2d min_range_;
  Eigen::Vector2d max_range_;
  Eigen::Vector2d cell_size_;
  Vec2ui dims_;
  DirectionMajor dir_major_ = DirectionMajor::XMAJOR;
  DirectionMajor op_dir_major_ = DirectionMajor::YMAJOR;

  std::vector<uint64_t> bitmap_;
  Vec2ui map_size_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
