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

#include "modules/common/math/aabox2d.h"

#include <algorithm>
#include <cmath>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace common {
namespace math {

AABox2d::AABox2d(const Vec2d &center, const double length, const double width)
    : center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0) {
  CHECK_GT(length_, -kMathEpsilon);
  CHECK_GT(width_, -kMathEpsilon);
}

AABox2d::AABox2d(const Vec2d &one_corner, const Vec2d &opposite_corner)
    : AABox2d((one_corner + opposite_corner) / 2.0,
              std::abs(one_corner.x() - opposite_corner.x()),
              std::abs(one_corner.y() - opposite_corner.y())) {}

AABox2d::AABox2d(const std::vector<Vec2d> &points) {
  ACHECK(!points.empty());
  double min_x = points[0].x();
  double max_x = points[0].x();
  double min_y = points[0].y();
  double max_y = points[0].y();
  for (const auto &point : points) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }

  center_ = {(min_x + max_x) / 2.0, (min_y + max_y) / 2.0};
  length_ = max_x - min_x;
  width_ = max_y - min_y;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

void AABox2d::GetAllCorners(std::vector<Vec2d> *const corners) const {
  CHECK_NOTNULL(corners)->clear();
  corners->reserve(4);
  corners->emplace_back(center_.x() + half_length_, center_.y() - half_width_);
  corners->emplace_back(center_.x() + half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() + half_width_);
  corners->emplace_back(center_.x() - half_length_, center_.y() - half_width_);
}

bool AABox2d::IsPointIn(const Vec2d &point) const {
  return std::abs(point.x() - center_.x()) <= half_length_ + kMathEpsilon &&
         std::abs(point.y() - center_.y()) <= half_width_ + kMathEpsilon;
}

bool AABox2d::IsPointOnBoundary(const Vec2d &point) const {
  const double dx = std::abs(point.x() - center_.x());
  const double dy = std::abs(point.y() - center_.y());
  return (std::abs(dx - half_length_) <= kMathEpsilon &&
          dy <= half_width_ + kMathEpsilon) ||
         (std::abs(dy - half_width_) <= kMathEpsilon &&
          dx <= half_length_ + kMathEpsilon);
}

double AABox2d::DistanceTo(const Vec2d &point) const {
  const double dx = std::abs(point.x() - center_.x()) - half_length_;
  const double dy = std::abs(point.y() - center_.y()) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

double AABox2d::DistanceTo(const AABox2d &box) const {
  const double dx =
      std::abs(box.center_x() - center_.x()) - box.half_length() - half_length_;
  const double dy =
      std::abs(box.center_y() - center_.y()) - box.half_width() - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

bool AABox2d::HasOverlap(const AABox2d &box) const {
  return std::abs(box.center_x() - center_.x()) <=
             box.half_length() + half_length_ &&
         std::abs(box.center_y() - center_.y()) <=
             box.half_width() + half_width_;
}

void AABox2d::Shift(const Vec2d &shift_vec) { center_ += shift_vec; }

void AABox2d::MergeFrom(const AABox2d &other_box) {
  const double x1 = std::min(min_x(), other_box.min_x());
  const double x2 = std::max(max_x(), other_box.max_x());
  const double y1 = std::min(min_y(), other_box.min_y());
  const double y2 = std::max(max_y(), other_box.max_y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  length_ = x2 - x1;
  width_ = y2 - y1;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

void AABox2d::MergeFrom(const Vec2d &other_point) {
  const double x1 = std::min(min_x(), other_point.x());
  const double x2 = std::max(max_x(), other_point.x());
  const double y1 = std::min(min_y(), other_point.y());
  const double y2 = std::max(max_y(), other_point.y());
  center_ = Vec2d((x1 + x2) / 2.0, (y1 + y2) / 2.0);
  length_ = x2 - x1;
  width_ = y2 - y1;
  half_length_ = length_ / 2.0;
  half_width_ = width_ / 2.0;
}

std::string AABox2d::DebugString() const {
  return absl::StrCat("aabox2d ( center = ", center_.DebugString(),
                      "  length = ", length_, "  width = ", width_, " )");
}

}  // namespace math
}  // namespace common
}  // namespace apollo
