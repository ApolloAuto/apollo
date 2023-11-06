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

#include "modules/common/math/box2d.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"

namespace apollo {
namespace common {
namespace math {
namespace {

double PtSegDistance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y,
                     double length) {
  const double x0 = query_x - start_x;
  const double y0 = query_y - start_y;
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double proj = x0 * dx + y0 * dy;
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length * length) {
    return hypot(x0 - dx, y0 - dy);
  }
  return std::abs(x0 * dy - y0 * dx) / length;
}

}  // namespace

Box2d::Box2d(const Vec2d &center, const double heading, const double length,
             const double width)
    : center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {
  CHECK_GT(length_, -kMathEpsilon);
  CHECK_GT(width_, -kMathEpsilon);
  InitCorners();
}

Box2d::Box2d(const Vec2d &point, double heading, double front_length,
             double back_length, double width)
    : length_(front_length + back_length),
      width_(width),
      half_length_(length_ / 2.0),
      half_width_(width / 2.0),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {
  CHECK_GT(length_, -kMathEpsilon);
  CHECK_GT(width_, -kMathEpsilon);
  double delta_length = (front_length - back_length) / 2.0;
  center_ = Vec2d(point.x() + cos_heading_ * delta_length,
                  point.y() + sin_heading_ * delta_length);
  InitCorners();
}

Box2d::Box2d(const LineSegment2d &axis, const double width)
    : center_(axis.center()),
      length_(axis.length()),
      width_(width),
      half_length_(axis.length() / 2.0),
      half_width_(width / 2.0),
      heading_(axis.heading()),
      cos_heading_(axis.cos_heading()),
      sin_heading_(axis.sin_heading()) {
  CHECK_GT(length_, -kMathEpsilon);
  CHECK_GT(width_, -kMathEpsilon);
  InitCorners();
}

void Box2d::InitCorners() {
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  corners_.clear();
  corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
  corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

  for (auto &corner : corners_) {
    max_x_ = std::fmax(corner.x(), max_x_);
    min_x_ = std::fmin(corner.x(), min_x_);
    max_y_ = std::fmax(corner.y(), max_y_);
    min_y_ = std::fmin(corner.y(), min_y_);
  }
}

Box2d::Box2d(const AABox2d &aabox)
    : center_(aabox.center()),
      length_(aabox.length()),
      width_(aabox.width()),
      half_length_(aabox.half_length()),
      half_width_(aabox.half_width()),
      heading_(0.0),
      cos_heading_(1.0),
      sin_heading_(0.0) {
  CHECK_GT(length_, -kMathEpsilon);
  CHECK_GT(width_, -kMathEpsilon);
}

Box2d Box2d::CreateAABox(const Vec2d &one_corner,
                         const Vec2d &opposite_corner) {
  const double x1 = std::min(one_corner.x(), opposite_corner.x());
  const double x2 = std::max(one_corner.x(), opposite_corner.x());
  const double y1 = std::min(one_corner.y(), opposite_corner.y());
  const double y2 = std::max(one_corner.y(), opposite_corner.y());
  return Box2d({(x1 + x2) / 2.0, (y1 + y2) / 2.0}, 0.0, x2 - x1, y2 - y1);
}

void Box2d::GetAllCorners(std::vector<Vec2d> *const corners) const {
  if (corners == nullptr) {
    return;
  }
  *corners = corners_;
}

const std::vector<Vec2d> &Box2d::GetAllCorners() const { return corners_; }

bool Box2d::IsPointIn(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
  return dx <= half_length_ + kMathEpsilon && dy <= half_width_ + kMathEpsilon;
}

bool Box2d::IsPointOnBoundary(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
  return (std::abs(dx - half_length_) <= kMathEpsilon &&
          dy <= half_width_ + kMathEpsilon) ||
         (std::abs(dy - half_width_) <= kMathEpsilon &&
          dx <= half_length_ + kMathEpsilon);
}

double Box2d::DistanceTo(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}

bool Box2d::HasOverlap(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return IsPointIn(line_segment.start());
  }
  if (std::fmax(line_segment.start().x(), line_segment.end().x()) < min_x() ||
      std::fmin(line_segment.start().x(), line_segment.end().x()) > max_x() ||
      std::fmax(line_segment.start().y(), line_segment.end().y()) < min_y() ||
      std::fmin(line_segment.start().y(), line_segment.end().y()) > max_y()) {
    return false;
  }
  // Construct coordinate system with origin point as left_bottom corner of
  // Box2d, y axis along direction of heading.
  Vec2d x_axis(sin_heading_, -cos_heading_);
  Vec2d y_axis(cos_heading_, sin_heading_);
  // corners_[2] is the left bottom point of the box.
  Vec2d start_v = line_segment.start() - corners_[2];
  // "start_point" is the start point of "line_segment" mapped in the new
  // coordinate system.
  Vec2d start_point(start_v.InnerProd(x_axis), start_v.InnerProd(y_axis));
  // Check if "start_point" is inside the box.
  if (is_inside_rectangle(start_point)) {
    return true;
  }
  // Check if "end_point" is inside the box.
  Vec2d end_v = line_segment.end() - corners_[2];
  Vec2d end_point(end_v.InnerProd(x_axis), end_v.InnerProd(y_axis));
  if (is_inside_rectangle(end_point)) {
    return true;
  }
  // Exclude the case when the 2 points of "line_segment" are at the same side
  // of rectangle.
  if ((start_point.x() < 0.0) && (end_point.x() < 0.0)) {
    return false;
  }
  if ((start_point.y() < 0.0) && (end_point.y() < 0.0)) {
    return false;
  }
  if ((start_point.x() > width_) && (end_point.x() > width_)) {
    return false;
  }
  if ((start_point.y() > length_) && (end_point.y() > length_)) {
    return false;
  }
  // Check if "line_segment" intersects with box.
  Vec2d line_direction = line_segment.end() - line_segment.start();
  Vec2d normal_vec(line_direction.y(), -line_direction.x());
  Vec2d p1 = center_ - line_segment.start();
  Vec2d diagonal_vec = center_ - corners_[0];
  // if project_p1 < projection of diagonal, "line_segment" intersects with box.
  double project_p1 = fabs(p1.InnerProd(normal_vec));
  if (fabs(diagonal_vec.InnerProd(normal_vec)) >= project_p1) {
    return true;
  }
  diagonal_vec = center_ - corners_[1];
  if (fabs(diagonal_vec.InnerProd(normal_vec)) >= project_p1) {
    return true;
  }
  return false;
}

double Box2d::DistanceTo(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return DistanceTo(line_segment.start());
  }
  const double ref_x1 = line_segment.start().x() - center_.x();
  const double ref_y1 = line_segment.start().y() - center_.y();
  double x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
  double y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
  double box_x = half_length_;
  double box_y = half_width_;
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }
  const double ref_x2 = line_segment.end().x() - center_.x();
  const double ref_y2 = line_segment.end().y() - center_.y();
  double x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
  double y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:
        return PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                             line_segment.length());
      case 3:
        return (x1 > x2) ? (x2 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 2:
        return (x1 > x2) ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                         line_segment.length())
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case -1:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -4:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                   ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length())
                   : (CrossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                          ? 0.0
                          : PtSegDistance(-box_x, box_y, x1, y1, x2, y2,
                                          line_segment.length()));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:
        return (x1 < x2) ? (x1 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 3:
        return std::min(x1, x2) - box_x;
      case 1:
      case -2:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -3:
        return 0.0;
    }
  }
  ACHECK(0) << "unimplemented state: " << gx1 << " " << gy1 << " " << gx2 << " "
            << gy2;
  return 0.0;
}

double Box2d::DistanceTo(const Box2d &box) const {
  return Polygon2d(box).DistanceTo(*this);
}

bool Box2d::HasOverlap(const Box2d &box) const {
  if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
      box.min_y() > max_y()) {
    return false;
  }

  const double shift_x = box.center_x() - center_.x();
  const double shift_y = box.center_y() - center_.y();

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  const double dx3 = box.cos_heading() * box.half_length();
  const double dy3 = box.sin_heading() * box.half_length();
  const double dx4 = box.sin_heading() * box.half_width();
  const double dy4 = -box.cos_heading() * box.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                 box.half_length() &&
         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                 box.half_width();
}

AABox2d Box2d::GetAABox() const {
  const double dx1 = std::abs(cos_heading_ * half_length_);
  const double dy1 = std::abs(sin_heading_ * half_length_);
  const double dx2 = std::abs(sin_heading_ * half_width_);
  const double dy2 = std::abs(cos_heading_ * half_width_);
  return AABox2d(center_, (dx1 + dx2) * 2.0, (dy1 + dy2) * 2.0);
}

void Box2d::RotateFromCenter(const double rotate_angle) {
  heading_ = NormalizeAngle(heading_ + rotate_angle);
  cos_heading_ = std::cos(heading_);
  sin_heading_ = std::sin(heading_);
  InitCorners();
}

void Box2d::Shift(const Vec2d &shift_vec) {
  center_ += shift_vec;
  for (size_t i = 0; i < 4; ++i) {
    corners_[i] += shift_vec;
  }
  for (auto &corner : corners_) {
    max_x_ = std::fmax(corner.x(), max_x_);
    min_x_ = std::fmin(corner.x(), min_x_);
    max_y_ = std::fmax(corner.y(), max_y_);
    min_y_ = std::fmin(corner.y(), min_y_);
  }
}

void Box2d::LongitudinalExtend(const double extension_length) {
  length_ += extension_length;
  half_length_ += extension_length / 2.0;
  InitCorners();
}

void Box2d::LateralExtend(const double extension_length) {
  width_ += extension_length;
  half_width_ += extension_length / 2.0;
  InitCorners();
}

std::string Box2d::DebugString() const {
  return absl::StrCat("box2d ( center = ", center_.DebugString(),
                      "  heading = ", heading_, "  length = ", length_,
                      "  width = ", width_, " )");
}

}  // namespace math
}  // namespace common
}  // namespace apollo
