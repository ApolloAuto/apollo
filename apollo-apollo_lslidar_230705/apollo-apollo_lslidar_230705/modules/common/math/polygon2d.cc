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

#include "modules/common/math/polygon2d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace common {
namespace math {

Polygon2d::Polygon2d(const Box2d &box) {
  box.GetAllCorners(&points_);
  BuildFromPoints();
}

Polygon2d::Polygon2d(std::vector<Vec2d> points) : points_(std::move(points)) {
  BuildFromPoints();
}

double Polygon2d::DistanceTo(const Vec2d &point) const {
  CHECK_GE(points_.size(), 3U);
  if (IsPointIn(point)) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segments_[i].DistanceTo(point));
  }
  return distance;
}

double Polygon2d::DistanceSquareTo(const Vec2d &point) const {
  CHECK_GE(points_.size(), 3U);
  if (IsPointIn(point)) {
    return 0.0;
  }
  double distance_sqr = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance_sqr =
        std::min(distance_sqr, line_segments_[i].DistanceSquareTo(point));
  }
  return distance_sqr;
}

double Polygon2d::DistanceTo(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return DistanceTo(line_segment.start());
  }
  CHECK_GE(points_.size(), 3U);
  if (IsPointIn(line_segment.center())) {
    return 0.0;
  }
  if (std::any_of(line_segments_.begin(), line_segments_.end(),
                  [&](const LineSegment2d &poly_seg) {
                    return poly_seg.HasIntersect(line_segment);
                  })) {
    return 0.0;
  }

  double distance = std::min(DistanceTo(line_segment.start()),
                             DistanceTo(line_segment.end()));
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segment.DistanceTo(points_[i]));
  }
  return distance;
}

double Polygon2d::DistanceTo(const Box2d &box) const {
  CHECK_GE(points_.size(), 3U);
  return DistanceTo(Polygon2d(box));
}

double Polygon2d::DistanceTo(const Polygon2d &polygon) const {
  CHECK_GE(points_.size(), 3U);
  CHECK_GE(polygon.num_points(), 3);

  if (IsPointIn(polygon.points()[0])) {
    return 0.0;
  }
  if (polygon.IsPointIn(points_[0])) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, polygon.DistanceTo(line_segments_[i]));
  }
  return distance;
}

double Polygon2d::DistanceToBoundary(const Vec2d &point) const {
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segments_[i].DistanceTo(point));
  }
  return distance;
}

bool Polygon2d::IsPointOnBoundary(const Vec2d &point) const {
  CHECK_GE(points_.size(), 3U);
  return std::any_of(
      line_segments_.begin(), line_segments_.end(),
      [&](const LineSegment2d &poly_seg) { return poly_seg.IsPointIn(point); });
}

bool Polygon2d::IsPointIn(const Vec2d &point) const {
  CHECK_GE(points_.size(), 3U);
  if (IsPointOnBoundary(point)) {
    return true;
  }
  int j = num_points_ - 1;
  int c = 0;
  for (int i = 0; i < num_points_; ++i) {
    if ((points_[i].y() > point.y()) != (points_[j].y() > point.y())) {
      const double side = CrossProd(point, points_[i], points_[j]);
      if (points_[i].y() < points_[j].y() ? side > 0.0 : side < 0.0) {
        ++c;
      }
    }
    j = i;
  }
  return c & 1;
}

bool Polygon2d::HasOverlap(const Polygon2d &polygon) const {
  CHECK_GE(points_.size(), 3U);
  if (polygon.max_x() < min_x() || polygon.min_x() > max_x() ||
      polygon.max_y() < min_y() || polygon.min_y() > max_y()) {
    return false;
  }
  return DistanceTo(polygon) <= kMathEpsilon;
}

bool Polygon2d::Contains(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return IsPointIn(line_segment.start());
  }
  CHECK_GE(points_.size(), 3U);
  if (!IsPointIn(line_segment.start())) {
    return false;
  }
  if (!IsPointIn(line_segment.end())) {
    return false;
  }
  if (!is_convex_) {
    std::vector<LineSegment2d> overlaps = GetAllOverlaps(line_segment);
    double total_length = 0;
    for (const auto &overlap_seg : overlaps) {
      total_length += overlap_seg.length();
    }
    return total_length >= line_segment.length() - kMathEpsilon;
  }
  return true;
}

bool Polygon2d::Contains(const Polygon2d &polygon) const {
  CHECK_GE(points_.size(), 3U);
  if (area_ < polygon.area() - kMathEpsilon) {
    return false;
  }
  if (!IsPointIn(polygon.points()[0])) {
    return false;
  }
  const auto &line_segments = polygon.line_segments();
  return std::all_of(line_segments.begin(), line_segments.end(),
                     [&](const LineSegment2d &line_segment) {
                       return Contains(line_segment);
                     });
}

int Polygon2d::Next(int at) const { return at >= num_points_ - 1 ? 0 : at + 1; }

int Polygon2d::Prev(int at) const { return at == 0 ? num_points_ - 1 : at - 1; }

void Polygon2d::BuildFromPoints() {
  num_points_ = static_cast<int>(points_.size());
  CHECK_GE(num_points_, 3);

  // Make sure the points are in ccw order.
  area_ = 0.0;
  for (int i = 1; i < num_points_; ++i) {
    area_ += CrossProd(points_[0], points_[i - 1], points_[i]);
  }
  if (area_ < 0) {
    area_ = -area_;
    std::reverse(points_.begin(), points_.end());
  }
  area_ /= 2.0;
  CHECK_GT(area_, kMathEpsilon);

  // Construct line_segments.
  line_segments_.reserve(num_points_);
  for (int i = 0; i < num_points_; ++i) {
    line_segments_.emplace_back(points_[i], points_[Next(i)]);
  }

  // Check convexity.
  is_convex_ = true;
  for (int i = 0; i < num_points_; ++i) {
    if (CrossProd(points_[Prev(i)], points_[i], points_[Next(i)]) <=
        -kMathEpsilon) {
      is_convex_ = false;
      break;
    }
  }

  // Compute aabox.
  min_x_ = points_[0].x();
  max_x_ = points_[0].x();
  min_y_ = points_[0].y();
  max_y_ = points_[0].y();
  for (const auto &point : points_) {
    min_x_ = std::min(min_x_, point.x());
    max_x_ = std::max(max_x_, point.x());
    min_y_ = std::min(min_y_, point.y());
    max_y_ = std::max(max_y_, point.y());
  }
}

bool Polygon2d::ComputeConvexHull(const std::vector<Vec2d> &points,
                                  Polygon2d *const polygon) {
  CHECK_NOTNULL(polygon);
  const int n = static_cast<int>(points.size());
  if (n < 3) {
    return false;
  }
  std::vector<int> sorted_indices(n);
  for (int i = 0; i < n; ++i) {
    sorted_indices[i] = i;
  }
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&](const int idx1, const int idx2) {
              const Vec2d &pt1 = points[idx1];
              const Vec2d &pt2 = points[idx2];
              const double dx = pt1.x() - pt2.x();
              if (std::abs(dx) > kMathEpsilon) {
                return dx < 0.0;
              }
              return pt1.y() < pt2.y();
            });
  int count = 0;
  std::vector<int> results;
  results.reserve(n);
  int last_count = 1;
  for (int i = 0; i < n + n; ++i) {
    if (i == n) {
      last_count = count;
    }
    const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
    const Vec2d &pt = points[idx];
    while (count > last_count &&
           CrossProd(points[results[count - 2]], points[results[count - 1]],
                     pt) <= kMathEpsilon) {
      results.pop_back();
      --count;
    }
    results.push_back(idx);
    ++count;
  }
  --count;
  if (count < 3) {
    return false;
  }
  std::vector<Vec2d> result_points;
  result_points.reserve(count);
  for (int i = 0; i < count; ++i) {
    result_points.push_back(points[results[i]]);
  }
  *polygon = Polygon2d(result_points);
  return true;
}

bool Polygon2d::ClipConvexHull(const LineSegment2d &line_segment,
                               std::vector<Vec2d> *const points) {
  if (line_segment.length() <= kMathEpsilon) {
    return true;
  }
  CHECK_NOTNULL(points);
  const size_t n = points->size();
  if (n < 3) {
    return false;
  }
  std::vector<double> prod(n);
  std::vector<int> side(n);
  for (size_t i = 0; i < n; ++i) {
    prod[i] = CrossProd(line_segment.start(), line_segment.end(), (*points)[i]);
    if (std::abs(prod[i]) <= kMathEpsilon) {
      side[i] = 0;
    } else {
      side[i] = ((prod[i] < 0) ? -1 : 1);
    }
  }

  std::vector<Vec2d> new_points;
  for (size_t i = 0; i < n; ++i) {
    if (side[i] >= 0) {
      new_points.push_back((*points)[i]);
    }
    const size_t j = ((i == n - 1) ? 0 : (i + 1));
    if (side[i] * side[j] < 0) {
      const double ratio = prod[j] / (prod[j] - prod[i]);
      new_points.emplace_back(
          (*points)[i].x() * ratio + (*points)[j].x() * (1.0 - ratio),
          (*points)[i].y() * ratio + (*points)[j].y() * (1.0 - ratio));
    }
  }

  points->swap(new_points);
  return points->size() >= 3U;
}

bool Polygon2d::ComputeOverlap(const Polygon2d &other_polygon,
                               Polygon2d *const overlap_polygon) const {
  CHECK_GE(points_.size(), 3U);
  CHECK_NOTNULL(overlap_polygon);
  ACHECK(is_convex_ && other_polygon.is_convex());
  std::vector<Vec2d> points = other_polygon.points();
  for (int i = 0; i < num_points_; ++i) {
    if (!ClipConvexHull(line_segments_[i], &points)) {
      return false;
    }
  }
  return ComputeConvexHull(points, overlap_polygon);
}

double Polygon2d::ComputeIoU(const Polygon2d &other_polygon) const {
  Polygon2d overlap_polygon;
  if (!ComputeOverlap(other_polygon, &overlap_polygon)) {
    return 0.0;
  }
  double intersection_area = overlap_polygon.area();
  double union_area = area_ + other_polygon.area() - overlap_polygon.area();
  return intersection_area / union_area;
}

bool Polygon2d::HasOverlap(const LineSegment2d &line_segment) const {
  CHECK_GE(points_.size(), 3U);
  if ((line_segment.start().x() < min_x_ && line_segment.end().x() < min_x_) ||
      (line_segment.start().x() > max_x_ && line_segment.end().x() > max_x_) ||
      (line_segment.start().y() < min_y_ && line_segment.end().y() < min_y_) ||
      (line_segment.start().y() > max_y_ && line_segment.end().y() > max_y_)) {
    return false;
  }
  Vec2d first;
  Vec2d last;
  return GetOverlap(line_segment, &first, &last);
}

bool Polygon2d::GetOverlap(const LineSegment2d &line_segment,
                           Vec2d *const first, Vec2d *const last) const {
  CHECK_GE(points_.size(), 3U);
  CHECK_NOTNULL(first);
  CHECK_NOTNULL(last);

  if (line_segment.length() <= kMathEpsilon) {
    if (!IsPointIn(line_segment.start())) {
      return false;
    }
    *first = line_segment.start();
    *last = line_segment.start();
    return true;
  }

  double min_proj = line_segment.length();
  double max_proj = 0;
  if (IsPointIn(line_segment.start())) {
    *first = line_segment.start();
    min_proj = 0.0;
  }
  if (IsPointIn(line_segment.end())) {
    *last = line_segment.end();
    max_proj = line_segment.length();
  }
  for (const auto &poly_seg : line_segments_) {
    Vec2d pt;
    if (poly_seg.GetIntersect(line_segment, &pt)) {
      const double proj = line_segment.ProjectOntoUnit(pt);
      if (proj < min_proj) {
        min_proj = proj;
        *first = pt;
      }
      if (proj > max_proj) {
        max_proj = proj;
        *last = pt;
      }
    }
  }
  return min_proj <= max_proj + kMathEpsilon;
}

void Polygon2d::GetAllVertices(std::vector<Vec2d> *const vertices) const {
  if (vertices == nullptr) {
    return;
  }
  *vertices = points_;
}

std::vector<Vec2d> Polygon2d::GetAllVertices() const { return points_; }

std::vector<LineSegment2d> Polygon2d::GetAllOverlaps(
    const LineSegment2d &line_segment) const {
  CHECK_GE(points_.size(), 3U);

  if (line_segment.length() <= kMathEpsilon) {
    std::vector<LineSegment2d> overlaps;
    if (IsPointIn(line_segment.start())) {
      overlaps.push_back(line_segment);
    }
    return overlaps;
  }
  std::vector<double> projections;
  if (IsPointIn(line_segment.start())) {
    projections.push_back(0.0);
  }
  if (IsPointIn(line_segment.end())) {
    projections.push_back(line_segment.length());
  }
  for (const auto &poly_seg : line_segments_) {
    Vec2d pt;
    if (poly_seg.GetIntersect(line_segment, &pt)) {
      projections.push_back(line_segment.ProjectOntoUnit(pt));
    }
  }
  std::sort(projections.begin(), projections.end());
  std::vector<std::pair<double, double>> overlaps;
  for (size_t i = 0; i + 1 < projections.size(); ++i) {
    const double start_proj = projections[i];
    const double end_proj = projections[i + 1];
    if (end_proj - start_proj <= kMathEpsilon) {
      continue;
    }
    const Vec2d reference_point =
        line_segment.start() +
        (start_proj + end_proj) / 2.0 * line_segment.unit_direction();
    if (!IsPointIn(reference_point)) {
      continue;
    }
    if (overlaps.empty() ||
        start_proj > overlaps.back().second + kMathEpsilon) {
      overlaps.emplace_back(start_proj, end_proj);
    } else {
      overlaps.back().second = end_proj;
    }
  }
  std::vector<LineSegment2d> overlap_line_segments;
  for (const auto &overlap : overlaps) {
    overlap_line_segments.emplace_back(
        line_segment.start() + overlap.first * line_segment.unit_direction(),
        line_segment.start() + overlap.second * line_segment.unit_direction());
  }
  return overlap_line_segments;
}

void Polygon2d::ExtremePoints(const double heading, Vec2d *const first,
                              Vec2d *const last) const {
  CHECK_GE(points_.size(), 3U);
  CHECK_NOTNULL(first);
  CHECK_NOTNULL(last);

  const Vec2d direction_vec = Vec2d::CreateUnitVec2d(heading);
  double min_proj = std::numeric_limits<double>::infinity();
  double max_proj = -std::numeric_limits<double>::infinity();
  for (const auto &pt : points_) {
    const double proj = pt.InnerProd(direction_vec);
    if (proj < min_proj) {
      min_proj = proj;
      *first = pt;
    }
    if (proj > max_proj) {
      max_proj = proj;
      *last = pt;
    }
  }
}

AABox2d Polygon2d::AABoundingBox() const {
  return AABox2d({min_x_, min_y_}, {max_x_, max_y_});
}

Box2d Polygon2d::BoundingBoxWithHeading(const double heading) const {
  CHECK_GE(points_.size(), 3U);
  const Vec2d direction_vec = Vec2d::CreateUnitVec2d(heading);
  Vec2d px1;
  Vec2d px2;
  Vec2d py1;
  Vec2d py2;
  ExtremePoints(heading, &px1, &px2);
  ExtremePoints(heading - M_PI_2, &py1, &py2);
  const double x1 = px1.InnerProd(direction_vec);
  const double x2 = px2.InnerProd(direction_vec);
  const double y1 = py1.CrossProd(direction_vec);
  const double y2 = py2.CrossProd(direction_vec);
  return Box2d(
      (x1 + x2) / 2.0 * direction_vec +
          (y1 + y2) / 2.0 * Vec2d(direction_vec.y(), -direction_vec.x()),
      heading, x2 - x1, y2 - y1);
}

Box2d Polygon2d::MinAreaBoundingBox() const {
  CHECK_GE(points_.size(), 3U);
  if (!is_convex_) {
    Polygon2d convex_polygon;
    ComputeConvexHull(points_, &convex_polygon);
    ACHECK(convex_polygon.is_convex());
    return convex_polygon.MinAreaBoundingBox();
  }
  double min_area = std::numeric_limits<double>::infinity();
  double min_area_at_heading = 0.0;
  int left_most = 0;
  int right_most = 0;
  int top_most = 0;
  for (int i = 0; i < num_points_; ++i) {
    const auto &line_segment = line_segments_[i];
    double proj = 0.0;
    double min_proj = line_segment.ProjectOntoUnit(points_[left_most]);
    while ((proj = line_segment.ProjectOntoUnit(points_[Prev(left_most)])) <
           min_proj) {
      min_proj = proj;
      left_most = Prev(left_most);
    }
    while ((proj = line_segment.ProjectOntoUnit(points_[Next(left_most)])) <
           min_proj) {
      min_proj = proj;
      left_most = Next(left_most);
    }
    double max_proj = line_segment.ProjectOntoUnit(points_[right_most]);
    while ((proj = line_segment.ProjectOntoUnit(points_[Prev(right_most)])) >
           max_proj) {
      max_proj = proj;
      right_most = Prev(right_most);
    }
    while ((proj = line_segment.ProjectOntoUnit(points_[Next(right_most)])) >
           max_proj) {
      max_proj = proj;
      right_most = Next(right_most);
    }
    double prod = 0.0;
    double max_prod = line_segment.ProductOntoUnit(points_[top_most]);
    while ((prod = line_segment.ProductOntoUnit(points_[Prev(top_most)])) >
           max_prod) {
      max_prod = prod;
      top_most = Prev(top_most);
    }
    while ((prod = line_segment.ProductOntoUnit(points_[Next(top_most)])) >
           max_prod) {
      max_prod = prod;
      top_most = Next(top_most);
    }
    const double area = max_prod * (max_proj - min_proj);
    if (area < min_area) {
      min_area = area;
      min_area_at_heading = line_segment.heading();
    }
  }
  return BoundingBoxWithHeading(min_area_at_heading);
}

Polygon2d Polygon2d::ExpandByDistance(const double distance) const {
  if (!is_convex_) {
    Polygon2d convex_polygon;
    ComputeConvexHull(points_, &convex_polygon);
    ACHECK(convex_polygon.is_convex());
    return convex_polygon.ExpandByDistance(distance);
  }
  const double kMinAngle = 0.1;
  std::vector<Vec2d> points;
  for (int i = 0; i < num_points_; ++i) {
    const double start_angle = line_segments_[Prev(i)].heading() - M_PI_2;
    const double end_angle = line_segments_[i].heading() - M_PI_2;
    const double diff = WrapAngle(end_angle - start_angle);
    if (diff <= kMathEpsilon) {
      points.push_back(points_[i] +
                       Vec2d::CreateUnitVec2d(start_angle) * distance);
    } else {
      const int count = static_cast<int>(diff / kMinAngle) + 1;
      for (int k = 0; k <= count; ++k) {
        const double angle = start_angle + diff * static_cast<double>(k) /
                                               static_cast<double>(count);
        points.push_back(points_[i] + Vec2d::CreateUnitVec2d(angle) * distance);
      }
    }
  }
  Polygon2d new_polygon;
  ACHECK(ComputeConvexHull(points, &new_polygon));
  return new_polygon;
}

std::string Polygon2d::DebugString() const {
  return absl::StrCat("polygon2d (  num_points = ", num_points_, "  points = (",
                      absl::StrJoin(points_, " ", util::DebugStringFormatter()),
                      " )  ", is_convex_ ? "convex" : "non-convex",
                      "  area = ", area_, " )");
}

}  // namespace math
}  // namespace common
}  // namespace apollo
