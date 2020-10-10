/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/tool/benchmark/lidar/util/geo_util.h"
#include <utility>

namespace apollo {
namespace perception {
namespace benchmark {

bool is_point_xy_in_polygon2d_xy(const Point& point, const PointCloud& polygon,
                                 float distance_to_boundary) {
  typedef float Type;
  bool in_poly = false;
  Type x1 = 0.0;
  Type x2 = 0.0;
  Type y1 = 0.0;
  Type y2 = 0.0;
  size_t nr_poly_points = polygon.size();
  // start with the last point to make the check last point<->first point the
  // first one
  Type xold = polygon.at(nr_poly_points - 1).x;
  Type yold = polygon.at(nr_poly_points - 1).y;
  for (size_t i = 0; i < nr_poly_points; ++i) {
    Type xnew = polygon.at(i).x;
    Type ynew = polygon.at(i).y;
    if (xnew > xold) {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    } else {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }
    // if the point is on the boundary, then it is defined as in the polygon
    Type value = (point.y - y1) * (x2 - x1) - (y2 - y1) * (point.x - x1);
    Type temp = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    if (temp < std::numeric_limits<Type>::epsilon()) {
      continue;
    }
    Type distance = std::abs(value) / temp;
    if (x1 <= point.x && point.x <= x2 && distance < distance_to_boundary) {
      return true;
    }
    if ((x1 < point.x) == (point.x <= x2) && value < 0.f) {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }
  return in_poly;
}

bool VisPoint::operator<(const VisPoint& other) const {
  bool is_a_left = strictly_less(x(), 0.0);
  bool is_b_left = strictly_less(other.x(), 0.0);
  if (is_a_left != is_b_left) {
    return is_b_left;
  }

  if (approx_equal(x(), 0.0) && approx_equal(other.x(), 0.0)) {
    bool is_a_down = strictly_less(y(), 0.0);
    bool is_b_down = strictly_less(other.y(), 0.0);
    if (is_a_down != is_b_down) {
      return is_b_down;
    }
    if (is_a_down) {
      return strictly_less(other.y(), y());
    } else {
      return strictly_less(y(), other.y());
    }
  }

  float det = cross(other);
  if (approx_equal(det, 0.0)) {
    return length_squared() < other.length_squared();
  }
  return det < 0;
}

bool VisPoint::operator==(const VisPoint& other) const {
  return approx_equal(x(), other.x()) && approx_equal(y(), other.y());
}

bool VisPoint::operator!=(const VisPoint& other) const {
  return !(*this == other);
}

bool Segment::operator<(const Segment& other) const {
  VisPoint a = start, b = end;
  VisPoint c = other.start, d = other.end;
  VisPoint o = VisPoint(0, 0);

  if (approx_equal(b.x(), 0.0)) {
    b.x() = -std::numeric_limits<float>::epsilon();
  }
  if (approx_equal(d.x(), 0.0)) {
    d.x() = -std::numeric_limits<float>::epsilon();
  }

  // will not have overlap part
  if (b < c) {
    return true;
  } else if (d < a) {
    return false;
  }

  // sort the endpoints so that if there are common endpoints,
  // it will be a and c
  if (b == c || b == d) {
    std::swap(a, b);
  }
  if (a == d) {
    std::swap(c, d);
  }

  // cases with common endpoints
  if (a == c) {
    if (b == d) {
      // same segment
      return false;
    }
    if (compute_orientation(o, a, d) != compute_orientation(o, a, b)) {
      // will not have overlap part, (true or false) are all functional correct
      // for std::set sort function
      return b < d;
    } else {
      // same side with origin or not
      return compute_orientation(a, b, d) != compute_orientation(a, b, o);
    }
  }

  // cases without common endpoints
  auto cda = compute_orientation(c, d, a);
  auto cdb = compute_orientation(c, d, b);
  auto abc = compute_orientation(a, b, c);
  auto abd = compute_orientation(a, b, d);

  if (cda == Orientation::collinear && cdb == Orientation::collinear) {
    // segments are collinear
    return a < c;
  } else if (cda == Orientation::collinear || cdb == Orientation::collinear) {
    auto cdo = compute_orientation(c, d, o);
    return cdo == cda || cdo == cdb;
  } else if (abc == Orientation::collinear || abd == Orientation::collinear) {
    auto abo = compute_orientation(a, b, o);
    return abo != abc && abo != abd;
  } else if (cda == cdb) {
    auto cdo = compute_orientation(c, d, o);
    return cdo == cda;
  } else {
    // segment cd is same side with origin or not
    return compute_orientation(a, b, o) != compute_orientation(a, b, c);
  }
}

bool Segment::operator==(const Segment& other) const {
  return start == other.start && end == other.end;
}

bool Segment::operator!=(const Segment& other) const {
  return !(*this == other);
}

Orientation compute_orientation(const VisPoint& o, const VisPoint& a,
                                const VisPoint& b) {
  float det = (a - o).cross(b - o);
  return static_cast<Orientation>(static_cast<int>(strictly_less(0.0, det)) -
                                  static_cast<int>(strictly_less(det, 0.0)));
}

bool intersects(const VisPoint& ray, const Segment& segment,
                VisPoint* intersection) {
  // let a = a2 - a1 (ray), b = b2 - b1 (segment),
  // accordingly segment: s1 = a1 + t*a, s2 = b1 + u*b
  // intersection: a1 + t*a = b1 + u*b, when 0<=t (ray) && 0<=u<=1 (segment)
  VisPoint a = ray;
  VisPoint b = segment.end - segment.start;

  float t = segment.start.cross(b) / a.cross(b);
  float u = a.cross(-segment.start) / a.cross(b);
  *intersection = a * t;

  if (std::isfinite(t) && t >= 0 && std::isfinite(u) && u >= 0 && u <= 1) {
    return true;
  }
  return false;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
