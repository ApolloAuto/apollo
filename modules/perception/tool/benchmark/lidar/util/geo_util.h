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
#pragma once

#include <algorithm>
#include <limits>

#include "Eigen/Core"

#include "modules/perception/tool/benchmark/lidar/util/types.h"

namespace apollo {
namespace perception {
namespace benchmark {

template <typename T>
void quaternion_to_rotation_matrix(const T* quat, T* R) {
  T x2 = quat[0] * quat[0];
  T xy = quat[0] * quat[1];
  T rx = quat[3] * quat[0];
  T y2 = quat[1] * quat[1];
  T yz = quat[1] * quat[2];
  T ry = quat[3] * quat[1];
  T z2 = quat[2] * quat[2];
  T zx = quat[2] * quat[0];
  T rz = quat[3] * quat[2];
  T r2 = quat[3] * quat[3];
  R[0] = r2 + x2 - y2 - z2;  // fill diagonal terms
  R[4] = r2 - x2 + y2 - z2;
  R[8] = r2 - x2 - y2 + z2;
  R[3] = 2 * (xy + rz);  // fill off diagonal terms
  R[6] = 2 * (zx - ry);
  R[7] = 2 * (yz + rx);
  R[1] = 2 * (xy - rz);
  R[2] = 2 * (zx + ry);
  R[5] = 2 * (yz - rx);
}

bool is_point_xy_in_polygon2d_xy(const Point& point, const PointCloud& polygon,
                                 float distance_to_boundary);

inline bool approx_equal(float a, float b) {
  return std::abs(a - b) <= std::max(std::abs(a), std::abs(b)) *
                                std::numeric_limits<float>::epsilon();
}

inline bool strictly_less(float a, float b) {
  return (b - a) > std::max(std::abs(a), std::abs(b)) *
                       std::numeric_limits<float>::epsilon();
}

enum class Orientation {
  left = 1,
  right = -1,
  collinear = 0,
};

struct VisPoint {
  VisPoint() = default;

  explicit VisPoint(float x, float y) : point(Eigen::Vector2f(x, y)) {}

  explicit VisPoint(const Eigen::Vector2f& point_) : point(point_) {}

  bool operator<(const VisPoint& other) const;

  bool operator==(const VisPoint& other) const;

  bool operator!=(const VisPoint& other) const;

  VisPoint operator+(const VisPoint& other) const {
    return VisPoint(x() + other.x(), y() + other.y());
  }

  VisPoint operator-(const VisPoint& other) const {
    return VisPoint(x() - other.x(), y() - other.y());
  }

  VisPoint operator-() const { return VisPoint(-x(), -y()); }

  VisPoint operator*(float scale) const {
    return VisPoint(x() * scale, y() * scale);
  }

  VisPoint operator/(float scale) const {
    return VisPoint(x() / scale, y() / scale);
  }

  friend std::ostream& operator<<(std::ostream& output, const VisPoint& p) {
    output << "(" << p.x() << ", " << p.y() << ")";
    return output;
  }

  inline const float& x() const { return point.x(); }

  inline const float& y() const { return point.y(); }

  inline float& x() { return point.x(); }

  inline float& y() { return point.y(); }

  inline float dot(const VisPoint& other) const {
    return x() * other.x() + y() * other.y();
  }

  inline float cross(const VisPoint& other) const {
    return x() * other.y() - y() * other.x();
  }

  inline float length_squared() const { return x() * x() + y() * y(); }

  Eigen::Vector2f point;
};

struct Segment {
  Segment() = default;

  explicit Segment(const VisPoint& start, const VisPoint& end, int idx = -2)
      : start(start), end(end), idx(idx) {}

  bool operator<(const Segment& other) const;

  bool operator==(const Segment& other) const;

  bool operator!=(const Segment& other) const;

  friend std::ostream& operator<<(std::ostream& output, const Segment& s) {
    output << "Segment: start: " << s.start << ", end: " << s.end
           << ", idx: " << s.idx;
    return output;
  }

  VisPoint start, end;
  int idx = -2;
};

// position relation between Point B and line OA
Orientation compute_orientation(const VisPoint& o, const VisPoint& a,
                                const VisPoint& b);

// calculate intersection point between ray and segment
bool intersects(const VisPoint& ray, const Segment& segment,
                VisPoint* intersection);

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
