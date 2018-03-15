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

/**
 * @file
 * @brief Some util functions.
 */

#ifndef MODULES_COMMON_UTIL_UTIL_H_
#define MODULES_COMMON_UTIL_UTIL_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "google/protobuf/util/message_differencer.h"

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/math/vec2d.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <typename ProtoA, typename ProtoB>
bool IsProtoEqual(const ProtoA& a, const ProtoB& b) {
  return google::protobuf::util::MessageDifferencer::Equals(a, b);
}

struct PairHash {
  template <typename T, typename U>
  size_t operator()(const std::pair<T, U>& pair) const {
    return std::hash<T>()(pair.first) ^ std::hash<U>()(pair.second);
  }
};

template <typename T>
bool WithinBound(T start, T end, T value) {
  return value >= start && value <= end;
}

/**
 * @brief create a SL point
 * @param s the s value
 * @param l the l value
 * @return a SLPoint instance
 */
SLPoint MakeSLPoint(const double s, const double l);

template <typename T>
common::math::Vec2d MakeVec2d(const T& t) {
  return common::math::Vec2d(t.x(), t.y());
}

PointENU MakePointENU(const double x, const double y, const double z);

PointENU operator+(const PointENU enu, const math::Vec2d& xy);

PointENU MakePointENU(const math::Vec2d& xy);

apollo::perception::Point MakePerceptionPoint(const double x, const double y,
                                              const double z);

SpeedPoint MakeSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da);

PathPoint MakePathPoint(const double x, const double y, const double z,
                        const double theta, const double kappa,
                        const double dkappa, const double ddkappa);

/**
 * uniformly slice a segment [start, end] to num + 1 pieces
 * the result sliced will contain the n + 1 points that slices the provided
 * segment. `start` and `end` will be the first and last element in `sliced`.
 */
void uniform_slice(double start, double end, uint32_t num,
                   std::vector<double>* sliced);

template <typename Container>
typename Container::value_type MaxElement(const Container& elements) {
  return *std::max_element(elements.begin(), elements.end());
}

template <typename Container>
typename Container::value_type MinElement(const Container& elements) {
  return *std::min_element(elements.begin(), elements.end());
}

template <typename T>
std::unordered_set<T> Intersection(const std::unordered_set<T>& s1,
                                   const std::unordered_set<T>& s2) {
  if (s1.size() < s2.size()) {
    std::unordered_set<T> result;
    for (const auto& v : s1) {
      if (s2.count(v) > 0) {
        result.insert(v);
      }
    }
    return result;
  } else {
    return intersection(s2, s1);
  }
}

/**
 * calculate the distance beteween Point u and Point v, which are all have
 * member function x() and y() in XY dimension.
 * @param u one point that has member function x() and y().
 * @param b one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2), i.e., the Euclid distance on XY
 * dimension.
 */
template <typename U, typename V>
double DistanceXY(const U& u, const V& v) {
  return std::hypot(u.x() - v.x(), u.y() - v.y());
}

/**
 * Check if two points u and v are the same point on XY dimension.
 * @param u one point that has member function x() and y().
 * @param v one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2) < epsilon, i.e., the Euclid distance
 * on XY dimension.
 */
template <typename U, typename V>
bool SamePointXY(const U& u, const V& v) {
  constexpr double kMathEpsilonSqr = 1e-8 * 1e-8;
  return (u.x() - v.x()) * (u.x() - v.x()) < kMathEpsilonSqr &&
         (u.y() - v.y()) * (u.y() - v.y()) < kMathEpsilonSqr;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2);

// a wrapper template function for remove_if (notice that remove_if cannot
// change the Container size)
template <class Container, class F>
void erase_where(Container& c, F&& f) {  // NOLINT
  c.erase(std::remove_if(c.begin(), c.end(), std::forward<F>(f)), c.end());
}

// a wrapper template function for remove_if on associative containers
template <class Container, class F>
void erase_map_where(Container& c, F&& f) {  // NOLINT
  for (auto it = c.begin(); it != c.end();) {
    if (f(*it)) {
      it = c.erase(it);
    } else {
      ++it;
    }
  }
}

template <typename T>
void QuaternionToRotationMatrix(const T* quat, T* R) {
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

}  // namespace util
}  // namespace common
}  // namespace apollo

template <typename A, typename B>
std::ostream& operator<<(std::ostream& os, std::pair<A, B>& p) {
  return os << "first: " << p.first << ", second: " << p.second;
}

#endif  // MODULES_COMMON_UTIL_UTIL_H_
