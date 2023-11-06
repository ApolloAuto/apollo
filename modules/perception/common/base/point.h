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

#include <limits>
#include <memory>
#include <vector>

namespace apollo {
namespace perception {
namespace base {

template <typename T>
struct alignas(16) Point {
  T x = 0;
  T y = 0;
  T z = 0;
  T intensity = 0;
  typedef T Type;
};

template <typename T>
struct PointXYZIT : public Point<T> {
  double timestamp = 0.0;
};

template <typename T>
struct PointXYZITH : public PointXYZIT<T> {
  float height = std::numeric_limits<float>::max();
};

template <typename T>
struct PointXYZITHB : public PointXYZITH<T> {
  int32_t beam_id = -1;
};

template <typename T>
struct PointXYZITHBL : public PointXYZITHB<T> {
  uint8_t label = 0;
};

using PointF = Point<float>;
using PointD = Point<double>;

using PointXYZIF = Point<float>;
using PointXYZID = Point<double>;
using PointXYZITF = PointXYZIT<float>;
using PointXYZITD = PointXYZIT<double>;
using PointXYZITHF = PointXYZITH<float>;
using PointXYZITHD = PointXYZITH<double>;
using PointXYZITHBF = PointXYZITHB<float>;
using PointXYZITHBD = PointXYZITHB<double>;
using PointXYZITHBLF = PointXYZITHBL<float>;
using PointXYZITHBLD = PointXYZITHBL<double>;

const std::size_t kDefaultReservePointNum = 50000;

struct PointIndices {
  PointIndices() { indices.reserve(kDefaultReservePointNum); }

  std::vector<int> indices;

  typedef std::shared_ptr<PointIndices> Ptr;
  typedef std::shared_ptr<const PointIndices> ConstPtr;
};

template <typename T>
struct Point2D {
  T x = 0;
  T y = 0;
};

using Point2DF = Point2D<float>;
using Point2DI = Point2D<int>;
using Point2DD = Point2D<double>;

template <typename T>
struct Point3D {
  T x = 0;
  T y = 0;
  T z = 0;
};

using Point3DF = Point3D<float>;
using Point3DI = Point3D<int>;
using Point3DD = Point3D<double>;

template <typename T>
struct alignas(16) RadarPoint {
  T x = 0;
  T y = 0;
  T z = 0;
  T velocity = 0;
  T comp_vel = 0;
  T rcs = 0;
  typedef T Type;
};

using RadarPointF = RadarPoint<float>;
using RadarPointD = RadarPoint<double>;
using RadarPointXYZVRF = RadarPoint<float>;
using RadarPointXYZVRD = RadarPoint<double>;

}  // namespace base
}  // namespace perception
}  // namespace apollo
