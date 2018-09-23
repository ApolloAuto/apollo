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

#ifndef MODULES_PERCEPTION_BASE_POINT_H_
#define MODULES_PERCEPTION_BASE_POINT_H_

#include <cfloat>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace apollo {
namespace perception {
namespace base {

template <typename T>
struct Point {
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

typedef Point<float> PointXYZIF;
typedef Point<double> PointXYZID;
typedef PointXYZIT<float> PointXYZITF;
typedef PointXYZIT<double> PointXYZITD;
typedef PointXYZITH<float> PointXYZITHF;
typedef PointXYZITH<double> PointXYZITHD;
typedef PointXYZITHB<float> PointXYZITHBF;
typedef PointXYZITHB<double> PointXYZITHBD;
typedef PointXYZITHBL<float> PointXYZITHBLF;
typedef PointXYZITHBL<double> PointXYZITHBLD;

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

typedef Point2D<int> Point2DI;
typedef Point2D<float> Point2DF;
typedef Point2D<double> Point2DD;

template <typename T>
struct Point3D {
  T x = 0;
  T y = 0;
  T z = 0;
};

typedef Point3D<int> Point3DI;
typedef Point3D<float> Point3DF;
typedef Point3D<double> Point3DD;

}  // namespace base
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_BASE_POINT_H_
