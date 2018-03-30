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

#include <cmath>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/QR"

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_UTIL_H_

namespace apollo {
namespace perception {

// @brief: convert angle from the range of [-pi, pi] to [0, 2*pi]
void RectAngle(ScalarType *theta);

// @brief: fit polynomial function with QR decomposition (using Eigen 3)
template <typename T = ScalarType>
bool PolyFit(const std::vector<Eigen::Matrix<T, 2, 1>> &pos_vec,
             const int &order, Eigen::Matrix<T, MAX_POLY_ORDER + 1, 1> *coeff,
             const bool &is_x_axis = true) {
  if (coeff == NULL) {
    AERROR << "The coefficient pointer is NULL.";
    return false;
  }

  if (order > MAX_POLY_ORDER) {
    AERROR << "The order of polynomial must be smaller than " << MAX_POLY_ORDER;
    return false;
  }

  int n = static_cast<int>(pos_vec.size());
  if (n <= order) {
    AERROR << "The number of points should be larger than the order. #points = "
           << pos_vec.size();
    return false;
  }

  // create data matrix
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A(n, order + 1);
  Eigen::Matrix<T, Eigen::Dynamic, 1> y(n);
  Eigen::Matrix<T, Eigen::Dynamic, 1> result;
  for (int i = 0; i < n; ++i) {
    float base = is_x_axis ? pos_vec[i].x() : pos_vec[i].y();
    float p_b_j = 1.0;
    for (int j = 0; j <= order; ++j) {
      A(i, j) = p_b_j;
      p_b_j *= base;
    }
    y(i) = is_x_axis ? pos_vec[i].y() : pos_vec[i].x();
  }

  // solve linear least squares
  result = A.householderQr().solve(y);
  assert(result.size() == order + 1);

  for (int j = 0; j <= MAX_POLY_ORDER; ++j) {
    (*coeff)(j) = (j <= order) ? result(j) : static_cast<T>(0);
  }

  return true;
}

// @brief: evaluate y value of given x for a polynomial function
template <typename T = ScalarType>
T PolyEval(const T &x, const int &order,
           const Eigen::Matrix<T, MAX_POLY_ORDER + 1, 1> &coeff) {
  int poly_order = order;
  if (order > MAX_POLY_ORDER) {
    AERROR << "the order of polynomial function must be smaller than "
           << MAX_POLY_ORDER;
    AINFO << "forcing polynomial order to " << MAX_POLY_ORDER;
    poly_order = MAX_POLY_ORDER;
  }

  T y = static_cast<T>(0);
  float p_x_j = 1.0;
  for (int j = 0; j <= poly_order; ++j) {
    y += coeff(j) * p_x_j;
    p_x_j *= x;
  }

  return y;
}

// @brief: evaluating y value of given x for a third-order polynomial function
template <typename T = float>
T GetPolyValue(T a, T b, T c, T d, T x) {
  T y = d;
  T v = x;
  y += (c * v);
  v *= x;
  y += (b * v);
  v *= x;
  y += (a * v);
  return y;
}

// @brief: non mask class which is used for filtering out the markers inside the
// polygon mask
class NonMask {
 public:
  NonMask() {}
  explicit NonMask(const size_t n) { polygon_.reserve(n); }

  void AddPolygonPoint(const ScalarType &x, const ScalarType &y);
  bool IsInsideMask(const Vector2D &p) const;

 protected:
  int ComputeOrientation(const Vector2D &p1, const Vector2D &p2,
                         const Vector2D &q) const;
  bool IsColinear(const Vector2D &p1, const Vector2D &p2,
                  const Vector2D &q) const;
  bool IsOnLineSegmentWhenColinear(const Vector2D &p1, const Vector2D &p2,
                                   const Vector2D &q) const;
  bool IsLineSegmentIntersect(const Vector2D &p1, const Vector2D &p2,
                              const Vector2D &p3, const Vector2D &p4) const;

 private:
  std::vector<Vector2D> polygon_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_UTIL_H_
