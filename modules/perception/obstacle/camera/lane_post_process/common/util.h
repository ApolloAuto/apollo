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

#include <Eigen/Core>
#include <Eigen/QR>

#include <cmath>
#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_UTIL_H_

namespace apollo {
namespace perception {

// @brief: convert angle from the range of [-pi, pi] to [0, 2*pi]
inline void rect_angle(ScalarType* theta) {
  if (*theta < 0) {
    (*theta) += static_cast<ScalarType>(2 * M_PI);
  }
}

// @brief: fit polynomial function with QR decomposition (using Eigen 3)
template <typename T = ScalarType>
bool poly_fit(const std::vector<Eigen::Matrix<T, 2, 1>>& pos_vec,
              const int& order, Eigen::Matrix<T, MAX_POLY_ORDER + 1, 1>* coeff,
              const bool& is_x_axis = true) {
  if (coeff == NULL) {
    AERROR << "The coefficient pointer is NULL.";
    return false;
  }

  if (order > MAX_POLY_ORDER) {
    AERROR << "The order of polynomial must be smaller than "
           << MAX_POLY_ORDER;
    return false;
  }

  int n = static_cast<int>(pos_vec.size());
  if (n <= order) {
    AERROR
        << "The number of points should be larger than the order. #points = "
        << pos_vec.size();
    return false;
  }

  // create data matrix
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A(n, order + 1);
  Eigen::Matrix<T, Eigen::Dynamic, 1> y(n);
  Eigen::Matrix<T, Eigen::Dynamic, 1> result;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j <= order; ++j) {
      A(i, j) = std::pow(is_x_axis ? pos_vec[i].x() : pos_vec[i].y(), j);
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
T poly_eval(const T& x, const int& order,
            const Eigen::Matrix<T, MAX_POLY_ORDER + 1, 1>& coeff) {
  int poly_order = order;
  if (order > MAX_POLY_ORDER) {
    AERROR << "the order of polynomial function must be smaller than "
           << MAX_POLY_ORDER;
    AINFO << "forcing polynomial order to " << MAX_POLY_ORDER;
    poly_order = MAX_POLY_ORDER;
  }

  T y = static_cast<T>(0);
  for (int j = 0; j <= poly_order; ++j) {
    y += coeff(j) * std::pow(x, j);
  }

  return y;
}

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_LANE_POST_PROCESS_COMMON_UTIL_H_
