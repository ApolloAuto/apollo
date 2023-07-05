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
 * @brief Defines some useful matrix operations.
 */

#include <cmath>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/SVD"

#include "modules/common/math/matrix_operations.h"

/**
 * @namespace apollo::common::math
 * @brief The math namespace deals with a number of useful mathematical objects.
 */
namespace apollo {
namespace common {
namespace math {

bool ContinuousToDiscrete(const Eigen::MatrixXd &m_a,
                          const Eigen::MatrixXd &m_b,
                          const Eigen::MatrixXd &m_c,
                          const Eigen::MatrixXd &m_d, const double ts,
                          Eigen::MatrixXd *ptr_a_d, Eigen::MatrixXd *ptr_b_d,
                          Eigen::MatrixXd *ptr_c_d, Eigen::MatrixXd *ptr_d_d) {
  if (ts <= 0.0) {
    AERROR << "ContinuousToDiscrete : ts is less than or equal to zero";
    return false;
  }

  // Only matrix_a is mandatory to be non-zeros in matrix
  // conversion.
  if (m_a.rows() == 0) {
    AERROR << "ContinuousToDiscrete: matrix_a size 0 ";
    return false;
  }

  if (m_a.cols() != m_b.rows() || m_b.cols() != m_d.cols() ||
      m_c.rows() != m_d.rows() || m_a.cols() != m_c.cols()) {
    AERROR << "ContinuousToDiscrete: matrix dimensions mismatch";
    return false;
  }

  Eigen::MatrixXd m_identity =
      Eigen::MatrixXd::Identity(m_a.cols(), m_a.rows());

  *ptr_a_d =
      (m_identity - ts * 0.5 * m_a).inverse() * (m_identity + ts * 0.5 * m_a);

  *ptr_b_d = std::sqrt(ts) * (m_identity - ts * 0.5 * m_a).inverse() * m_b;

  *ptr_c_d = std::sqrt(ts) * m_c * (m_identity - ts * 0.5 * m_a).inverse();

  *ptr_d_d = 0.5 * m_c * (m_identity - ts * 0.5 * m_a).inverse() * m_b + m_d;

  return true;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
