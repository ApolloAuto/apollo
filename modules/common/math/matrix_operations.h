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

#ifndef MODULES_COMMON_MATH_MATRIX_OPERATIONS_H_
#define MODULES_COMMON_MATH_MATRIX_OPERATIONS_H_

#include <utility>
#include "Eigen/Dense"
#include "Eigen/SVD"

/**
 * @namespace apollo::common::math
 * @brief The math namespace deals with a number of useful mathematical objects.
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a given square matrix,
 * rounding all eigenvalues with absolute value bounded by epsilon to zero.
 *
 * @param m The square matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 *
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int N>
Eigen::Matrix<T, N, N> PseudoInverse(const Eigen::Matrix<T, N, N> &m,
                                     const double epsilon = 1.0e-6) {
  Eigen::JacobiSVD<Eigen::Matrix<T, N, N>> svd(
      m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.matrixV() *
         (svd.singularValues().array().abs() > epsilon)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a given matrix,
 * rounding all eigenvalues with absolute value bounded by epsilon to zero.
 *
 * @param m The matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 *
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int M, unsigned int N>
Eigen::Matrix<T, N, M> PseudoInverse(const Eigen::Matrix<T, M, N>& m,
        const double epsilon = 1.0e-6) {
    Eigen::Matrix<T, M, M> t = m * m.transpose();
    return m.transpose() * PseudoInverse<T, M>(t);
}

/**
 * @brief Implements Tustin's method for converting transfer functions from
 * continuous to discrete time domains.
 * https://en.wikipedia.org/wiki/Bilinear_transform
 *
 * @param m_c Matrix
 * @param ts Time interval
 *
 * @return Matrix
 */
template <typename T, unsigned int N>
Eigen::Matrix<T, N, N> ContinuousToDiscrete(const Eigen::Matrix<T, N, N> &m_c,
                                            const double ts) {
  Eigen::Matrix<T, N, N> m_identity = Eigen::Matrix<T, N, N>::Identity();
  Eigen::Matrix<T, N, N> m_d = (m_identity + ts * 0.5 * m_c) *
                               PseudoInverse<T, N>(m_identity - ts * 0.5 * m_c);
  return m_d;
}

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif /* MODULES_COMMON_MATH_MATRIX_OPERATIONS_H_ */
