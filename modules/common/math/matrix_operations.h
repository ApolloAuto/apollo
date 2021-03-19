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

#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/SVD"

#include "cyber/common/log.h"

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
  return static_cast<Eigen::Matrix<T, N, N>>(
      svd.matrixV() *
      (svd.singularValues().array().abs() > epsilon)
          .select(svd.singularValues().array().inverse(), 0)
          .matrix()
          .asDiagonal() *
      svd.matrixU().adjoint());
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
Eigen::Matrix<T, N, M> PseudoInverse(const Eigen::Matrix<T, M, N> &m,
                                     const double epsilon = 1.0e-6) {
  Eigen::Matrix<T, M, M> t = m * m.transpose();
  return static_cast<Eigen::Matrix<T, N, M>>(m.transpose() *
                                             PseudoInverse<T, M>(t));
}

/**
* @brief Computes bilinear transformation of the continuous to discrete form
for state space representation
* This assumes equation format of
*
*           dot_x = Ax + Bu
*           y = Cx + Du
*
*
*
* @param m_a, m_b, m_c, m_d are the state space matrix control matrix
*
* @return true or false.

 */

template <typename T, unsigned int L, unsigned int N, unsigned int O>
bool ContinuousToDiscrete(const Eigen::Matrix<T, L, L> &m_a,
                          const Eigen::Matrix<T, L, N> &m_b,
                          const Eigen::Matrix<T, O, L> &m_c,
                          const Eigen::Matrix<T, O, N> &m_d, const double ts,
                          Eigen::Matrix<T, L, L> *ptr_a_d,
                          Eigen::Matrix<T, L, N> *ptr_b_d,
                          Eigen::Matrix<T, O, L> *ptr_c_d,
                          Eigen::Matrix<T, O, N> *ptr_d_d) {
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

  Eigen::Matrix<T, L, L> m_identity = Eigen::Matrix<T, L, L>::Identity();
  *ptr_a_d = PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) *
             (m_identity + ts * 0.5 * m_a);

  *ptr_b_d =
      std::sqrt(ts) * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) * m_b;

  *ptr_c_d =
      std::sqrt(ts) * m_c * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a);

  *ptr_d_d =
      0.5 * m_c * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) * m_b + m_d;

  return true;
}

bool ContinuousToDiscrete(const Eigen::MatrixXd &m_a,
                          const Eigen::MatrixXd &m_b,
                          const Eigen::MatrixXd &m_c,
                          const Eigen::MatrixXd &m_d, const double ts,
                          Eigen::MatrixXd *ptr_a_d, Eigen::MatrixXd *ptr_b_d,
                          Eigen::MatrixXd *ptr_c_d, Eigen::MatrixXd *ptr_d_d);

template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N> &dense_matrix,
                      std::vector<T> *data, std::vector<D> *indices,
                      std::vector<D> *indptr) {
  static constexpr double epsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r < dense_matrix.rows(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < epsilon) {
        continue;
      }
      data->emplace_back(dense_matrix(r, c));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
