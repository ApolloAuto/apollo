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
 * @brief Defines the templated KalmanFilter class.
 */

#pragma once

#include <sstream>
#include <string>

#include "Eigen/Dense"

#include "cyber/common/log.h"
#include "modules/common/math/matrix_operations.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @class KalmanFilter
 *
 * @brief Implements a discrete-time Kalman filter.
 *
 * @param XN dimension of state
 * @param ZN dimension of observations
 * @param UN dimension of controls
 */
template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
class KalmanFilter {
 public:
  /**
   * @brief Constructor which defers initialization until the initial state
   * distribution parameters are set (with SetStateEstimate),
   * typically on the first observation
   */
  KalmanFilter() {
    F_.setIdentity();
    Q_.setZero();
    H_.setIdentity();
    R_.setZero();
    B_.setZero();

    x_.setZero();
    P_.setZero();
    y_.setZero();
    S_.setZero();
    K_.setZero();
  }

  /**
   * @brief Sets the initial state belief distribution.
   *
   * @param x Mean of the state belief distribution
   * @param P Covariance of the state belief distribution
   */
  void SetStateEstimate(const Eigen::Matrix<T, XN, 1> &x,
                        const Eigen::Matrix<T, XN, XN> &P) {
    x_ = x;
    P_ = P;
    is_initialized_ = true;
  }

  /**
   * @brief Constructor which fully initializes the Kalman filter
   * @param x Mean of the state belief distribution
   * @param P Covariance of the state belief distribution
   */
  KalmanFilter(const Eigen::Matrix<T, XN, 1> &x,
               const Eigen::Matrix<T, XN, XN> &P)
      : KalmanFilter() {
    SetStateEstimate(x, P);
  }

  /**
   * @brief Destructor
   */
  virtual ~KalmanFilter() {}

  /**
   * @brief Changes the system transition function under zero control.
   *
   * @param F New transition matrix
   */
  void SetTransitionMatrix(const Eigen::Matrix<T, XN, XN> &F) { F_ = F; }

  /**
   * @brief Changes the covariance matrix of the transition noise.
   *
   * @param Q New covariance matrix
   */
  void SetTransitionNoise(const Eigen::Matrix<T, XN, XN> &Q) { Q_ = Q; }

  /**
   * @brief Changes the observation matrix, which maps states to observations.
   *
   * @param H New observation matrix
   */
  void SetObservationMatrix(const Eigen::Matrix<T, ZN, XN> &H) { H_ = H; }

  /**
   * @brief Changes the covariance matrix of the observation noise.
   *
   * @param R New covariance matrix
   */
  void SetObservationNoise(const Eigen::Matrix<T, ZN, ZN> &R) { R_ = R; }

  /**
   * @brief Changes the covariance matrix of current state belief distribution.
   *
   * @param P New state covariance matrix
   */
  void SetStateCovariance(const Eigen::Matrix<T, XN, XN> &P) { P_ = P; }

  /**
   * @brief Changes the control matrix in the state transition rule.
   *
   * @param B New control matrix
   */
  void SetControlMatrix(const Eigen::Matrix<T, XN, UN> &B) { B_ = B; }

  /**
   * @brief Get the system transition function under zero control.
   *
   * @return Transition matrix.
   */
  const Eigen::Matrix<T, XN, XN> &GetTransitionMatrix() const { return F_; }

  /**
   * @brief Get the covariance matrix of the transition noise.
   *
   * @return Covariance matrix
   */
  const Eigen::Matrix<T, XN, XN> &GetTransitionNoise() const { return Q_; }

  /**
   * @brief Get the observation matrix, which maps states to observations.
   *
   * @return Observation matrix
   */
  const Eigen::Matrix<T, ZN, XN> &GetObservationMatrix() const { return H_; }

  /**
   * @brief Get the covariance matrix of the observation noise.
   *
   * @return Covariance matrix
   */
  const Eigen::Matrix<T, ZN, ZN> &GetObservationNoise() const { return R_; }

  /**
   * @brief Get the control matrix in the state transition rule.
   *
   * @return Control matrix
   */
  const Eigen::Matrix<T, XN, UN> &GetControlMatrix() const { return B_; }

  /**
   * @brief Updates the state belief distribution given the control input u.
   *
   * @param u Control input (by default, zero)
   */
  void Predict(
      const Eigen::Matrix<T, UN, 1> &u = Eigen::Matrix<T, UN, 1>::Zero());

  /**
   * @brief Updates the state belief distribution given an observation z.
   *
   * @param z Observation
   */
  void Correct(const Eigen::Matrix<T, ZN, 1> &z);

  /**
   * @brief Gets mean of our current state belief distribution
   *
   * @return State vector
   */
  Eigen::Matrix<T, XN, 1> GetStateEstimate() const { return x_; }

  /**
   * @brief Gets covariance of our current state belief distribution
   *
   * @return Covariance matrix
   */
  Eigen::Matrix<T, XN, XN> GetStateCovariance() const { return P_; }

  /**
   * @brief Gets debug string containing detailed information about the filter.
   *
   * @return Debug string
   */
  std::string DebugString() const;

  /**
   * @brief Get initialization state of the filter
   * @return True if the filter is initialized
   */
  bool IsInitialized() const { return is_initialized_; }

 private:
  // Mean of current state belief distribution
  Eigen::Matrix<T, XN, 1> x_;

  // Covariance of current state belief dist
  Eigen::Matrix<T, XN, XN> P_;

  // State transition matrix under zero control
  Eigen::Matrix<T, XN, XN> F_;

  // Covariance of the state transition noise
  Eigen::Matrix<T, XN, XN> Q_;

  // Observation matrix
  Eigen::Matrix<T, ZN, XN> H_;

  // Covariance of observation noise
  Eigen::Matrix<T, ZN, ZN> R_;

  // Control matrix in state transition rule
  Eigen::Matrix<T, XN, UN> B_;

  // Innovation; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, ZN, 1> y_;

  // Innovation covariance; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, ZN, ZN> S_;

  // Kalman gain; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, XN, ZN> K_;

  // true iff SetStateEstimate has been called.
  bool is_initialized_ = false;
};

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::Predict(
    const Eigen::Matrix<T, UN, 1> &u) {
  ACHECK(is_initialized_);

  x_ = F_ * x_ + B_ * u;

  P_ = F_ * P_ * F_.transpose() + Q_;
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::Correct(
    const Eigen::Matrix<T, ZN, 1> &z) {
  ACHECK(is_initialized_);
  y_ = z - H_ * x_;

  S_ = static_cast<Eigen::Matrix<T, ZN, ZN>>(H_ * P_ * H_.transpose() + R_);

  K_ = static_cast<Eigen::Matrix<T, XN, ZN>>(P_ * H_.transpose() *
                                             PseudoInverse<T, ZN>(S_));

  x_ = x_ + K_ * y_;

  P_ = static_cast<Eigen::Matrix<T, XN, XN>>(
      (Eigen::Matrix<T, XN, XN>::Identity() - K_ * H_) * P_);
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline std::string KalmanFilter<T, XN, ZN, UN>::DebugString() const {
  Eigen::IOFormat clean_fmt(4, 0, ", ", " ", "[", "]");
  std::stringstream ss;
  ss << "F = " << F_.format(clean_fmt) << "\n"
     << "B = " << B_.format(clean_fmt) << "\n"
     << "H = " << H_.format(clean_fmt) << "\n"
     << "Q = " << Q_.format(clean_fmt) << "\n"
     << "R = " << R_.format(clean_fmt) << "\n"
     << "x = " << x_.format(clean_fmt) << "\n"
     << "P = " << P_.format(clean_fmt) << "\n";
  return ss.str();
}

}  // namespace math
}  // namespace common
}  // namespace apollo
