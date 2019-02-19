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

/**
 * @file
 * @brief Defines the templated extended Kalman filter class.
 */

#pragma once

#include <functional>
#include <utility>

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
 * @class ExtendedKalmanFilter
 *
 * @brief Implements a discrete-time extended Kalman filter.
 *
 * @param XN dimension of state
 * @param ZN dimension of observations
 * @param UN dimension of controls
 */
template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
class ExtendedKalmanFilter {
 public:
  /**
   * @brief Constructor which defers initialization until the initial state
   * distribution parameters are set (with SetStateEstimate),
   * typically on the first observation
   */
  ExtendedKalmanFilter() {
    F_.setIdentity();
    Q_.setZero();
    H_.setIdentity();
    R_.setZero();

    x_.setZero();
    P_.setZero();
    y_.setZero();
    S_.setZero();
    K_.setZero();
  }

  /**
   * @brief Constructor which fully initializes the extended Kalman filter
   * @param x Mean of the state belief distribution
   * @param P Covariance of the state belief distribution
   */
  ExtendedKalmanFilter(const Eigen::Matrix<T, XN, 1> &x,
                       const Eigen::Matrix<T, XN, XN> &P)
      : ExtendedKalmanFilter() {
    SetStateEstimate(x, P);
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
  }

  /**
   * @brief Destructor
   */
  virtual ~ExtendedKalmanFilter() {}

  /**
   * @brief Changes the system transition function under zero control.
   *
   * @param F New transition matrix
   */
  void SetTransitionModel(
      std::function<Eigen::Matrix<T, XN, 1>(const Eigen::Matrix<T, XN, 1> &,
                                            const Eigen::Matrix<T, UN, 1> &)>
          f,
      const Eigen::Matrix<T, XN, XN> &F) {
    f_ = std::move(f);
    F_ = F;
  }

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
  void SetObservationModel(
      std::function<Eigen::Matrix<T, ZN, 1>(const Eigen::Matrix<T, XN, 1> &)> h,
      const Eigen::Matrix<T, ZN, XN> &H) {
    h_ = std::move(h);
    H_ = H;
  }

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
   * @brief Get the covariance matrix of the transition noise.
   *
   * @return Covariance matrix
   */
  const Eigen::Matrix<T, XN, XN> &GetTransitionNoise() const { return Q_; }

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
  Eigen::Matrix<T, XN, 1> GetStateMean() const { return x_; }

  /**
   * @brief Gets covariance of our current state belief distribution
   *
   * @return Covariance matrix
   */
  Eigen::Matrix<T, XN, XN> GetStateCovariance() const { return P_; }

 private:
  // Mean of current state belief distribution
  Eigen::Matrix<T, XN, 1> x_;

  // Covariance of current state belief dist
  Eigen::Matrix<T, XN, XN> P_;

  // Transition function
  std::function<Eigen::Matrix<T, XN, 1>(const Eigen::Matrix<T, XN, 1> &,
                                        const Eigen::Matrix<T, UN, 1> &)>
      f_;

  // State transition matrix under zero control
  Eigen::Matrix<T, XN, XN> F_;

  // Covariance of the state transition noise
  Eigen::Matrix<T, XN, XN> Q_;

  // Observation function
  std::function<Eigen::Matrix<T, ZN, 1>(const Eigen::Matrix<T, XN, 1> &)> h_;

  // Observation matrix
  Eigen::Matrix<T, ZN, XN> H_;

  // Covariance of observation noise
  Eigen::Matrix<T, ZN, ZN> R_;

  // Innovation; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, ZN, 1> y_;

  // Innovation covariance; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, ZN, ZN> S_;

  // Kalman gain; marked as member to prevent memory re-allocation.
  Eigen::Matrix<T, XN, ZN> K_;
};

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void ExtendedKalmanFilter<T, XN, ZN, UN>::Predict(
    const Eigen::Matrix<T, UN, 1> &u) {
  x_ = f_(x_, u);

  P_ = F_ * P_ * F_.transpose() + Q_;
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void ExtendedKalmanFilter<T, XN, ZN, UN>::Correct(
    const Eigen::Matrix<T, ZN, 1> &z) {
  y_ = z - h_(x_);

  S_ = H_ * P_ * H_.transpose() + R_;

  K_ = P_ * H_.transpose() * PseudoInverse<T, ZN>(S_);

  x_ = x_ + K_ * y_;

  P_ = (Eigen::Matrix<T, XN, XN>::Identity() - K_ * H_) * P_;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
