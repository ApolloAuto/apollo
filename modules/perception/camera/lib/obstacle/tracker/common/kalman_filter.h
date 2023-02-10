/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "modules/common/util/eigen_defs.h"

namespace apollo {
namespace perception {
namespace camera {

class KalmanFilterConstVelocity {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  KalmanFilterConstVelocity();

  void Init(const Eigen::VectorXd &x);

  void Predict(float delta_t);

  void Correct(const Eigen::VectorXd &z);

  Eigen::Vector4d get_state() const;
  void MagicPosition(const Eigen::VectorXd &pos);
  void MagicVelocity(const Eigen::VectorXd &vel);
  Eigen::Matrix4d variance_;
  Eigen::Matrix2d measure_noise_;
  Eigen::Matrix4d process_noise_;

  Eigen::Matrix<double, 4, 1> predict_state_;
  double likelihood_ = 1.0;

 protected:
  // x = (x,y,vx,vy)'
  // z = (x,y)'
  // X = A*x
  // z = H*x
  Eigen::Matrix4d state_transition_matrix_;

  Eigen::Matrix<double, 4, 1> state_;
  Eigen::Matrix<double, 2, 4> measure_matrix_;
  Eigen::Matrix<double, 4, 2> kalman_gain_;
  bool inited_;
};

template <std::size_t N>
class KalmanFilterConstState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  typedef Eigen::Matrix<double, N, N> MatrixNd;
  typedef Eigen::Matrix<double, N, 1> VectorNd;

 public:
  KalmanFilterConstState() = default;
  ~KalmanFilterConstState() = default;

  bool Init(const VectorNd &param);

  bool Predict(float delta_t);

  void Correct(const VectorNd &measurement);

  VectorNd get_state() const { return state_; }

  MatrixNd covariance_;     // P
  MatrixNd measure_noise_;  // R
  MatrixNd process_noise_;  // Q

  VectorNd predict_state_;
  VectorNd residual_;
  double likelihood_ = 1.0;

 protected:
  // x = (x, y)
  // z = (x, y)
  bool inited_ = false;

  VectorNd state_;
  MatrixNd kalman_gain_;
};

class ExtendedKalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  ExtendedKalmanFilter() : inited_(false) {}
  void Init();

  void Init(const Eigen::VectorXd &x);

  void Predict(float delta_t);

  void Correct(const Eigen::VectorXd &z);

  Eigen::Vector4d get_state() const;

  Eigen::Matrix4d variance_;
  Eigen::Matrix3d measure_noise_;
  Eigen::Matrix4d process_noise_;

 protected:
  // X = (x,y,v,theta)'
  // x = x + v cos(theta)
  // y = y + v sin(theta)
  // v = v + noise
  // theta = theta + noise
  // z = (x,y,theta)'
  Eigen::Matrix4d state_transition_matrix_;

  Eigen::Matrix<double, 4, 1> state_;
  Eigen::Matrix<double, 3, 4> measure_matrix_;
  Eigen::Matrix<double, 4, 3> kalman_gain_;
  bool inited_;
};

class FirstOrderRCLowPassFilter {
 public:
  FirstOrderRCLowPassFilter() : alpha_(0), inited_(false) {}
  void SetAlpha(float alpha);
  void AddMeasure(const Eigen::VectorXd &z);
  Eigen::VectorXd get_state() const;

 private:
  float alpha_;
  bool inited_;
  Eigen::VectorXd state_;
};

class MaxNMeanFilter {
 public:
  MaxNMeanFilter() : index_(0), window_(1) {}
  void SetWindow(int window);

  void AddMeasure(const Eigen::VectorXd &z);

  void Clear();

  Eigen::VectorXd get_state() const;

 private:
  apollo::common::EigenVector<Eigen::VectorXd> measures_;
  int index_;
  int window_;
};

class MeanFilter {
 public:
  MeanFilter() : index_(0), window_(1) {}
  void SetWindow(int window);

  void AddMeasure(const Eigen::VectorXd &z);

  const Eigen::VectorXd &get_state() const;
  const Eigen::MatrixXd &get_variance() const;
  int size() const { return static_cast<int>(measures_.size()); }

 private:
  apollo::common::EigenVector<Eigen::VectorXd> measures_;
  Eigen::VectorXd state_;
  Eigen::MatrixXd variance_;
  int index_;
  int window_;
};

// [BEGIN] KalmanFilterConstState
template <std::size_t N>
bool KalmanFilterConstState<N>::Init(const VectorNd &param) {
  state_ = param;
  inited_ = true;

  return true;
}

template <std::size_t N>
bool KalmanFilterConstState<N>::Predict(float delta_t) {
  if (!inited_) {
    return false;
  }
  predict_state_ = state_;
  // state_transition_matrix is Identity
  covariance_ += process_noise_;

  return true;
}

template <std::size_t N>
void KalmanFilterConstState<N>::Correct(const VectorNd &measurement) {
  if (!inited_) {
    Init(measurement);
  } else {
    auto measurements_cov = covariance_ + measure_noise_;
    kalman_gain_ = covariance_ * measurements_cov.inverse();
    state_ = state_ + kalman_gain_ * (measurement - state_);
    covariance_ = (MatrixNd::Identity() - kalman_gain_) * covariance_;

    // compute likelihood
    residual_ = measurement - predict_state_;
    // Ref: https://eigen.tuxfamily.org/bz/show_bug.cgi?id=1610
    double kval = -0.5 * residual_.transpose().adjoint().dot(
                             measurements_cov.inverse() * residual_);
    likelihood_ =
        std::exp(kval) / std::sqrt(2 * M_PI * measurements_cov.determinant());
  }
}
// [END] KalmanFilterConstState

}  // namespace camera
}  // namespace perception
}  // namespace apollo
