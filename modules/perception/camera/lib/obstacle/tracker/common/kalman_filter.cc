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
#include "modules/perception/camera/lib/obstacle/tracker/common/kalman_filter.h"

#include <algorithm>

#include "Eigen/LU"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

KalmanFilterConstVelocity::KalmanFilterConstVelocity() {
  // other value should be changed in predict
  state_transition_matrix_.setIdentity();
  measure_matrix_ << 1, 0, 0, 0, 0, 1, 0, 0;
  variance_.setIdentity();
  measure_noise_.setIdentity();
  process_noise_.setIdentity();
  inited_ = false;
}

void KalmanFilterConstVelocity::Init(Eigen::VectorXd x) {
  state_ << x(0), x(1), 0, 0;
  inited_ = true;
}

void KalmanFilterConstVelocity::Predict(float delta_t) {
  if (inited_) {
    state_transition_matrix_(0, 2) = delta_t;
    state_transition_matrix_(1, 3) = delta_t;
    state_ = state_transition_matrix_ * state_;
    predict_state_ = state_;
    variance_ = state_transition_matrix_ * variance_ *
                    state_transition_matrix_.transpose() +
                process_noise_;
  }
}
void KalmanFilterConstVelocity::MagicVelocity(const Eigen::VectorXd &vel) {
  state_(2) = vel(0);
  state_(3) = vel(1);
}
void KalmanFilterConstVelocity::Correct(const Eigen::VectorXd &z) {
  if (inited_) {
    Eigen::Vector2d measure;
    measure << z[0], z[1];
    // measurement covariance: S = H*P*H^T + R
    Eigen::Matrix2d cov =
        measure_matrix_ * variance_ * measure_matrix_.transpose() +
        measure_noise_;

    kalman_gain_ = variance_ * measure_matrix_.transpose() * cov.inverse();
    variance_ = variance_ - kalman_gain_ * measure_matrix_ * variance_;
    state_ = state_ + kalman_gain_ * (measure - measure_matrix_ * state_);

    // compute likelihood
    auto residual = measure - predict_state_.head(2);
    likelihood_ =
        std::exp(-0.5 * residual.transpose() * cov.inverse() * residual) /
        std::sqrt(2 * M_PI * cov.determinant());
  } else {
    Init(z);
  }
}

Eigen::Vector4d KalmanFilterConstVelocity::get_state() const { return state_; }
void KalmanFilterConstVelocity::MagicPosition(const Eigen::VectorXd &pos) {
  state_(0) = pos(0);
  state_(1) = pos(1);
}

void ExtendedKalmanFilter::Init() {
  // other value should be changed in predict
  state_transition_matrix_.setIdentity();
  measure_matrix_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  variance_.setIdentity();
  measure_noise_.setIdentity();
  process_noise_.setIdentity();
  inited_ = false;
}

void ExtendedKalmanFilter::Init(Eigen::VectorXd x) {
  Init();
  state_ << x(0), x(1), 0, x(2);
  inited_ = true;
}

void ExtendedKalmanFilter::Predict(float delta_t) {
  if (inited_) {
    float sin_theta = static_cast<float>(std::sin(state_(3)));
    float cos_theta = static_cast<float>(std::cos(state_(3)));
    state_transition_matrix_(0, 2) = delta_t * cos_theta;
    state_transition_matrix_(0, 3) = -delta_t * state_(2) * sin_theta;
    state_transition_matrix_(1, 2) = delta_t * sin_theta;
    state_transition_matrix_(1, 3) = delta_t * state_(2) * cos_theta;
    state_(0) += state_(2) * cos_theta * delta_t;
    state_(1) += state_(2) * sin_theta * delta_t;
    //    No change
    //    state_(2) = state_(2);
    //    state_(3) = state_(3);
    variance_ = state_transition_matrix_ * variance_ *
                    state_transition_matrix_.transpose() +
                process_noise_;
  }
}

void ExtendedKalmanFilter::Correct(const Eigen::VectorXd &z) {
  if (inited_) {
    Eigen::Vector3d measure;
    measure << z[0], z[1], z[2];
    Eigen::Matrix3d cov =
        measure_matrix_ * variance_ * measure_matrix_.transpose() +
        measure_noise_;

    kalman_gain_ = variance_ * measure_matrix_.transpose() * cov.inverse();
    variance_ = variance_ - kalman_gain_ * measure_matrix_ * variance_;
    state_ = state_ + kalman_gain_ * (measure - measure_matrix_ * state_);
  } else {
    inited_ = true;
    state_ << z[0], z[1], 0, z[2];
  }
}

Eigen::Vector4d ExtendedKalmanFilter::get_state() const { return state_; }

void MeanFilter::SetWindow(int window) {
  window_ = window;
  index_ = 0;
}

void MeanFilter::AddMeasure(const Eigen::VectorXd &z) {
  if (measures_.size() < static_cast<unsigned int>(window_)) {
    measures_.push_back(z);
  } else {
    measures_[index_] = z;
  }
  index_ = (index_ + 1) % window_;
  int n = static_cast<int>(measures_.size());

  state_.resizeLike(z);
  state_.setConstant(0);
  for (int i = 0; i < n; ++i) {
    state_ += measures_[i];
  }
  state_ = state_ / n;

  variance_.resize(state_.rows(), state_.rows());
  variance_.setConstant(0);
  for (int i = 0; i < n; ++i) {
    auto z = measures_[i] - state_;
    variance_ += z * z.transpose();
  }
  if (n > 1) {
    variance_ /= n - 1;
  }
}

const Eigen::VectorXd &MeanFilter::get_state() const { return state_; }

const Eigen::MatrixXd &MeanFilter::get_variance() const { return variance_; }

void FirstOrderRCLowPassFilter::SetAlpha(float alpha) {
  alpha_ = alpha;
  inited_ = false;
}

void FirstOrderRCLowPassFilter::AddMeasure(const Eigen::VectorXd &z) {
  if (inited_) {
    state_ = z + alpha_ * (state_ - z);
  } else {
    state_ = z;
    inited_ = true;
  }
}

Eigen::VectorXd FirstOrderRCLowPassFilter::get_state() const { return state_; }

struct {
  bool operator()(Eigen::VectorXd a, Eigen::VectorXd b) const {
    return a[0] > b[0];
  }
} customLess;

void MaxNMeanFilter::SetWindow(int window) {
  window_ = window;
  index_ = 0;
}

void MaxNMeanFilter::AddMeasure(const Eigen::VectorXd &z) {
  measures_.push_back(z);
  std::sort(measures_.begin(), measures_.end(), customLess);
  if (measures_.size() > static_cast<unsigned int>(window_)) {
    measures_.resize(window_);
  }
}

Eigen::VectorXd MaxNMeanFilter::get_state() const {
  Eigen::VectorXd x = measures_[0];
  for (size_t i = 1; i < measures_.size(); ++i) {
    x += measures_[i];
  }
  x = x / static_cast<double>(measures_.size());
  return x;
}
void MaxNMeanFilter::Clear() { measures_.clear(); }
}  // namespace camera
}  // namespace perception
}  // namespace apollo
