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

#include "modules/common/math/kalman_filter_1d.h"

namespace apollo {
namespace common {
namespace math {

bool KalmanFilter1D::Init(const float& x) {
  x_ << x, 0.0f;
  F_ << 1.0f, 0.033f, 0.0f, 1.0f;
  H_ << 1.0f, 0.0f;

  // TODO(later) tune and put in config
  P_.setIdentity();
  P_ *= 20.0f;

  Q_.setIdentity();
  Q_ *= 20.0f;

  R_.setIdentity();
  R_ *= 20.0f;

  return true;
}

bool KalmanFilter1D::Predict(const float& time_diff) {
  F_(0, 1) = time_diff;

  x_ = F_ * x_;

  P_ = F_ * P_ * F_.transpose() + Q_;

  return true;
}

bool KalmanFilter1D::Update(const float& z) {
  z_.x() = z;

  y_ = z_ - H_ * x_;

  S_ = H_ * P_ * H_.transpose() + R_;

  K_ = P_ * H_.transpose() * S_.inverse();

  x_ = x_ + K_ * y_;

  P_ = P_ - K_ * H_ * P_;

  return true;
}

Eigen::Vector2f KalmanFilter1D::GetState() { return x_; }

Eigen::Matrix2f KalmanFilter1D::GetCov() { return P_; }

}  // namespace math
}  // namespace common
}  // namespace apollo
