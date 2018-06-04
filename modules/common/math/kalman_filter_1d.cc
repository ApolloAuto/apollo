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
  Eigen::Matrix<float, 2, 1> state_x;
  state_x << x, 0.0f;

  Eigen::Matrix<float, 2, 2> p;
  p.setIdentity();
  p *= 20.0f;

  SetStateEstimate(state_x, p);

  Eigen::Matrix<float, 2, 2> f;
  f << 1.0f, 0.033f, 0.0f, 1.0f;
  SetTransitionMatrix(f);

  Eigen::Matrix<float, 1, 2> h;
  h << 1.0f, 0.0f;
  SetObservationMatrix(h);

  Eigen::Matrix<float, 2, 2> q;
  q.setIdentity();
  q *= 2.0f;
  SetTransitionNoise(q);

  Eigen::Matrix<float, 1, 1> r;
  r.setIdentity();
  r *= 20.0f;
  SetObservationNoise(r);

  Eigen::Matrix<float, 2, 1> b;
  b.setZero();
  SetControlMatrix(b);

  return true;
}

bool KalmanFilter1D::Predict(const float& time_diff) {
  Eigen::Matrix<float, 2, 2> f =
      KalmanFilter<float, 2, 1, 1>::GetTransitionMatrix();
  f(0, 1) = time_diff;
  SetTransitionMatrix(f);
  KalmanFilter<float, 2, 1, 1>::Predict();

  return true;
}

bool KalmanFilter1D::Update(const float& z) {
  Eigen::Matrix<float, 1, 1> state_z;
  state_z << z;
  Correct(state_z);
  return true;
}

Eigen::Vector2f KalmanFilter1D::GetState() { return GetStateEstimate(); }

Eigen::Matrix2f KalmanFilter1D::GetCov() { return GetStateCovariance(); }

}  // namespace math
}  // namespace common
}  // namespace apollo
