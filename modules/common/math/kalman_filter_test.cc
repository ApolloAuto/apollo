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

#include "modules/common/math/kalman_filter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(KalmanFilterTest, SyntheticTracking) {
  // a synthetic example of vehicle position and velocity tracking.
  // state: position, linear velocity
  // observation: linear velocity from speedometer
  // control input: throttle or brake (linear acceleration)
  Eigen::Vector2d x;
  x[0] = 0.0;
  x[1] = 10.0;

  Eigen::Matrix2d P;
  P.setZero();
  P(0, 0) = 1.0;
  P(1, 1) = 0.1;

  double dt = 0.1;
  // transition matrix
  Eigen::Matrix2d F;
  F(0, 0) = 1.0;
  F(0, 1) = dt;
  F(1, 0) = 0.0;
  F(1, 1) = 1.0;

  // transition noise covariance
  Eigen::Matrix2d Q;
  Q.setZero();
  Q(0, 0) = 0.05;
  Q(1, 1) = 0.1;

  // observation matrix
  Eigen::Matrix<double, 1, 2> H;
  H(0, 0) = 0.0;
  H(0, 1) = 1.0;

  // observation noise covariance
  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = 0.5 * 0.5;

  // control matrix
  Eigen::Vector2d B;
  B[0] = 0.5 * dt * dt;
  B[1] = dt;

  KalmanFilter<double, 2, 1, 1> kf(x, P);
  kf.SetTransitionMatrix(F);
  kf.SetTransitionNoise(Q);
  kf.SetObservationMatrix(H);
  kf.SetObservationNoise(R);
  kf.SetControlMatrix(B);

  kf.Predict();
  Eigen::Vector2d x_predict = kf.GetStateEstimate();
  Eigen::Matrix2d P_predict = kf.GetStateCovariance();
  EXPECT_DOUBLE_EQ(1.0, x_predict[0]);
  EXPECT_DOUBLE_EQ(10.0, x_predict[1]);

  EXPECT_DOUBLE_EQ(1.051, P_predict(0, 0));
  EXPECT_DOUBLE_EQ(0.01, P_predict(0, 1));
  EXPECT_DOUBLE_EQ(0.01, P_predict(1, 0));
  EXPECT_DOUBLE_EQ(0.2, P_predict(1, 1));

  Eigen::Matrix<double, 1, 1> z;
  z(0, 0) = 10.0;
  kf.Correct(z);

  Eigen::Vector2d x_correct = kf.GetStateEstimate();
  Eigen::Matrix2d P_correct = kf.GetStateCovariance();

  EXPECT_DOUBLE_EQ(1.0, x_correct[0]);
  EXPECT_DOUBLE_EQ(10.0, x_correct[1]);

  EXPECT_NEAR(1.05078, P_correct(0, 0), 0.001);
  EXPECT_NEAR(0.00556, P_correct(0, 1), 0.001);
  EXPECT_NEAR(0.00556, P_correct(1, 0), 0.001);
  EXPECT_NEAR(0.11111, P_correct(1, 1), 0.001);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
