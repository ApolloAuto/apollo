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

#include "modules/common/math/kalman_filter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

class KalmanFilterTest : public ::testing::Test {
 public:
  KalmanFilterTest() : kf() {}

  virtual void SetUp() {
    // Initial state
    Eigen::Matrix<double, 2, 1> x;
    x(0, 0) = 0.0;
    x(1, 0) = 1.0;

    // Initial state belief covariance
    Eigen::Matrix<double, 2, 2> P;
    P.setZero();
    P(0, 0) = 0.1;
    P(1, 1) = 0.1;

    // Transition matrix
    Eigen::Matrix<double, 2, 2> F;
    F.setZero();
    F(0, 0) = 1.0;
    F(0, 1) = 1.0;
    F(1, 1) = 1.0;

    // Transition noise covariance
    Eigen::Matrix<double, 2, 2> Q;
    Q.setZero();
    Q(0, 0) = 0.01;
    Q(1, 1) = 0.01;

    // Observation matrix
    Eigen::Matrix<double, 1, 2> H;
    H.setIdentity();

    // Observation noise covariance
    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = 0.5 * 0.5;

    // Control matrix
    Eigen::Matrix<double, 2, 1> B;
    B[0] = 0.5 * 1.0 * 1.0;
    B[1] = 1.0;

    kf.SetStateEstimate(x, P);
    kf.SetTransitionMatrix(F);
    kf.SetTransitionNoise(Q);
    kf.SetObservationMatrix(H);
    kf.SetObservationNoise(R);
    kf.SetControlMatrix(B);
  }

 protected:
  KalmanFilter<double, 2, 1, 1> kf;
};

TEST_F(KalmanFilterTest, SyntheticTrackingTest) {
  kf.Predict();
  Eigen::Matrix<double, 2, 1> state = kf.GetStateEstimate();
  Eigen::Matrix<double, 2, 2> state_cov = kf.GetStateCovariance();
  EXPECT_DOUBLE_EQ(1.0, state(0, 0));
  EXPECT_DOUBLE_EQ(1.0, state(1, 0));
  EXPECT_NEAR(0.21, state_cov(0, 0), 0.001);
  EXPECT_NEAR(0.10, state_cov(0, 1), 0.001);
  EXPECT_NEAR(0.10, state_cov(1, 0), 0.001);
  EXPECT_NEAR(0.11, state_cov(1, 1), 0.001);

  Eigen::Matrix<double, 1, 1> z;
  z(0, 0) = 1.0;
  kf.Correct(z);
  state = kf.GetStateEstimate();
  state_cov = kf.GetStateCovariance();

  EXPECT_DOUBLE_EQ(1.0, state(0, 0));
  EXPECT_DOUBLE_EQ(1.0, state(1, 0));
  EXPECT_NEAR(0.11413, state_cov(0, 0), 0.001);
  EXPECT_NEAR(0.05348, state_cov(0, 1), 0.001);
  EXPECT_NEAR(0.05348, state_cov(1, 0), 0.001);
  EXPECT_NEAR(0.08826, state_cov(1, 1), 0.001);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
