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
#include "gtest/gtest.h"
#define private public
#include "cyber/common/log.h"
#include "modules/perception/fusion/common/kalman_filter.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(FusionCommonTest, return_name_test) {
  KalmanFilter kal;
  EXPECT_EQ(kal.Name(), "KalmanFilter");
}

TEST(FusionCommonTest, test_init) {
  KalmanFilter kal;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  EXPECT_FALSE(kal.Init(states, uncertainty));
  states.setIdentity(3, 1);
  uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(kal.Init(states, uncertainty));
  uncertainty.setIdentity(4, 4);
  EXPECT_FALSE(kal.Init(states, uncertainty));
  states.setIdentity(4, 1);
  EXPECT_TRUE(kal.Init(states, uncertainty));
}

TEST(FusionCommonTest, test_predict) {
  KalmanFilter kal;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;
  EXPECT_FALSE(kal.Predict(transform_matrix, env_uncertainty));
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(kal.Init(states, uncertainty));
  transform_matrix.setIdentity(3, 4);
  EXPECT_FALSE(kal.Predict(transform_matrix, env_uncertainty));
  transform_matrix.setIdentity(4, 3);
  EXPECT_FALSE(kal.Predict(transform_matrix, env_uncertainty));
  transform_matrix.setIdentity(4, 4);
  env_uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(kal.Predict(transform_matrix, env_uncertainty));
  env_uncertainty.setIdentity(4, 3);
  EXPECT_FALSE(kal.Predict(transform_matrix, env_uncertainty));
  env_uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(kal.Predict(transform_matrix, env_uncertainty));
}

TEST(FusionCommonTest, test_correct) {
  KalmanFilter kal;
  Eigen::VectorXd obs_states;
  Eigen::MatrixXd obs_uncertainty;
  EXPECT_FALSE(kal.Correct(obs_states, obs_uncertainty));
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(kal.Init(states, uncertainty));
  obs_states.setIdentity(3, 1);
  EXPECT_FALSE(kal.Correct(obs_states, obs_uncertainty));
  obs_states.setIdentity(4, 1);
  obs_uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(kal.Correct(obs_states, obs_uncertainty));
  obs_uncertainty.setIdentity(4, 3);
  EXPECT_FALSE(kal.Correct(obs_states, obs_uncertainty));
  obs_uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(kal.Correct(obs_states, obs_uncertainty));
}

TEST(FusionCommonTest, test_breakdown_setting) {
  KalmanFilter kal;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(kal.Init(states, uncertainty));
  std::vector<bool> break_down = {1, 0, 0};
  EXPECT_FALSE(kal.SetGainBreakdownThresh(break_down, 1.0));
  EXPECT_FALSE(kal.SetValueBreakdownThresh(break_down, 1.0));
  break_down.push_back(1);
  EXPECT_TRUE(kal.SetGainBreakdownThresh(break_down, 1.0));
  EXPECT_TRUE(kal.SetValueBreakdownThresh(break_down, 1.0));
}

TEST(FusionCommonTest, test_decorrelation) {
  KalmanFilter kal;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(kal.Init(states, uncertainty));
  EXPECT_FALSE(kal.DeCorrelation(5, 5, 5, 5));
  EXPECT_TRUE(kal.DeCorrelation(0, 0, 3, 3));
}

TEST(FusionCommonTest, test_value) {
  KalmanFilter kal;
  double time_diff = 0.05;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  states.setIdentity(6, 1);
  uncertainty.setIdentity(6, 6);
  states(0) = 10.0;
  states(1) = 20.0;
  states(2) = 2.5;
  states(3) = 2.5;
  states(4) = 0.5;
  states(5) = 0.5;
  uncertainty(0, 2) = 0.5;
  uncertainty(1, 3) = 0.25;
  uncertainty(2, 0) = 0.5;
  uncertainty(3, 1) = 0.25;
  EXPECT_TRUE(kal.Init(states, uncertainty));
  Eigen::MatrixXd transform_matrix;
  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  transform_matrix(0, 4) = 0.5 * time_diff * time_diff;
  transform_matrix(1, 5) = 0.5 * time_diff * time_diff;
  transform_matrix(2, 4) = time_diff;
  transform_matrix(3, 5) = time_diff;
  Eigen::MatrixXd env_uncertainty;
  env_uncertainty.setIdentity(6, 6);
  env_uncertainty = env_uncertainty * 0.1 * time_diff;
  EXPECT_TRUE(kal.Predict(transform_matrix, env_uncertainty));
  Eigen::VectorXd ret_states = kal.GetStates();
  EXPECT_NEAR(ret_states(0), 10.125625, 1e-6);
  EXPECT_NEAR(ret_states(1), 20.125625, 1e-6);
  EXPECT_NEAR(ret_states(2), 2.525, 1e-6);
  EXPECT_NEAR(ret_states(3), 2.525, 1e-6);
  EXPECT_NEAR(ret_states(4), 0.5, 1e-6);
  EXPECT_NEAR(ret_states(5), 0.5, 1e-6);
  Eigen::VectorXd measurement;
  measurement.setIdentity(6, 1);
  measurement(0) = 10.5;
  measurement(1) = 20.5;
  measurement(2) = 2.5;
  measurement(3) = 2.5;
  measurement(4) = 0.5;
  measurement(5) = 0.5;
  Eigen::MatrixXd r_matrix;
  r_matrix.setIdentity(6, 6);
  r_matrix *= 0.45;
  EXPECT_TRUE(kal.Correct(measurement, r_matrix));
  ret_states = kal.GetStates();
  EXPECT_NEAR(ret_states(0), 10.3671077, 1e-6);
  EXPECT_NEAR(ret_states(1), 20.3797840, 1e-6);
  EXPECT_NEAR(ret_states(2), 2.5579367, 1e-6);
  EXPECT_NEAR(ret_states(3), 2.5325029, 1e-6);
  EXPECT_NEAR(ret_states(4), 0.49812299, 1e-6);
  EXPECT_NEAR(ret_states(5), 0.49898634, 1e-6);
}

TEST(FusionCommonTest, test_correction) {
  KalmanFilter kal;
  double time_diff = 0.05;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  states.setIdentity(6, 1);
  uncertainty.setIdentity(6, 6);
  states(0) = 10.0;
  states(1) = 20.0;
  states(2) = 2.5;
  states(3) = 2.5;
  states(4) = 0.5;
  states(5) = 0.5;
  uncertainty(0, 2) = 0.5;
  uncertainty(1, 3) = 0.25;
  uncertainty(2, 0) = 0.5;
  uncertainty(3, 1) = 0.25;
  EXPECT_TRUE(kal.Init(states, uncertainty));
  Eigen::MatrixXd transform_matrix;
  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  transform_matrix(0, 4) = 0.5 * time_diff * time_diff;
  transform_matrix(1, 5) = 0.5 * time_diff * time_diff;
  transform_matrix(2, 4) = time_diff;
  transform_matrix(3, 5) = time_diff;
  transform_matrix(4, 4) = 100 * time_diff;
  transform_matrix(5, 5) = 100 * time_diff;
  Eigen::MatrixXd env_uncertainty;
  env_uncertainty.setIdentity(6, 6);
  env_uncertainty = env_uncertainty * 0.1 * time_diff;
  EXPECT_TRUE(kal.Predict(transform_matrix, env_uncertainty));
  std::vector<bool> break_down = {0, 0, 0, 0, 1, 1};
  std::vector<bool> break_down2 = {0, 0, 1, 1, 0, 0};
  EXPECT_TRUE(kal.SetGainBreakdownThresh(break_down, 1.0));
  EXPECT_TRUE(kal.SetValueBreakdownThresh(break_down2, 100));
  kal.CorrectionBreakdown();
  Eigen::VectorXd ret_states = kal.GetStates();
  EXPECT_NEAR(ret_states(3), 0.0, 1e-5);
  EXPECT_NEAR(ret_states(2), 0.0, 1e-5);
  EXPECT_NEAR(ret_states(4), 1.20711, 1e-5);
  EXPECT_NEAR(ret_states(5), 1.20711, 1e-5);
}

TEST(FusionCommonTest, test_set_cmat) {
  KalmanFilter kal;
  Eigen::MatrixXd c_mat;
  c_mat.setIdentity(6, 1);
  bool state = kal.SetControlMatrix(c_mat);
  EXPECT_FALSE(state);
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  EXPECT_FALSE(kal.Init(states, uncertainty));
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  state = kal.Init(states, uncertainty);
  EXPECT_TRUE(state);
  state = kal.SetControlMatrix(c_mat);
  EXPECT_FALSE(state);
  c_mat.setIdentity(4, 4);
  state = kal.SetControlMatrix(c_mat);
  EXPECT_TRUE(state);
}
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
