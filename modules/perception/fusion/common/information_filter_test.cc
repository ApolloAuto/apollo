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

#include "cyber/common/log.h"
#include "modules/perception/fusion/common/information_filter.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(FusionCommonTest, return_name_test) {
  InformationFilter imf;
  EXPECT_EQ(imf.Name(), "InformationFilter");
}

TEST(FusionCommonTest, test_init) {
  InformationFilter imf;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  EXPECT_FALSE(imf.Init(states, uncertainty));
  states.setIdentity(3, 1);
  uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(imf.Init(states, uncertainty));
  uncertainty.setIdentity(4, 4);
  EXPECT_FALSE(imf.Init(states, uncertainty));
  states.setIdentity(4, 1);
  EXPECT_TRUE(imf.Init(states, uncertainty));
}

TEST(FusionCommonTest, test_set_last_observation) {
  InformationFilter imf;
  Eigen::VectorXd last_observation;
  Eigen::MatrixXd last_observation_uncertainty;
  Eigen::MatrixXd last_to_cur_transform_matrix;
  Eigen::MatrixXd last_to_cur_env_uncertainty;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(imf.Init(states, uncertainty));
  last_observation.setIdentity(3, 1);
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  last_observation.setIdentity(4, 1);
  last_observation_uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  last_observation_uncertainty.setIdentity(4, 3);
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  last_observation_uncertainty.setIdentity(4, 4);
  last_to_cur_transform_matrix.setIdentity(3, 4);
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  last_to_cur_transform_matrix.setIdentity(4, 3);
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  last_to_cur_transform_matrix.setIdentity(4, 4);
  last_to_cur_env_uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  last_to_cur_env_uncertainty.setIdentity(4, 3);
  EXPECT_FALSE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
  last_to_cur_env_uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(imf.SetLastObservation(
      last_observation, last_observation_uncertainty,
      last_to_cur_transform_matrix, last_to_cur_env_uncertainty));
}

TEST(FusionCommonTest, test_predict) {
  InformationFilter imf;
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;
  EXPECT_FALSE(imf.Predict(transform_matrix, env_uncertainty));
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(imf.Init(states, uncertainty));
  transform_matrix.setIdentity(3, 4);
  EXPECT_FALSE(imf.Predict(transform_matrix, env_uncertainty));
  transform_matrix.setIdentity(4, 3);
  EXPECT_FALSE(imf.Predict(transform_matrix, env_uncertainty));
  transform_matrix.setIdentity(4, 4);
  env_uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(imf.Predict(transform_matrix, env_uncertainty));
  env_uncertainty.setIdentity(4, 3);
  EXPECT_FALSE(imf.Predict(transform_matrix, env_uncertainty));
  env_uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(imf.Predict(transform_matrix, env_uncertainty));
}

TEST(FusionCommonTest, test_set_cmat) {
  InformationFilter imf;
  Eigen::MatrixXd c_mat;
  c_mat.setIdentity(6, 1);
  bool state = imf.SetControlMatrix(c_mat);
  EXPECT_FALSE(state);
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  EXPECT_FALSE(imf.Init(states, uncertainty));
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  state = imf.Init(states, uncertainty);
  EXPECT_TRUE(state);
  state = imf.SetControlMatrix(c_mat);
  EXPECT_FALSE(state);
  c_mat.setIdentity(4, 4);
  state = imf.SetControlMatrix(c_mat);
  EXPECT_TRUE(state);
}

TEST(FusionCommonTest, test_correct) {
  InformationFilter imf;
  Eigen::VectorXd obs_states;
  Eigen::MatrixXd obs_uncertainty;
  EXPECT_FALSE(imf.Correct(obs_states, obs_uncertainty));
  Eigen::VectorXd states;
  Eigen::MatrixXd uncertainty;
  states.setIdentity(4, 1);
  uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(imf.Init(states, uncertainty));
  obs_states.setIdentity(3, 1);
  EXPECT_FALSE(imf.Correct(obs_states, obs_uncertainty));
  obs_states.setIdentity(4, 1);
  obs_uncertainty.setIdentity(3, 4);
  EXPECT_FALSE(imf.Correct(obs_states, obs_uncertainty));
  obs_uncertainty.setIdentity(4, 3);
  EXPECT_FALSE(imf.Correct(obs_states, obs_uncertainty));
  obs_uncertainty.setIdentity(4, 4);
  EXPECT_TRUE(imf.Correct(obs_states, obs_uncertainty));
}

TEST(FusionCommonTest, test_value_correct) {
  double time_diff = 0.05;
  InformationFilter imf;
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
  EXPECT_TRUE(imf.Init(states, uncertainty));

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
  EXPECT_TRUE(imf.Predict(transform_matrix, env_uncertainty));

  Eigen::VectorXd ret_states = imf.GetStates();

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
  EXPECT_TRUE(imf.Correct(measurement, r_matrix));

  ret_states = imf.GetStates();
  EXPECT_NEAR(ret_states(0), 10.3671077, 1e-6);
  EXPECT_NEAR(ret_states(1), 20.3797840, 1e-6);
  EXPECT_NEAR(ret_states(2), 2.5579367, 1e-6);
  EXPECT_NEAR(ret_states(3), 2.5325029, 1e-6);
  EXPECT_NEAR(ret_states(4), 0.49812299, 1e-6);
  EXPECT_NEAR(ret_states(5), 0.49898634, 1e-6);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
