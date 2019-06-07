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
#pragma once

#include <Eigen/Dense>
#include <string>

namespace apollo {
namespace perception {
namespace fusion {

// @brief base filter inference
class BaseFilter {
 public:
  // @brief constructor
  explicit BaseFilter(const std::string name)
      : init_(false), name_(name), states_num_(0) {}

  // @brief destructor
  virtual ~BaseFilter() {}

  // @brief filter initialized
  // @params[IN] gloabl_states: a vector contains system states(
  //             position, velocity etc.)
  // @params[IN] global_uncertainty: a covariance matrix which
  //             indicate the uncertainty of each system state
  virtual bool Init(const Eigen::VectorXd &global_states,
                    const Eigen::MatrixXd &global_uncertainty) = 0;

  // @brief predict the current state and uncertainty of system
  // @params[IN] transform_matrix: transform the state from the
  //             pre moment to current moment
  // @params[IN] env_uncertainty_matrix: the uncertainty brought by
  //             the environment when predict.
  virtual bool Predict(const Eigen::MatrixXd &transform_matrix,
                       const Eigen::MatrixXd &env_uncertainty_matrix) = 0;

  // @brief use the current observation to correct the predict
  // @params[IN] cur_observation: the observation in current time
  // @params[IN] cur_observation_uncertainty: the uncertainty of
  //             the observation
  virtual bool Correct(const Eigen::VectorXd &cur_observation,
                       const Eigen::MatrixXd &cur_observation_uncertainty) = 0;

  // @brief set the control matrix
  virtual bool SetControlMatrix(const Eigen::MatrixXd &c_matrix) = 0;

  // @brief get the system states
  virtual Eigen::VectorXd GetStates() const = 0;

  // @brief get the name of the filter
  std::string Name() { return name_; }

 protected:
  // @brief whether the filter has been init
  bool init_;

  // @brief the name of the filter
  std::string name_;

  // @brief the number of the system states
  int states_num_;

  Eigen::MatrixXd transform_matrix_;
  Eigen::VectorXd global_states_;
  Eigen::MatrixXd global_uncertainty_;
  Eigen::MatrixXd env_uncertainty_;
  Eigen::MatrixXd cur_observation_;
  Eigen::MatrixXd cur_observation_uncertainty_;
  Eigen::MatrixXd c_matrix_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
