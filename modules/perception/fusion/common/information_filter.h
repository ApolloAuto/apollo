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

#include "modules/perception/fusion/common/base_filter.h"

namespace apollo {
namespace perception {
namespace fusion {

class InformationFilter : public BaseFilter {
 public:
  InformationFilter();
  ~InformationFilter() = default;

  bool Init(const Eigen::VectorXd &global_states,
            const Eigen::MatrixXd &global_uncertainty);

  // @brief: Set the obervation of the last moment(imf use
  //         the latest two moment of the observation which
  //         from the same source to correct the prediction.
  // @parmas[IN] last_observation: the observation of the last moment
  // @params[IN] last_observation_uncertainty: the uncertainty of the
  //             observation of the last moment
  // @params[IN] last_to_cur_transform_matrix: regard last observatopn
  //             as a state and apply a motion model(the transform
  //             matrix) to predict current state of the last observation.
  // @params[IN] last_to_cur_env_uncertainty: the uncertainty brought
  //             by the environment when predict the current state of
  //             last observation.
  bool SetLastObservation(const Eigen::VectorXd &last_observation,
                          const Eigen::MatrixXd &last_observation_uncertainty,
                          const Eigen::MatrixXd &last_to_cur_transform_matrix,
                          const Eigen::MatrixXd &last_to_cur_env_uncertainty);

  // @brief predict the current state and uncertainty of system
  // @params[IN] transform_matrix: transform the state from the
  //             pre moment to current moment.
  // @params[IN] env_uncertainty_matrix: the uncertainty brought by
  //             the environment when predict system state.
  bool Predict(const Eigen::MatrixXd &transform_matrix,
               const Eigen::MatrixXd &env_uncertainty_matrix);

  // @brief Use the current observation to correct the prediction.
  // @params[IN] cur_observation: the observation in current moment.
  // @params[IN] cur_observation_uncertainty: the uncertainty of
  //             the current observation
  bool Correct(const Eigen::VectorXd &current_observation,
               const Eigen::MatrixXd &current_observation_uncertainty);

  // @brief get the system state
  Eigen::VectorXd GetStates() const { return global_states_; }

  // @brief get the number of the system states
  inline int GetStateNums() const { return states_num_; }

  // @brief set the control matrix
  bool SetControlMatrix(const Eigen::MatrixXd &control_matrix);

 protected:
  // @brief whether the observation of last moment has been set
  bool last_observation_init_;

  // @brief the observation of the last moment
  Eigen::VectorXd last_observation_;

  // @brief the uncertainty of the last moment observation
  Eigen::MatrixXd last_observation_uncertainty_;

  // @brief a temporary var for compute conveniece
  Eigen::VectorXd tmp_states_;

  // @brief motion model to predict the current state of the
  //        last observation
  Eigen::MatrixXd last_to_cur_transform_matrix_;

  // @brife uncertainty when predict the current state of the
  //        last  observation
  Eigen::MatrixXd last_to_cur_env_uncertainty_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
