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
#include "modules/perception/fusion/common/information_filter.h"

namespace apollo {
namespace perception {
namespace fusion {

InformationFilter::InformationFilter()
    : BaseFilter("InformationFilter"), last_observation_init_(false) {}

bool InformationFilter::Init(const Eigen::VectorXd &global_states,
                             const Eigen::MatrixXd &global_uncertainty) {
  if (global_uncertainty.rows() != global_uncertainty.cols()) {
    return false;
  }

  states_num_ = static_cast<int>(global_uncertainty.rows());

  if (states_num_ <= 0) {
    return false;
  }

  if (states_num_ != global_states.rows()) {
    return false;
  }

  global_states_ = global_states;
  global_uncertainty_ = global_uncertainty;
  tmp_states_ = global_uncertainty_.inverse() * global_states_;

  transform_matrix_.setIdentity(states_num_, states_num_);
  last_to_cur_transform_matrix_.setIdentity(states_num_, states_num_);

  cur_observation_.setZero(states_num_, 1);
  cur_observation_uncertainty_.setIdentity(states_num_, states_num_);

  last_observation_.setZero(states_num_, 1);
  last_observation_uncertainty_.setIdentity(states_num_, states_num_);

  c_matrix_.setIdentity(states_num_, states_num_);
  env_uncertainty_.setZero(states_num_, states_num_);

  init_ = true;
  return true;
}

bool InformationFilter::SetLastObservation(
    const Eigen::VectorXd &last_observation,
    const Eigen::MatrixXd &last_observation_uncertainty,
    const Eigen::MatrixXd &last_to_cur_transform_matrix,
    const Eigen::MatrixXd &last_to_cur_env_uncertainty) {
  if (!init_) {
    return false;
  }
  if (last_observation.rows() != states_num_) {
    return false;
  }
  if (last_observation_uncertainty.rows() != states_num_) {
    return false;
  }
  if (last_observation_uncertainty.cols() != states_num_) {
    return false;
  }
  if (last_to_cur_transform_matrix.rows() != states_num_) {
    return false;
  }
  if (last_to_cur_transform_matrix.cols() != states_num_) {
    return false;
  }
  if (last_to_cur_env_uncertainty.rows() != states_num_) {
    return false;
  }
  if (last_to_cur_env_uncertainty.cols() != states_num_) {
    return false;
  }

  last_observation_ = last_observation;
  last_observation_uncertainty_ = last_observation_uncertainty;
  last_to_cur_transform_matrix_ = last_to_cur_transform_matrix;
  last_to_cur_env_uncertainty_ = last_to_cur_env_uncertainty;
  last_observation_init_ = true;
  return true;
}

bool InformationFilter::Predict(const Eigen::MatrixXd &transform_matrix,
                                const Eigen::MatrixXd &env_uncertainty) {
  if (!init_) {
    return false;
  }
  if (transform_matrix.rows() != states_num_) {
    return false;
  }
  if (transform_matrix.cols() != states_num_) {
    return false;
  }
  if (env_uncertainty.rows() != states_num_) {
    return false;
  }
  if (env_uncertainty.cols() != states_num_) {
    return false;
  }
  transform_matrix_ = transform_matrix;
  env_uncertainty_ = env_uncertainty;
  global_states_ = transform_matrix_ * global_states_;
  global_uncertainty_ =
      transform_matrix_ * global_uncertainty_ * transform_matrix_.transpose() +
      env_uncertainty_;
  return true;
}

bool InformationFilter::Correct(
    const Eigen::VectorXd &cur_observation,
    const Eigen::MatrixXd &cur_observation_uncertainty) {
  if (!init_) {
    return false;
  }
  if (cur_observation.rows() != states_num_) {
    return false;
  }
  if (cur_observation_uncertainty.rows() != states_num_) {
    return false;
  }
  if (cur_observation_uncertainty.cols() != states_num_) {
    return false;
  }
  cur_observation_ = cur_observation;
  cur_observation_uncertainty_ = cur_observation_uncertainty;
  // global_uncertainty now stores information matrix
  global_uncertainty_ = global_uncertainty_.inverse();
  // tmp_states_ is information vector
  tmp_states_ = global_uncertainty_ * global_states_;
  // cur_observation_uncertainty_ is now the inverse of covariance matrix
  cur_observation_uncertainty_ = cur_observation_uncertainty_.inverse();
  if (last_observation_init_) {
    // propate to current time
    last_observation_ = last_to_cur_transform_matrix_ * last_observation_;
    last_observation_uncertainty_ =
        last_to_cur_transform_matrix_ * last_observation_uncertainty_ *
            last_to_cur_transform_matrix_.transpose() +
        last_to_cur_env_uncertainty_;
    last_observation_uncertainty_ =  // transform to measurement space
        c_matrix_ * last_observation_uncertainty_ * c_matrix_.transpose();
    global_uncertainty_ =  // update information matrix
        global_uncertainty_ +
        (c_matrix_.transpose() * cur_observation_uncertainty_ * c_matrix_ -
         c_matrix_.transpose() * last_observation_uncertainty_.inverse() *
             c_matrix_);
    tmp_states_ +=  // update information vector
        (c_matrix_.transpose() * cur_observation_uncertainty_ *
             cur_observation_ -
         c_matrix_.transpose() * last_observation_uncertainty_.inverse() *
             last_observation_);
  } else {
    global_uncertainty_ +=
        c_matrix_.transpose() * cur_observation_uncertainty_ * c_matrix_;
    tmp_states_ +=
        c_matrix_.transpose() * cur_observation_uncertainty_ * cur_observation_;
  }

  global_states_ = global_uncertainty_.inverse() * tmp_states_;
  last_observation_init_ = false;
  return true;
}
bool InformationFilter::SetControlMatrix(
    const Eigen::MatrixXd &control_matrix) {
  if (!init_) {
    return false;
  }
  if (control_matrix.rows() != states_num_ ||
      control_matrix.cols() != states_num_) {
    return false;
  }
  c_matrix_ = control_matrix;
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
