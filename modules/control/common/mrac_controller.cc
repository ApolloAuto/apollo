/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/common/mrac_controller.h"

#include <cmath>
#include <vector>

#include "Eigen/LU"

#include "cyber/common/log.h"

namespace apollo {
namespace control {

using Matrix = Eigen::MatrixXd;

double MracController::Control(const double command, const Matrix state,
                               const double dt) {
  // check if the reference/adaption model well set up during the initilization
  if (!reference_model_enabled_ || !adaption_model_enabled_) {
    AWARN << "MRAC: model build failed; will work as a unity compensator. The "
             "reference_model building status: "
          << reference_model_enabled_
          << "; The adaption_model building status: "
          << adaption_model_enabled_;
    return command;  // treat the mrac as a unity proportional controller
  }
  // check if the current sampling time is valid
  if (dt <= 0.0) {
    AWARN << "MRAC: current sampling time <= 0, will use the last control, dt: "
          << dt;
    return control_previous_;
  }
  double control = 0.0;

  // update the state in the real actuation system
  state_action_.col(0) = state;

  // update the desired command in the real actuation system
  input_desired_(0, 0) = command;

  // update the state in the reference system
  Matrix matrix_i = Matrix::Identity(model_order_, model_order_);
  state_reference_.col(0) =
      (matrix_i - dt * 0.5 * matrix_a_reference_).inverse() *
      ((matrix_i + dt * 0.5 * matrix_a_reference_) * state_reference_.col(0) +
       dt * 0.5 * matrix_b_reference_ *
           (input_desired_(0, 0) + input_desired_(0, 1)));

  if (state_reference_(0, 0) > bound_reference_high_) {
    state_reference_(0, 0) = bound_reference_high_;
    saturation_status_reference_ = 1;
  } else if (state_reference_(0, 0) < bound_reference_low_) {
    state_reference_(0, 0) = bound_reference_low_;
    saturation_status_reference_ = -1;
  } else {
    saturation_status_reference_ = 0;
  }

  // update the adaption laws including state adaption, command adaption and
  // nonlinear components adaption
  Adaption(&gain_state_adaption_, state_action_,
           gamma_state_adaption_ * gamma_ratio_state_);
  Adaption(&gain_input_adaption_, input_desired_,
           gamma_input_adaption_ * gamma_ratio_input_);

  // update the generated control based on the adaptive law
  double control_unbounded =
      gain_state_adaption_.col(0).transpose() * state_action_.col(0) +
      gain_input_adaption_(0, 0) * input_desired_(0, 0);
  if (control_unbounded > bound_control_high_) {
    control = bound_control_high_;
    saturation_status_control_ = 1;
  } else if (control_unbounded < bound_control_low_) {
    control = bound_control_low_;
    saturation_status_control_ = -1;
  } else {
    control = control_unbounded;
    saturation_status_control_ = 0;
  }

  // update the anti-windup compensation if applied
  AntiWindupCompensation(control_unbounded, bound_control_high_,
                         bound_control_low_);

  // update the previous value for next iteration
  state_reference_.col(1) = state_reference_.col(0);
  state_action_.col(1) = state_action_.col(0);
  input_desired_.col(1) = input_desired_.col(0);
  control_previous_ = control;
  return control;
}

void MracController::Reset() {
  // reset the overall states
  ResetStates();
  // reset the adaptive gains
  ResetGains();
  // reset all the externally-setting control parameters
  gamma_ratio_state_ = 1.0;
  gamma_ratio_input_ = 1.0;
  gamma_ratio_nonlinear_ = 1.0;
}

void MracController::ResetStates() {
  // reset the inputs and outputs of the closed-loop MRAC controller
  control_previous_ = 0.0;
  input_desired_.setZero(1, 2);
  // reset the internal states, anti-windup compensations and status
  state_action_.setZero(model_order_, 2);
  state_reference_.setZero(model_order_, 2);
  compensation_anti_windup_.setZero(model_order_, 2);
  saturation_status_reference_ = 0;
  saturation_status_control_ = 0;
}

void MracController::ResetGains() {
  gain_state_adaption_.setZero(model_order_, 2);
  gain_input_adaption_.setZero(1, 2);
  gain_nonlinear_adaption_.setZero(1, 2);
}

void MracController::Init(const MracConf &mrac_conf, const double dt) {
  control_previous_ = 0.0;
  saturation_status_control_ = 0;
  saturation_status_reference_ = 0;
  // Initialize the saturation limits
  bound_control_high_ = std::fabs(mrac_conf.mrac_saturation_level());
  bound_control_low_ = -std::fabs(mrac_conf.mrac_saturation_level());
  bound_reference_high_ = std::fabs(mrac_conf.mrac_saturation_level());
  bound_reference_low_ = -std::fabs(mrac_conf.mrac_saturation_level());
  // Initialize the common model parameters
  model_order_ = mrac_conf.mrac_reference_order();
  // Initialize the system states
  input_desired_ = Matrix::Zero(1, 2);
  state_action_ = Matrix::Zero(model_order_, 2);
  state_reference_ = Matrix::Zero(model_order_, 2);
  gain_state_adaption_ = Matrix::Zero(model_order_, 2);
  gain_input_adaption_ = Matrix::Zero(1, 2);
  gain_nonlinear_adaption_ = Matrix::Zero(1, 2);
  gamma_state_adaption_ = Matrix::Zero(model_order_, 1);
  gamma_input_adaption_ = Matrix::Zero(1, 1);
  gamma_nonlinear_adaption_ = Matrix::Zero(1, 1);
  compensation_anti_windup_ = Matrix::Zero(model_order_, 2);
  // Initialize the reference model parameters
  matrix_a_reference_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_reference_ = Matrix::Zero(model_order_, 1);
  SetReferenceModel(mrac_conf);
  BuildReferenceModel(dt);
  // Initialize the adaption model parameters
  matrix_p_adaption_ = Matrix::Zero(model_order_, model_order_);
  matrix_b_adaption_ = Matrix::Zero(model_order_, 1);
  SetAdaptionModel(mrac_conf);
  BuildAdaptionModel();
  // Initialize the anti-windup parameters
  gain_anti_windup_ = mrac_conf.anti_windup_compensation_gain();
}

void MracController::SetReferenceModel(const MracConf &mrac_conf) {
  tau_reference_ = mrac_conf.reference_time_constant();
  wn_reference_ = mrac_conf.reference_natural_frequency();
  zeta_reference_ = mrac_conf.reference_damping_ratio();
}

void MracController::SetAdaptionModel(const MracConf &mrac_conf) {
  for (int i = 0; i < model_order_; ++i) {
    gamma_state_adaption_(i, 0) = mrac_conf.adaption_state_gain();
  }
  gamma_input_adaption_(0, 0) = mrac_conf.adaption_desired_gain();
  gamma_nonlinear_adaption_(0, 0) = mrac_conf.adaption_nonlinear_gain();
}

void MracController::BuildReferenceModel(const double dt) {
  if (dt <= 0.0) {
    AWARN << "dt <= 0, continuous-discrete transformation for reference model "
             "failed, dt: "
          << dt;
    reference_model_enabled_ = false;
  } else {
    if (model_order_ == 1) {
      matrix_a_reference_(0, 0) = -1.0 / tau_reference_;
      matrix_b_reference_(0, 0) = 1.0 / tau_reference_;
    } else if (model_order_ == 2) {
      Ts_ = dt;
      matrix_a_reference_(0, 1) = 1.0;
      matrix_a_reference_(1, 0) = -wn_reference_ * wn_reference_;
      matrix_a_reference_(1, 1) = -2 * zeta_reference_ / wn_reference_;
      matrix_b_reference_(1, 0) = wn_reference_ * wn_reference_;
    } else {
      AWARN << "reference model order beyond the designed range, "
               "model_order: "
            << model_order_;
      reference_model_enabled_ = false;
    }
  }
  reference_model_enabled_ = true;
}

void MracController::BuildAdaptionModel() {
  if (model_order_ == 1) {
    matrix_b_adaption_(0, 0) = 1.0;
    matrix_p_adaption_(0, 0) = 1.0;
  } else if (model_order_ == 2) {
    matrix_b_adaption_(1, 0) = 1.0;
    matrix_p_adaption_(0, 0) = 1.0;
    matrix_p_adaption_(1, 1) = 1.0;
    // TODO(Yu): check whether the high-order P matrix is positive definite
  } else {
    AWARN << "Adaption model order beyond the designed range, "
             "model_order: "
          << model_order_;
    adaption_model_enabled_ = false;
  }
  adaption_model_enabled_ = true;
}

void MracController::Adaption(Matrix *law_adp, const Matrix state_adp,
                              const Matrix gain_adp) {
  Matrix state_error = state_action_ - state_reference_;
  law_adp->col(0) = law_adp->col(1) -
                    (0.5 * Ts_ * gain_adp * state_adp.col(0) *
                     (state_error.col(0) + compensation_anti_windup_.col(0))) -
                    (0.5 * Ts_ * gain_adp * state_adp.col(1) *
                     (state_error.col(1) + compensation_anti_windup_.col(1)));
  law_adp->col(1) = law_adp->col(0);
}

void MracController::AntiWindupCompensation(const double control_command,
                                            const double upper_bound,
                                            const double lower_bound) {
  if (upper_bound < lower_bound) {
    AWARN << "windup upper_bound < lower_bound; failed to execute the "
             "anti-windup logic";
    compensation_anti_windup_.setZero(model_order_, 2);
  }
  double offset_windup =
      ((control_command > upper_bound) ? upper_bound - control_command : 0.0) +
      ((control_command < lower_bound) ? lower_bound - control_command : 0.0);
  compensation_anti_windup_.col(1) = compensation_anti_windup_.col(0);
  compensation_anti_windup_(0, 0) = gain_anti_windup_ * offset_windup;
  // Todo(Yu): refactor the anti-windup for high-order system
}

void MracController::SetStateAdaptionRate(const double ratio_state) {
  if (ratio_state < 0.0) {
    AWARN << "failed to set the state adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_state_;
  } else {
    gamma_ratio_state_ = ratio_state;
  }
}

void MracController::SetInputAdaptionRate(const double ratio_input) {
  if (ratio_input < 0.0) {
    AWARN << "failed to set the input adaption rate, due to new ratio < 0; the "
             "current ratio is still: "
          << gamma_ratio_input_;
  } else {
    gamma_ratio_input_ = ratio_input;
  }
}

void MracController::SetNonlinearAdaptionRate(const double ratio_nonlinear) {
  if (ratio_nonlinear < 0.0) {
    AWARN << "failed to set the nonlinear adaption rate, due to new ratio < 0; "
             "the current ratio is still: "
          << gamma_ratio_nonlinear_;
  } else {
    gamma_ratio_nonlinear_ = ratio_nonlinear;
  }
}

double MracController::StateAdaptionRate() const { return gamma_ratio_state_; }

double MracController::InputAdaptionRate() const { return gamma_ratio_input_; }

double MracController::NonlinearAdaptionRate() const {
  return gamma_ratio_nonlinear_;
}

int MracController::ReferenceSaturationStatus() const {
  return saturation_status_reference_;
}

int MracController::ControlSaturationStatus() const {
  return saturation_status_control_;
}

double MracController::CurrentReferenceState() const {
  return state_reference_(0, 0);
}

double MracController::CurrentStateAdaptionGain() const {
  return gain_state_adaption_(0, 0);
}

double MracController::CurrentInputAdaptionGain() const {
  return gain_input_adaption_(0, 0);
}

}  // namespace control
}  // namespace apollo
