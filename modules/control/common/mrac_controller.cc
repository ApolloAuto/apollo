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

#include "cyber/common/log.h"

namespace apollo {
namespace control {

double MracController::Control(const double command, const double state,
                               const double dt) {
  // check if the reference model well set up during the initilization
  if (!reference_model_enabled_) {
    TransformReferenceModel(dt);
    reference_model_.set_coefficients(kd_reference_, kn_reference_);
    if (!reference_model_enabled_) {
      AWARN << "MRAC: reference model setting failed; will work as a unity "
               "compensator, dt: "
            << dt;
      return command;  // treat the mrac as a unity proportional controller
    }
  }
  // check if the current sampling time is valid
  if (dt <= 0.0) {
    AWARN << "MRAC: current sampling time <= 0, will use the last control, dt: "
          << dt;
    return control_previous_;
  }
  double control = 0.0;

  // update the state in the reference system
  state_reference_[0] = reference_model_.Filter(command);
  if (state_reference_[0] > bound_reference_high_) {
    state_reference_[0] = bound_reference_high_;
    saturation_status_reference_ = 1;
  } else if (state_reference_[0] < bound_reference_low_) {
    state_reference_[0] = bound_reference_low_;
    saturation_status_reference_ = -1;
  } else {
    saturation_status_reference_ = 0;
  }

  // update the state in the real actuation system
  state_action_[0] = state;

  // update the desired command in the real actuation system
  input_desired_[0] = command;

  // update the adaption laws including state adaption, command adaption and
  // nonlinear components adaption
  Adaption(&gain_state_adaption_, state_action_, gamma_state_adaption_);
  Adaption(&gain_input_adaption_, input_desired_, gamma_input_adaption_);

  // update the generated control based on the adaptive law
  double control_unbounded = gain_state_adaption_[0] * state_action_[0] +
                             gain_input_adaption_[0] * input_desired_[0];
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
  state_reference_[1] = state_reference_[0];
  state_action_[1] = state_action_[0];
  input_desired_[1] = input_desired_[0];
  control_previous_ = control;
  return control;
}

void MracController::Reset() {
  control_previous_ = 0.0;
  saturation_status_reference_ = 0;
  saturation_status_control_ = 0;

  std::fill(input_desired_.begin(), input_desired_.end(), 0.0);
  std::fill(state_action_.begin(), state_action_.end(), 0.0);
  std::fill(state_reference_.begin(), state_reference_.end(), 0.0);
  std::fill(gain_state_adaption_.begin(), gain_state_adaption_.end(), 0.0);
  std::fill(gain_input_adaption_.begin(), gain_input_adaption_.end(), 0.0);
  std::fill(gain_nonlinear_adaption_.begin(), gain_nonlinear_adaption_.end(),
            0.0);

  reference_model_.reset_values();
  reference_model_enabled_ = false;
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
  // Initialize the adaption model parameters
  SetAdaptionModel(mrac_conf);
  // Initialize the reference model parameters
  reference_model_order_ = mrac_conf.mrac_reference_order();
  SetReferenceModel(mrac_conf);
  TransformReferenceModel(dt);
  reference_model_.set_coefficients(kd_reference_, kn_reference_);
  // Initialize the anti-windup parameters
  gain_anti_windup_ = mrac_conf.anti_windup_compensation_gain();
}

void MracController::SetReferenceModel(const MracConf &mrac_conf) {
  tau_reference_ = mrac_conf.reference_time_constant();
  wn_reference_ = mrac_conf.reference_natural_frequency();
  zeta_reference_ = mrac_conf.reference_damping_ratio();
}

void MracController::SetAdaptionModel(const MracConf &mrac_conf) {
  gamma_state_adaption_ = mrac_conf.adaption_state_gain();
  gamma_input_adaption_ = mrac_conf.adaption_desired_gain();
  gamma_nonlinear_adaption_ = mrac_conf.adaption_nonlinear_gain();
}

void MracController::Adaption(std::vector<double> *law_adp,
                              const std::vector<double> state_adp,
                              const double gain_adp) {
  std::vector<double> state_error{state_action_[0] - state_reference_[0],
                                  state_action_[1] - state_reference_[1]};
  double tmp = (*law_adp)[0];
  (*law_adp)[0] = (*law_adp)[1] -
                  (0.5 * Ts_ * gain_adp * state_adp[0] *
                   (state_error[0] + compensation_anti_windup_[0])) -
                  (0.5 * Ts_ * gain_adp * state_adp[1] *
                   (state_error[1] + compensation_anti_windup_[1]));
  (*law_adp)[1] = tmp;
}

void MracController::AntiWindupCompensation(const double control_command,
                                            const double upper_bound,
                                            const double lower_bound) {
  if (upper_bound < lower_bound) {
    AWARN << "windup upper_bound < lower_bound; failed to exectute the "
             "anti-windup logic";
    compensation_anti_windup_ = {0.0, 0.0};
  }
  double offset_windup =
      ((control_command > upper_bound) ? upper_bound - control_command : 0.0) +
      ((control_command < lower_bound) ? lower_bound - control_command : 0.0);
  compensation_anti_windup_[1] = compensation_anti_windup_[0];
  compensation_anti_windup_[0] = gain_anti_windup_ * offset_windup;
}

void MracController::TransformReferenceModel(const double dt) {
  if (dt <= 0.0) {
    AWARN << "dt <= 0, continuous-discrete transformation for reference model "
             "failed, dt: "
          << dt;
    reference_model_enabled_ = false;
  } else {
    if (reference_model_order_ == 1) {
      double a_ref = -1 / tau_reference_;
      kn_reference_[0] = a_ref * Ts_;
      kn_reference_[1] = a_ref * Ts_;
      kd_reference_[0] = -2;
      kd_reference_[1] = 2;
    } else if (reference_model_order_ == 2) {
      double a_ref = 2 * wn_reference_ * zeta_reference_;
      double b_ref = wn_reference_ * wn_reference_;
      Ts_ = dt;
      kn_reference_[0] = b_ref * Ts_ * Ts_;
      kn_reference_[1] = 2 * b_ref * Ts_ * Ts_;
      kn_reference_[2] = b_ref * Ts_ * Ts_;
      kd_reference_[0] = 4 + 2 * a_ref * Ts_ + b_ref * Ts_ * Ts_;
      kd_reference_[1] = -8 + 2 * b_ref * Ts_ * Ts_;
      kd_reference_[2] = 4 - 2 * a_ref * Ts_ + b_ref * Ts_ * Ts_;
    } else {
      AWARN << "reference model order beyond the designed range, "
               "reference_model_order: "
            << reference_model_order_;
      reference_model_enabled_ = false;
    }
  }
  reference_model_enabled_ = true;
}  // namespace control

int MracController::ReferenceSaturationStatus() const {
  return saturation_status_reference_;
}

int MracController::ControlSaturationStatus() const {
  return saturation_status_control_;
}

double MracController::CurrentReferenceState() const {
  return state_reference_[0];
}

}  // namespace control
}  // namespace apollo
