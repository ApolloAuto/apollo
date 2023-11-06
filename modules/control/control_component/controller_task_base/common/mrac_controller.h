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

/**
 * @file mrac controller.h
 * @brief Defines the MracController class.
 */

#pragma once

#include <vector>

#include "Eigen/Core"

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/control/control_component/proto/mrac_conf.pb.h"

#include "modules/common/status/status.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class MracController
 * @brief A mrac controller for actuation system (throttle/brake and steering)
 */
class MracController {
 public:
  /**
   * @brief initialize mrac controller
   * @param mrac_conf configuration for mrac controller
   * @param latency_param configuration for latency parameter
   * @param dt sampling time interval
   */
  void Init(const MracConf &mrac_conf,
            const common::LatencyParam &latency_param, const double dt);

  /**
   * time constant, natural frequency and damping ratio
   * @param mrac_conf configuration for reference model
   * @return Status parameter initialization status
   */
  common::Status SetReferenceModel(const MracConf &mrac_conf);

  /**
   * state adaptive gain, desired adaptive gain and nonlinear-component adaptive
   * gain
   * @param mrac_conf configuration for adaption model
   * @return Status parameter initialization status
   */
  common::Status SetAdaptionModel(const MracConf &mrac_conf);

  /**
   * @brief build mrac (1st or 2nd) order reference model in the discrete-time
   form, with the bilinear transform (trapezoidal integration) method
   * @return Status reference model initialization status
   */
  common::Status BuildReferenceModel();

  /**
   * @brief build mrac (1st or 2nd) order adaptive dynamic model in the
   * discrete-time form
   * @return Status adaption model initialization status
   */
  common::Status BuildAdaptionModel();

  /**
   * @brief check if the solution of the algebraic Lyapunov Equation is
   * symmetric positive definite
   * @param matrix_a reference model matrix
   * @param matrix_p Lyapunov function matrix
   * @return indicator of the symmetric positive definite matrix
   */
  bool CheckLyapunovPD(const Eigen::MatrixXd matrix_a,
                       const Eigen::MatrixXd matrix_p) const;

  /**
   * @brief estimate the initial states of the adaptive gains via known
   * actuation dynamics approximation
   * @param latency_param configuration for actuation latency parameters
   */
  void EstimateInitialGains(const common::LatencyParam &latency_param);

  /**
   * @brief execute the reference state interation with respect to the designed
   inputs in discrete-time form, with the bilinear transform (trapezoidal
   integration) method
   */
  void UpdateReference();

  /**
   * @brief execute the adaption interation with respect to the designed law in
   discrete-time form, with the bilinear transform (trapezoidal integration)
   method
   * @param law_adp adaptive law at k and k-1 steps
   * @param state_adp state used in the adaptive law at k and k-1 steps
   * @param gain_adp adaptive gain for the given adaptive law
   */
  void UpdateAdaption(Eigen::MatrixXd *law_adp, const Eigen::MatrixXd state_adp,
                      const Eigen::MatrixXd gain_adp);

  /**
   * @brief calculate the anti-windup compensation with respect to the integral
   * windup issue
   * @param control_command desired control command for the actuator
   * @param previous_command last control command for the actuator
   * @param dt control sampling time
   */
  void AntiWindupCompensation(const double control_command,
                              const double previous_command);

  /**
   * @brief reset all the variables (including all the states, gains and
   * externally-setting control parameters) for mrac controller
   */
  void Reset();

  /**
   * @brief reset internal states for mrac controller
   */
  void ResetStates();

  /**
   * @brief reset adaptive gains for mrac controller
   */
  void ResetGains();

  /**
   * @brief compute control value based on the original command
   * @param command original command as the input of the actuation system
   * @param state actual output state of the actuation system
   * @param dt sampling time interval
   * @param input_limit physical or designed bound of the input
   * @param input_rate_limit physical or designed bound of the input
   * changing-rate
   * @return control value based on mrac controller architecture
   */
  virtual double Control(const double command, const Eigen::MatrixXd state,
                         const double input_limit,
                         const double input_rate_limit);

  /**
   * @brief bound the system output with the given bound and change-rate bound
   * @param output_unbounded original system output without bound
   * @param previous_output system output in the last step
   * @param dt sampling time interval
   * @param output_bounded bounded system output
   * @return saturation_status system saturation status indicator
   */
  int BoundOutput(const double output_unbounded, const double previous_output,
                  double *output_bounded);

  /**
   * @brief set initial values for state components in reference model dynamics
   * @param state_reference_init initial reference states
   */
  void SetInitialReferenceState(const Eigen::MatrixXd &state_reference_init);

  /**
   * @brief set initial values for state components in actual actuator dynamics
   * @param state_reference_init initial action states
   */
  void SetInitialActionState(const Eigen::MatrixXd &state_action_init);

  /**
   * @brief set initial command (desired input)
   * @param command_init initial desired input
   */
  void SetInitialCommand(const double command_init);

  /**
   * @brief set initial values of state adaption gains for mrac control
   * @param gain_state_adaption_init initial state adaption gains
   */
  void SetInitialStateAdaptionGain(
      const Eigen::MatrixXd &gain_state_adaption_init);

  /**
   * @brief set initial value of input adaption gain for mrac control
   * @param gain_input_adaption_init initial input adaption gain
   */
  void SetInitialInputAdaptionGain(const double gain_input_adaption_init);

  /**
   * @brief set initial value of nonlinear adaption gain for mrac control
   * @param gain_nonlinear_adaption_init initial nonlinear adaption gain
   */
  void SetInitialNonlinearAdaptionGain(
      const double gain_nonlinear_adaption_init);

  /**
   * @brief set convergence ratio for state components in adaptive dynamics
   * @param ratio_state convergence ratio for state adaption
   */
  void SetStateAdaptionRate(const double ratio_state);

  /**
   * @brief set convergence ratio for input components in adaptive dynamics
   * @param ratio_input convergence ratio for input adaption
   */
  void SetInputAdaptionRate(const double ratio_input);

  /**
   * @brief set convergence ratio for nonlinear components in adaptive dynamics
   * @param ratio_nonlinear convergence ratio for additional nonlinear adaption
   */
  void SetNonlinearAdaptionRate(const double ratio_nonlinear);

  /**
   * @brief get convergence ratio for state components in adaptive dynamics
   * @return state adaption ratio
   */
  double StateAdaptionRate() const;

  /**
   * @brief get convergence ratio for input components in adaptive dynamics
   * @return input adaption ratio
   */
  double InputAdaptionRate() const;

  /**
   * @brief get convergence ratio for nonlinear components in adaptive dynamics
   * @return nonlinear adaption ratio
   */
  double NonlinearAdaptionRate() const;

  /**
   * @brief get saturation status for reference system
   * @return saturation status
   */
  int ReferenceSaturationStatus() const;

  /**
   * @brief get saturation status for control system
   * @return saturation status
   */
  int ControlSaturationStatus() const;

  /**
   * @brief get current state for reference system
   * @return current state
   */
  Eigen::MatrixXd CurrentReferenceState() const;

  /**
   * @brief get current state adaptive gain for mrac control
   * @return current state adaptive gain
   */
  Eigen::MatrixXd CurrentStateAdaptionGain() const;

  /**
   * @brief get current input adaptive gain for mrac control
   * @return current input adaptive gain
   */
  Eigen::MatrixXd CurrentInputAdaptionGain() const;

  /**
   * @brief get current nonlinear adaptive gain for mrac control
   * @return current nonlinear adaptive gain
   */
  Eigen::MatrixXd CurrentNonlinearAdaptionGain() const;

 protected:
  // indicator if the reference/adaption model is valid
  bool reference_model_enabled_ = false;
  bool adaption_model_enabled_ = false;

  // indicator if clamp the adaption laws
  bool adaption_clamping_enabled = false;

  // The order of the reference/adaption model
  int model_order_ = 1;

  // 1st-order Reference system coefficients in continuous-time domain
  double tau_reference_ = 0.0;
  double tau_clamping_ = 0.0;
  // 2nd-order Reference system coefficients in continuous-time domain
  double wn_reference_ = 0.0;
  double zeta_reference_ = 0.0;

  double ts_ = 0.01;  // By default, control sampling time is 0.01 sec

  // Adaption system coefficients
  // State adaption gain
  Eigen::MatrixXd gamma_state_adaption_;
  // Desired command adaption gain
  Eigen::MatrixXd gamma_input_adaption_;
  // Nonlinear dynamics adaption gain
  Eigen::MatrixXd gamma_nonlinear_adaption_;
  // Adjustable ratio of the state adaption gain
  double gamma_ratio_state_ = 1.0;
  // Adjustable ratio of the desired command adaption gain
  double gamma_ratio_input_ = 1.0;
  // Adjustable ratio of the nonlinear dynamics adaption gain
  double gamma_ratio_nonlinear_ = 1.0;

  // Reference system matrix in continuous-time domain
  Eigen::MatrixXd matrix_a_reference_;  // Matrix A in reference models
  Eigen::MatrixXd matrix_b_reference_;  // Matrix B in reference models

  // Adaption system matrix in discrete-time domain
  Eigen::MatrixXd matrix_p_adaption_;  // Matrix P in adaption models
  Eigen::MatrixXd matrix_b_adaption_;  // Matrix B in adaption models

  // Adaption system input variables in discrete-time domain
  Eigen::MatrixXd input_desired_;            // Updated desired command vector
  Eigen::MatrixXd state_action_;             // Updated actuation states vector
  Eigen::MatrixXd state_reference_;          // Reference states vector
  Eigen::MatrixXd gain_state_adaption_;      // State adaption vector
  Eigen::MatrixXd gain_input_adaption_;      // Desired command adaption vector
  Eigen::MatrixXd gain_nonlinear_adaption_;  // Nonlinear adaption vector

  Eigen::MatrixXd gain_state_clamping_;      // To clamp the state adaption
  Eigen::MatrixXd gain_input_clamping_;      // To clamp the command adaption
  Eigen::MatrixXd gain_nonlinear_clamping_;  // To clamp the nonlinear adaption

  Eigen::MatrixXd gain_state_adaption_init_;      // Initial state adaption
  Eigen::MatrixXd gain_input_adaption_init_;      // Initial input adaption
  Eigen::MatrixXd gain_nonlinear_adaption_init_;  // Initial nonlinear adaption

  // Mrac control output in the last step
  double control_previous_ = 0.0;

  // State saturation limits in discrete-time domain
  double bound_ratio_ = 0.0;
  double bound_command_ = 0.0;
  double bound_command_rate_ = 0.0;
  int saturation_status_reference_ = 0;
  int saturation_status_control_ = 0;

  // Anti-Windup compensation
  Eigen::MatrixXd gain_anti_windup_;
  Eigen::MatrixXd compensation_anti_windup_;
};

}  // namespace control
}  // namespace apollo
