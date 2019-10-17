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

#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/digital_filter_coefficients.h"
#include "modules/control/proto/mrac_conf.pb.h"

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
   * @param dt sampling time interval
   */
  void Init(const MracConf &mrac_conf, const double dt);

  /**
   * time constant, natrual frequency and damping ratio
   * @param mrac_conf configuration for reference model
   */
  void SetReferenceModel(const MracConf &mrac_conf);

  /**
   * state adaptive gain, desired adaptive gain and nonlinear-component adaptive
   * gain
   * @param mrac_conf configuration for adaption model
   */
  void SetAdaptionModel(const MracConf &mrac_conf);

  /**
   * @brief transfer mrac (1st or 2nd) order reference model coefficients to the
   discrete-time form, with the bilinear transform (trapezoidal integration)
   method
   * @param dt sampling time interval
   */
  void TransformReferenceModel(const double dt);

  /**
   * @brief exexute the adaption interation with respect to the designed law in
   discrete-time form, with the bilinear transform (trapezoidal integration)
   method
   * @param law_adp adaptive law at k and k-1 steps
   * @param state_adp state used in the adaptive law at k and k-1 steps
   * @param gain_adp adaptive gain for the given adaptive law
   */
  void Adaption(std::vector<double> *law_adp,
                const std::vector<double> state_adp, const double gain_adp);

  /**
   * @brief calculate the anti-windup compensation with respect to the integral
   * windup issue
   * @param control_command desired control command for the actuator
   * @param upper_bound the physical or designed upper bound of the actuator
   * @param upper_bound the physical or designed lower bound of the actuator
   */
  void AntiWindupCompensation(const double control_command,
                              const double upper_bound,
                              const double lower_bound);

  /**
   * @brief reset variables for mrac controller
   */
  void Reset();

  /**
   * @brief compute control value based on the original command
   * @param command original command as the input of the actuation system
   * @param state actual output state of the actuation system
   * @param dt sampling time interval
   * @return control value based on mrac controller architecture
   */
  virtual double Control(const double command, const double state,
                         const double dt);

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
  double CurrentReferenceState() const;

  /**
   * @brief get current state adaptive gain for reference system
   * @return current state adaptive gain
   */
  double CurrentStateAdaptionGain() const;

  /**
   * @brief get current input adaptive gain for reference system
   * @return current input adaptive gain
   */
  double CurrentInputAdaptionGain() const;

 protected:
  // reference model as a digital filter
  common::DigitalFilter reference_model_;
  // indicator if the reference model is valid
  bool reference_model_enabled_ = false;
  // The order of the reference model
  int reference_model_order_ = 1;
  // 1st-order Reference system coefficients in contiouous-time domain
  double tau_reference_ = 0.0;
  // 2nd-order Reference system coefficients in contiouous-time domain
  double wn_reference_ = 0.0;
  double zeta_reference_ = 0.0;
  double Ts_ = 0.01;  // By default, control sampling time is 0.01 sec

  // Adaption system coefficients
  double gamma_state_adaption_ = 0.0;      // State adaption gain
  double gamma_input_adaption_ = 0.0;      // Desired command adaption gain
  double gamma_nonlinear_adaption_ = 0.0;  // Nonlinear dynamics adaption gain

  // Reference system coefficients in discrete-time domain
  std::vector<double> kd_reference_{0.0, 0.0, 0.0};  // Denominator coefficients
  std::vector<double> kn_reference_{0.0, 0.0, 0.0};  // Numerator coefficients

  // Adaption system input variables in discrete-time domain
  std::vector<double> input_desired_{0.0,
                                     0.0};  // Updated desired command vector
  std::vector<double> state_action_{0.0,
                                    0.0};  // Updated actuation states vector
  std::vector<double> state_reference_{0.0, 0.0};  // Reference states vector
  std::vector<double> gain_state_adaption_{0.0, 0.0};  // State adaption vector
  std::vector<double> gain_input_adaption_{
      0.0, 0.0};  // Desired command adaption vector
  std::vector<double> gain_nonlinear_adaption_{
      0.0, 0.0};  // Nonlinear adaption vector

  // Mrac control output in the last step
  double control_previous_ = 0.0;

  // State saturation limits in discrete-time domain
  double bound_reference_high_ = 0.0;
  double bound_reference_low_ = 0.0;
  double bound_control_high_ = 0.0;
  double bound_control_low_ = 0.0;
  int saturation_status_reference_ = 0;
  int saturation_status_control_ = 0;

  // Anti-Windup compensation
  double gain_anti_windup_ = 0.0;
  std::vector<double> compensation_anti_windup_{0.0, 0.0};
};

}  // namespace control
}  // namespace apollo
