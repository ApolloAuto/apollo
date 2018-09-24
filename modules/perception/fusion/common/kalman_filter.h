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
#ifndef PERCEPTION_FUSION_COMMON_KALMAN_FILTER_H_
#define PERCEPTION_FUSION_COMMON_KALMAN_FILTER_H_

#include <vector>
#include "modules/perception/fusion/common/base_filter.h"

namespace apollo {
namespace perception {
namespace fusion {

class KalmanFilter : public BaseFilter {
 public:
  KalmanFilter();
  ~KalmanFilter();

  bool Init(const Eigen::VectorXd &initial_belief_states,
            const Eigen::MatrixXd &initial_uncertainty);

  // @brief predict the current state and uncertainty of system
  // @params[IN] transform_matrix: transform the state from the
  //             pre moment to current moment
  // @params[IN] env_uncertainty_matrix: the uncertainty brought by
  //             the environment when predict.
  bool Predict(const Eigen::MatrixXd &transform_matrix,
               const Eigen::MatrixXd &env_uncertainty_matrix);

  // @brief use the current observation to correct the predict
  // @params[IN] cur_observation: the observationin in current time
  // @params[IN] cur_observation_uncertainty: the uncertainty of
  //             the observation in current time.
  bool Correct(const Eigen::VectorXd &cur_observation,
               const Eigen::MatrixXd &cur_observation_uncertainty);

  // @brief set the control matrix
  bool SetControlMatrix(const Eigen::MatrixXd &control_matrix);

  // @brief get the system states
  Eigen::VectorXd GetStates() const;

  // @brief get the belief uncertainty
  Eigen::MatrixXd GetUncertainty() const;
  bool DeCorrelation(size_t x, size_t y, size_t x_len, size_t y_len);
  void CorrectionBreakdown();
  bool SetGainBreakdownThresh(const std::vector<bool> &break_down,
                              const float threshold = 2.0);
  bool SetValueBreakdownThresh(const std::vector<bool> &break_down,
                               const float threshold = 0.05);

 private:
  // @brief kalman gain
  Eigen::VectorXd prior_global_states_;
  Eigen::VectorXd gain_break_down_;
  Eigen::VectorXd value_break_down_;
  Eigen::MatrixXd kalman_gain_;

  float value_break_down_threshold_ = 999.0;
  float gain_break_down_threshold_ = 0.0;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_FUSION_COMMON_KALMAN_FILTER_H_
