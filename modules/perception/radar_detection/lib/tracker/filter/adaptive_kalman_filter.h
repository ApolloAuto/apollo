/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include "Eigen/Core"

#include "modules/perception/radar_detection/interface/base_filter.h"

namespace apollo {
namespace perception {
namespace radar {

class AdaptiveKalmanFilter : public BaseFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  AdaptiveKalmanFilter();
  ~AdaptiveKalmanFilter();

  /**
   * @brief Init base filter config
   *
   * @param object
   */
  void Init(const base::Object& object) override;

  /**
   * @brief Predict the state of the object after time_diff
   *
   * @param time_diff
   * @return Eigen::VectorXd
   */
  Eigen::VectorXd Predict(const double time_diff) override;

  /**
   * @brief Update the state of the target based on the observations
   *
   * @param new_object
   * @param time_diff
   * @return Eigen::VectorXd
   */
  Eigen::VectorXd UpdateWithObject(const base::Object& new_object,
                                   double time_diff) override;

  /**
   * @brief Get the State object
   *
   * @param anchor_point
   * @param velocity
   */
  void GetState(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity);

  /**
   * @brief Get the Covariance Matrix object
   *
   * @return Eigen::Matrix4d
   */
  Eigen::Matrix4d GetCovarianceMatrix() override { return p_matrix_; }

  /**
   * @brief Set the QMatrix Ratio
   *
   * @param q_matrix_ratio
   */
  static void SetQMatrixRatio(double q_matrix_ratio) {
    s_q_matrix_ratio_ = q_matrix_ratio;
  }

 private:
  Eigen::Vector3d belief_anchor_point_;
  Eigen::Vector3d belief_velocity_;
  Eigen::Vector4d priori_state_;
  Eigen::Vector4d posteriori_state_;
  Eigen::Matrix4d p_matrix_;
  // the state-transition matrix
  Eigen::Matrix4d a_matrix_;
  // the observation mode
  Eigen::Matrix4d c_matrix_;
  // the covariance of the process noise
  Eigen::Matrix4d q_matrix_;
  //  the covariance of the observation noise
  Eigen::Matrix4d r_matrix_;
  // Optimal Kalman gain
  Eigen::Matrix4d k_matrix_;
  static double s_q_matrix_ratio_;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
