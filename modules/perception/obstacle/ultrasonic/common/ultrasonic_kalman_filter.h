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

#ifndef MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_ULTRASONIC_KALMAN_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_ULTRASONIC_KALMAN_FILTER_H_

#include <string>
#include <vector>
#include "boost/shared_ptr.hpp"
#include "Eigen/Core"

namespace apollo {
namespace perception {

class UltrasonicKalmanFilter{
 public:
  UltrasonicKalmanFilter();

  ~UltrasonicKalmanFilter();

  void Initialize(const float& rho_init);

  void Predict(const double time_diff);

  void UpdateWithObject(const float rho_new, const double time_diff);

  void UpdateWithoutObject(const double time_diff);

  float GetState();

  int GetTrackCount();

  int GetLostCount();

  void Reset();

 private:
  std::string name_;
  float states_;
  float states_predict_;

  float p_matrix_;  // predicted estimate covariance matrix
  float a_matrix_;  // state transition matrix
  float q_matrix_;  // covariance matrix for process noise
  float r_matrix_;  // covariance matrix for observation noise
  float h_matrix_;  // state transition matrix
  float k_matrix_;  // kalman gain
  int track_count_;
  int lost_count_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ULTRASONIC_ULTRASONIC_KALMAN_FILTER_H_
