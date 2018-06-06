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

#include "modules/perception/obstacle/ultrasonic/common/ultrasonic_kalman_filter.h"

namespace apollo {
namespace perception {

static constexpr int kUltrasonicMaxTrackCount = 5;

UltrasonicKalmanFilter::UltrasonicKalmanFilter() {
    name_ = "UltrasonicKalmanFilter";
    Initialize(0);
}

UltrasonicKalmanFilter::~UltrasonicKalmanFilter() {}

void UltrasonicKalmanFilter::Initialize(const float& rho_init) {
    // System evoluation matrix
    a_matrix_ = 1;
    // Initialize states to the states of the detected obstacle
    states_ = rho_init;
    // Initialize H, Q, R, P matrices
    h_matrix_ = 1;
    q_matrix_ = 1;
    r_matrix_ = 5;
    p_matrix_ = 1;
    track_count_ = 0;
    lost_count_ = 0;

    if (states_ > 0) {
        track_count_ = 1;
        lost_count_ = 0;
    }
}

void UltrasonicKalmanFilter::Predict(const double time_diff) {
    states_predict_ = a_matrix_ * states_;
    p_matrix_ = ((a_matrix_ * p_matrix_) * a_matrix_) + q_matrix_;
}

void UltrasonicKalmanFilter::UpdateWithObject(
    const float rho_new, const double time_diff) {
  Predict(time_diff);
  k_matrix_ = p_matrix_ * h_matrix_ /
              (h_matrix_ * p_matrix_ * h_matrix_ + r_matrix_);

  states_ = states_predict_ + k_matrix_ * (rho_new - states_predict_);
  p_matrix_ = p_matrix_ - k_matrix_*h_matrix_*p_matrix_;

  while (track_count_ < kUltrasonicMaxTrackCount) {
    ++track_count_;
  }
  lost_count_ = 0;
}

void UltrasonicKalmanFilter::UpdateWithoutObject(const double time_diff) {
    Predict(time_diff);
    k_matrix_ = p_matrix_ * h_matrix_ /
                (h_matrix_ * p_matrix_ * h_matrix_ + r_matrix_);

    states_ = states_predict_;
    p_matrix_ = p_matrix_ - k_matrix_ * h_matrix_ * p_matrix_;
    ++lost_count_;
    track_count_ = 1;
}

float UltrasonicKalmanFilter::GetState() {
    return states_;
}

int UltrasonicKalmanFilter::GetTrackCount() {
    return track_count_;
}

int UltrasonicKalmanFilter::GetLostCount() {
    return lost_count_;
}

void UltrasonicKalmanFilter::Reset() {
    Initialize(0);
}

}  // namespace perception
}  // namespace apollo
