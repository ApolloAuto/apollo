/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 
#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_PREPROCESS_ADAPTIVE_KALMAN_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_PREPROCESS_ADAPTIVE_KALMAN_FILTER_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "modules/perception/obstacle/radar/interface/base_filter.h"

namespace apollo {
namespace perception {

class AdaptiveKalmanFilter : public BaseFilter {
public:
    AdaptiveKalmanFilter();

    ~AdaptiveKalmanFilter();

    void Initialize(const Object& state);

    Eigen::Vector4d Predict(const double time_diff);

    Eigen::Vector4d UpdateWithObject(Object& new_object);

    Eigen::Matrix4d GetCovarianceMatrix() {return p_matrix_;}
    
    void get_state(Eigen::Vector3d& anchor_point, Eigen::Vector3d& velocity);

private:
    Eigen::Vector3d  belief_anchor_point_;
    Eigen::Vector3d  belief_velocity_;

    Eigen::Vector4d  state_;
    Eigen::Vector4d  priori_state_;
    Eigen::Vector4d  posteriori_state_;

    Eigen::Matrix4d  p_matrix_;
    // the state-transition matrix
    Eigen::Matrix4d  a_matrix_;
    // the observation mode
    Eigen::Matrix4d  c_matrix_;

    // the covariance of the process noise
    Eigen::Matrix4d  q_matrix_;
    //  the covariance of the observation noise
    Eigen::Matrix4d  r_matrix_;

    // Optimal Kalman gain
    Eigen::Matrix4d  k_matrix_;
};

} // namespace perception
} // namespace apollo

#endif // MODULES_PERCEPTION_OBSTACLE_RADAR_PREPROCESS_ADAPTIVE_KALMAN_FILTER_H_
