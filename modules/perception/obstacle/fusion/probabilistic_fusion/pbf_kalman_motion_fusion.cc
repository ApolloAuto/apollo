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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

PbfKalmanMotionFusion::PbfKalmanMotionFusion() {
    initialized_ = false;
    name_ = "PbfKalmanMotionFusion";
}

PbfKalmanMotionFusion::~PbfKalmanMotionFusion() {

}

void PbfKalmanMotionFusion::Initialize(const Eigen::Vector3d& anchor_point,
    const Eigen::Vector3d& velocity) {

    belief_anchor_point_ = anchor_point;
    belief_velocity_ = velocity;
}

void PbfKalmanMotionFusion::Initialize(const PbfSensorObjectPtr new_object) {
    ACHECK(new_object != nullptr && new_object->object != nullptr)
        << "Initialize PbfKalmanMotionFusion with null sensor object";

    if (is_lidar(new_object->sensor_type)) {
        belief_anchor_point_ = new_object->object->anchor_point;
        belief_velocity_ = new_object->object->velocity;
        initialized_ = true;
    } else if (is_radar(new_object->sensor_type)) {
        belief_anchor_point_ = new_object->object->anchor_point;
        belief_velocity_ = new_object->object->velocity;
        initialized_ = true;
    } 

    a_matrix_.setIdentity();
    a_matrix_(0, 2) = 0.05;
    a_matrix_(1, 3) = 0.05;
    // initialize states to the states of the detected obstacle
    posteriori_state_(0) = belief_anchor_point_(0);
    posteriori_state_(1) = belief_anchor_point_(1);
    posteriori_state_(2) = belief_velocity_(0);
    posteriori_state_(3) = belief_velocity_(1);
    priori_state_ = posteriori_state_;

    q_matrix_.setIdentity();

    r_matrix_.setIdentity();
    r_matrix_.topLeftCorner(2, 2) = new_object->object->position_uncertainty.topLeftCorner(2, 2);
    r_matrix_.block<2, 2>(2, 2) = new_object->object->velocity_uncertainty.topLeftCorner(2, 2);

    p_matrix_.setIdentity();
    p_matrix_.topLeftCorner(2, 2) = new_object->object->position_uncertainty.topLeftCorner(2, 2);
    p_matrix_.block<2, 2>(2, 2) = new_object->object->velocity_uncertainty.topLeftCorner(2, 2);
    c_matrix_.setIdentity();
}

void PbfKalmanMotionFusion::Predict(Eigen::Vector3d& anchor_point,
    Eigen::Vector3d& velocity, const double time_diff) {

    anchor_point = belief_anchor_point_ + belief_velocity_ * time_diff;
    velocity = belief_velocity_;
}

void PbfKalmanMotionFusion::UpdateWithObject(const PbfSensorObjectPtr new_object,
                                             const double time_diff) {
    ACHECK(new_object != nullptr && new_object->object != nullptr)
        << "update PbfKalmanMotionFusion with null sensor object";

    //predict and then correct
    a_matrix_.setIdentity();
    a_matrix_(0, 2) = time_diff;
    a_matrix_(1, 3) = time_diff;

    priori_state_  = a_matrix_ * posteriori_state_;
    p_matrix_ = ((a_matrix_ * p_matrix_) * a_matrix_.transpose()) + q_matrix_;

    if (new_object->sensor_type == VELODYNE_64) {
        belief_anchor_point_ = new_object->object->center;
        belief_velocity_ = new_object->object->velocity;
    } else if (new_object->sensor_type == RADAR) {
        belief_anchor_point_(0) = new_object->object->center(0);
        belief_anchor_point_(1) = new_object->object->center(1);
        belief_velocity_(0) = new_object->object->velocity(0);
        belief_velocity_(1) = new_object->object->velocity(1);
    } else {
        AERROR << "unsupported sensor type for PbfKalmanMotionFusion: "
            << new_object->sensor_type;
        return;
    }

    Eigen::Vector4d measurement;
    measurement(0) = belief_anchor_point_(0);
    measurement(1) = belief_anchor_point_(1);
    measurement(2) = belief_velocity_(0);
    measurement(3) = belief_velocity_(1);

    //r_matrix_ = new_object.uncertainty_mat;
    r_matrix_.setIdentity();
    r_matrix_.topLeftCorner(2, 2) = new_object->object->position_uncertainty.topLeftCorner(2, 2);
    r_matrix_.block<2, 2>(2, 2) = new_object->object->velocity_uncertainty.topLeftCorner(2, 2);

    k_matrix_ = p_matrix_ * c_matrix_.transpose() *
        (c_matrix_ * p_matrix_ * c_matrix_.transpose() + r_matrix_).inverse();

    Eigen::Vector4d predict_measurement(priori_state_(0), priori_state_(1),
                priori_state_(2), priori_state_(3));

    posteriori_state_ = priori_state_ + k_matrix_ * (measurement - predict_measurement);
    p_matrix_ = (Eigen::Matrix4d::Identity() - k_matrix_ * c_matrix_) * p_matrix_
        * (Eigen::Matrix4d::Identity() - k_matrix_ * c_matrix_).transpose()
        + k_matrix_ * r_matrix_ * k_matrix_.transpose();

    belief_anchor_point_(0) = posteriori_state_(0);
    belief_anchor_point_(1) = posteriori_state_(1);
    belief_velocity_(0) = posteriori_state_(2);
    belief_velocity_(1) = posteriori_state_(3);
}

void PbfKalmanMotionFusion::UpdateWithoutObject(const double time_diff) {

    belief_anchor_point_ = belief_anchor_point_ + belief_velocity_ * time_diff;
}

void PbfKalmanMotionFusion::GetState(Eigen::Vector3d& anchor_point, Eigen::Vector3d& velocity) {

    anchor_point = belief_anchor_point_;
    velocity = belief_velocity_;
}

} // namespace perception
} // namespace apollo
