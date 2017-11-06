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
    _initialized = false;
    _name = "PbfKalmanMotionFusion";
}

PbfKalmanMotionFusion::~PbfKalmanMotionFusion() {

}

void PbfKalmanMotionFusion::initialize(const Eigen::Vector3d& anchor_point,
    const Eigen::Vector3d& velocity) {

    _belief_anchor_point = anchor_point;
    _belief_velocity = velocity;
}

void PbfKalmanMotionFusion::initialize(const PbfSensorObjectPtr new_object) {
    ACHECK(new_object != nullptr && new_object->object != nullptr)
        << "Initialize PbfKalmanMotionFusion with null sensor object";

    if (is_lidar(new_object->sensor_type)) {
        _belief_anchor_point = new_object->object->anchor_point;
        _belief_velocity = new_object->object->velocity;
        _initialized = true;
    } else if (is_radar(new_object->sensor_type)) {
        _belief_anchor_point = new_object->object->anchor_point;
        _belief_velocity = new_object->object->velocity;
        _initialized = true;
    } 

    _a_matrix.setIdentity();
    _a_matrix(0, 2) = 0.05;
    _a_matrix(1, 3) = 0.05;
    // initialize states to the states of the detected obstacle
    _posteriori_state(0) = _belief_anchor_point(0);
    _posteriori_state(1) = _belief_anchor_point(1);
    _posteriori_state(2) = _belief_velocity(0);
    _posteriori_state(3) = _belief_velocity(1);
    _priori_state = _posteriori_state;

    _q_matrix.setIdentity();

    _r_matrix.setIdentity();
    _r_matrix.topLeftCorner(2, 2) = new_object->object->position_uncertainty.topLeftCorner(2, 2);
    _r_matrix.block<2, 2>(2, 2) = new_object->object->velocity_uncertainty.topLeftCorner(2, 2);

    _p_matrix.setIdentity();
    _p_matrix.topLeftCorner(2, 2) = new_object->object->position_uncertainty.topLeftCorner(2, 2);
    _p_matrix.block<2, 2>(2, 2) = new_object->object->velocity_uncertainty.topLeftCorner(2, 2);
    _c_matrix.setIdentity();
}

void PbfKalmanMotionFusion::predict(Eigen::Vector3d& anchor_point,
    Eigen::Vector3d& velocity, const double time_diff) {

    anchor_point = _belief_anchor_point + _belief_velocity * time_diff;
    velocity = _belief_velocity;
}

void PbfKalmanMotionFusion::update_with_object(const PbfSensorObjectPtr new_object,
    const double time_diff) {
    ACHECK(new_object != nullptr && new_object->object != nullptr)
        << "update PbfKalmanMotionFusion with null sensor object";

    //predict and then correct
    _a_matrix.setIdentity();
    _a_matrix(0, 2) = time_diff;
    _a_matrix(1, 3) = time_diff;

    _priori_state  = _a_matrix * _posteriori_state;
    _p_matrix = ((_a_matrix * _p_matrix) * _a_matrix.transpose()) + _q_matrix;

    if (new_object->sensor_type == VELODYNE_64) {
        _belief_anchor_point = new_object->object->center;
        _belief_velocity = new_object->object->velocity;
    } else if (new_object->sensor_type == RADAR) {
        _belief_anchor_point(0) = new_object->object->center(0);
        _belief_anchor_point(1) = new_object->object->center(1);
        _belief_velocity(0) = new_object->object->velocity(0);
        _belief_velocity(1) = new_object->object->velocity(1);
    } else {
        AERROR << "unsupported sensor type for PbfKalmanMotionFusion: "
            << new_object->sensor_type;
        return;
    }

    Eigen::Vector4d measurement;
    measurement(0) = _belief_anchor_point(0);
    measurement(1) = _belief_anchor_point(1);
    measurement(2) = _belief_velocity(0);
    measurement(3) = _belief_velocity(1);

    //_r_matrix = new_object.uncertainty_mat;
    _r_matrix.setIdentity();
    _r_matrix.topLeftCorner(2, 2) = new_object->object->position_uncertainty.topLeftCorner(2, 2);
    _r_matrix.block<2, 2>(2, 2) = new_object->object->velocity_uncertainty.topLeftCorner(2, 2);

    _k_matrix = _p_matrix * _c_matrix.transpose() *
        (_c_matrix * _p_matrix * _c_matrix.transpose() + _r_matrix).inverse();

    Eigen::Vector4d predict_measurement(_priori_state(0), _priori_state(1),
                _priori_state(2), _priori_state(3));

    _posteriori_state = _priori_state + _k_matrix * (measurement - predict_measurement);
    _p_matrix = (Eigen::Matrix4d::Identity() - _k_matrix * _c_matrix) * _p_matrix
        * (Eigen::Matrix4d::Identity() - _k_matrix * _c_matrix).transpose()
        + _k_matrix * _r_matrix * _k_matrix.transpose();

    _belief_anchor_point(0) = _posteriori_state(0);
    _belief_anchor_point(1) = _posteriori_state(1);
    _belief_velocity(0) = _posteriori_state(2);
    _belief_velocity(1) = _posteriori_state(3);
}

void PbfKalmanMotionFusion::update_without_object(const double time_diff) {

    _belief_anchor_point = _belief_anchor_point + _belief_velocity * time_diff;
}

void PbfKalmanMotionFusion::get_state(Eigen::Vector3d& anchor_point, Eigen::Vector3d& velocity) {

    anchor_point = _belief_anchor_point;
    velocity = _belief_velocity;
}

} // namespace perception
} // namespace apollo
