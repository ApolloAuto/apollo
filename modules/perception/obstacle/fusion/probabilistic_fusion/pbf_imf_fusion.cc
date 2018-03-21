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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_imf_fusion.h"  // NOLINT
#include "modules/common/log.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

PbfIMFFusion::PbfIMFFusion() {
  initialized_ = false;
  name_ = "PbfInformationMotionFusion";
}

PbfIMFFusion::~PbfIMFFusion() {}

void PbfIMFFusion::Initialize(const Eigen::Vector3d& anchor_point,
                              const Eigen::Vector3d& velocity) {
  _belief_anchor_point = anchor_point;
  _belief_velocity = velocity;
}

void PbfIMFFusion::Initialize(const PbfSensorObjectPtr new_object) {
  if (new_object == nullptr) {
    AERROR << "Initialize PbfInformationMotionFusion with null sensor object";
    return;
  }
  if (new_object->object == nullptr) {
    AERROR << "Initialize PbfInformationMotionFusion with null object";
    return;
  }

  _belief_anchor_point = new_object->object->anchor_point;
  _belief_velocity = new_object->object->velocity;
  initialized_ = true;

  // initialize states to the states of the detected obstacle
  _posteriori_state(0) = _belief_anchor_point(0);
  _posteriori_state(1) = _belief_anchor_point(1);
  _posteriori_state(2) = _belief_velocity(0);
  _posteriori_state(3) = _belief_velocity(1);
  _priori_state = _posteriori_state;
  _omega_matrix.setIdentity();
  // _omega_matrix.topLeftCorner(2, 2) =
  //     new_object->object->position_uncertainty.topLeftCorner(2, 2);
  // _omega_matrix.block<2, 2>(2, 2) =
  // new_object->object->velocity_uncertainty.topLeftCorner(2, 2);
  // _omega_matrix.block<2, 2>(4, 4) =
  // new_object->object->acceleration_uncertainty.topLeftCorner(2, 2);
  _omega_matrix = new_object->object->uncertainty;
  _omega_matrix = _omega_matrix.inverse().eval();
  _xi = _omega_matrix * _posteriori_state;
  _a_matrix.setIdentity();
  _a_matrix(0, 2) = 0.05;
  _a_matrix(1, 3) = 0.05;
  _c_matrix.setIdentity();
  _q_matrix.setIdentity();
  _q_matrix = _q_matrix * 0.05;
  _r_matrix_inverse.setIdentity();
  _r_matrix_inverse = new_object->object->uncertainty;
  _r_matrix_inverse = _r_matrix_inverse.inverse().eval();
  CacheSensorObjects(new_object);
}

void PbfIMFFusion::Predict(Eigen::Vector3d* anchor_point,
                           Eigen::Vector3d* velocity, const double time_diff) {
  *anchor_point = _belief_anchor_point + _belief_velocity * time_diff;
  *velocity = _belief_velocity;
}

void PbfIMFFusion::UpdateWithObject(const PbfSensorObjectPtr new_object,
                                    const double time_diff) {
  if (new_object == nullptr) {
    AERROR << "update PbfInformationMotionFusion with null sensor object";
    return;
  }
  if (new_object->object == nullptr) {
    AERROR << "update PbfInformationMotionFusion with null object";
    return;
  }
  const double timestamp = new_object->timestamp;
  RemoveOutdatedSensorObjects(timestamp);
  // compute priori
  _a_matrix.setIdentity();
  _a_matrix(0, 2) = time_diff;
  _a_matrix(1, 3) = time_diff;

  _q_matrix.setIdentity();
  _q_matrix = _q_matrix * 0.1 * time_diff;
  _priori_state = _a_matrix * _posteriori_state;
  _omega_matrix =
      (_a_matrix * _omega_matrix.inverse() * _a_matrix.transpose() + _q_matrix);
  _omega_matrix = _omega_matrix.inverse().eval();
  _xi = _omega_matrix * _priori_state;
  if (new_object->sensor_type == SensorType::VELODYNE_64) {
    _belief_anchor_point = new_object->object->center;
    _belief_velocity = new_object->object->velocity;
  } else if (new_object->sensor_type == SensorType::RADAR) {
    _belief_anchor_point(0) = new_object->object->center(0);
    _belief_anchor_point(1) = new_object->object->center(1);
    _belief_velocity(0) = new_object->object->velocity(0);
    _belief_velocity(1) = new_object->object->velocity(1);
  } else if (new_object->sensor_type == SensorType::CAMERA) {
    _belief_anchor_point = new_object->object->center;
    _belief_velocity = new_object->object->velocity;
  } else {
    AERROR << "unsupported sensor type";
    return;
  }
  // compute posteriori
  Eigen::Matrix<double, 4, 1> measurement;
  measurement(0) = _belief_anchor_point(0);
  measurement(1) = _belief_anchor_point(1);
  measurement(2) = _belief_velocity(0);
  measurement(3) = _belief_velocity(1);

  // updated covariance matrix at sensor level ki
  _r_matrix_inverse.setIdentity();
  _r_matrix_inverse = new_object->object->uncertainty;
  _r_matrix_inverse = _r_matrix_inverse.inverse().eval();
  const PbfSensorObjectPtr sensor_object =
      GetSensorLatestCache(new_object->sensor_type);

  if (sensor_object != nullptr) {
    // if (false) {
    // Not first time.
    Eigen::Matrix<double, 4, 4> p_pre;
    p_pre.setIdentity();
    p_pre = sensor_object->object->uncertainty;
    Eigen::Matrix<double, 4, 1> state_pre;
    state_pre(0) = sensor_object->object->center(0);
    state_pre(1) = sensor_object->object->center(1);
    state_pre(2) = sensor_object->object->velocity(0);
    state_pre(3) = sensor_object->object->velocity(1);

    double time_diff_pre = new_object->timestamp - sensor_object->timestamp;
    Eigen::Matrix<double, 4, 4> trans_matrix;
    trans_matrix.setIdentity();
    trans_matrix(0, 2) = time_diff_pre;
    trans_matrix(1, 3) = time_diff_pre;
    // TODO(zhangweide): use sensor process noise is more reasonable.
    Eigen::Matrix<double, 4, 4> q_matrix_pre;
    q_matrix_pre.setIdentity();
    q_matrix_pre = q_matrix_pre * 0.1 * time_diff_pre;

    // trans_matrix is F matrix for state transition
    // p_pre is sensor level covariance prediction P(ki|ki-1)
    // state_pre is sensor level state prediction x(ki|ki-1)

    state_pre = trans_matrix * state_pre;
    p_pre = trans_matrix * p_pre * trans_matrix.transpose() + q_matrix_pre;

    _omega_matrix = _c_matrix.transpose() * _omega_matrix +
                    (_c_matrix.transpose() * _r_matrix_inverse * _c_matrix -
                     _c_matrix.transpose() * p_pre.inverse() * _c_matrix);
    _xi = _xi + (_c_matrix.transpose() * _r_matrix_inverse * measurement -
                 _c_matrix.transpose() * p_pre.inverse() * state_pre);
  } else {
    // First time.
    _omega_matrix =
        _omega_matrix + _c_matrix.transpose() * _r_matrix_inverse * _c_matrix;
    _xi = _xi + _c_matrix.transpose() * _r_matrix_inverse * measurement;
  }
  _posteriori_state = _omega_matrix.inverse() * _xi;
  _belief_anchor_point(0) = _posteriori_state(0);
  _belief_anchor_point(1) = _posteriori_state(1);
  _belief_velocity(0) = _posteriori_state(2);
  _belief_velocity(1) = _posteriori_state(3);
  CacheSensorObjects(new_object);
}

void PbfIMFFusion::UpdateWithoutObject(const double time_diff) {
  _belief_anchor_point = _belief_anchor_point + _belief_velocity * time_diff;
}

void PbfIMFFusion::GetState(Eigen::Vector3d* anchor_point,
                            Eigen::Vector3d* velocity) {
  *anchor_point = _belief_anchor_point;
  *velocity = _belief_velocity;
}

void PbfIMFFusion::GetUncertainty(Eigen::Matrix3d* position_uncertainty,
                                  Eigen::Matrix3d* velocity_uncertainty) {
  *position_uncertainty << 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01;
  *velocity_uncertainty << 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01;
  (*position_uncertainty).topLeftCorner(2, 2) =
      _omega_matrix.inverse().topLeftCorner(2, 2);
  (*velocity_uncertainty).topLeftCorner(2, 2) =
      _omega_matrix.inverse().block<2, 2>(2, 2);
}

void PbfIMFFusion::CacheSensorObjects(const PbfSensorObjectPtr new_object) {
  const SensorType& type = new_object->sensor_type;
  auto it = _cached_sensor_objects.find(type);
  if (it != _cached_sensor_objects.end()) {
    it->second.push(new_object);
  } else {
    std::queue<PbfSensorObjectPtr> objects;
    objects.push(new_object);
    _cached_sensor_objects[type] = objects;
  }
}

void PbfIMFFusion::RemoveOutdatedSensorObjects(const double timestamp) {
  auto it = _cached_sensor_objects.begin();
  for (; it != _cached_sensor_objects.end(); ++it) {
    double time_invisible = 0.0;
    /*if (it->first == SensorType::VELODYNE_64) {
        time_invisible = PbfTrack::get_max_lidar_invisible_period();
    } else if (it->first == SensorType::RADAR) {
        time_invisible = PbfTrack::get_max_radar_invisible_period();
    } else if (it->first == SensorType::CAMERA) {
        time_invisible = PbfTrack::get_max_camera_invisible_period();
    } else {
        AERROR << "Unexpected sensor type!";
    }*/
    auto& objects = it->second;
    while (!objects.empty()) {
      const auto& object = objects.front();
      if (timestamp - object->timestamp > time_invisible) {
        objects.pop();
      } else {
        break;
      }
    }
  }
}

PbfSensorObjectPtr PbfIMFFusion::GetSensorLatestCache(const SensorType type) {
  auto it = _cached_sensor_objects.find(type);
  if (it != _cached_sensor_objects.end()) {
    const auto& objects = it->second;
    if (!objects.empty()) {
      return objects.back();
    }
  }
  return nullptr;
}

}  // namespace perception
}  // namespace apollo
