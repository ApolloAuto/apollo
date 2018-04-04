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

  // initialize states to the states of the detected obstacle
  _posteriori_state(0) = _belief_anchor_point(0);
  _posteriori_state(1) = _belief_anchor_point(1);
  _posteriori_state(2) = _belief_velocity(0);
  _posteriori_state(3) = _belief_velocity(1);
  _priori_state = _posteriori_state;
  _omega_matrix = new_object->object->state_uncertainty;
  _omega_matrix = _omega_matrix.inverse().eval();
  _xi = _omega_matrix * _posteriori_state;
  _a_matrix.setIdentity();
  _a_matrix(0, 2) = 0.05;
  _a_matrix(1, 3) = 0.05;
  _q_matrix.setIdentity();
  ADEBUG << "PBFIMF pbf imf filter initial state is " << _posteriori_state(0)
         << " " << _posteriori_state(1) << " " << _posteriori_state(2) << " "
         << _posteriori_state(3)
         << " for object info: " << new_object->object->ToString();
  std::cerr << "PBFIMF pbf imf initial uncertainty set "
            << new_object->object->state_uncertainty << std::endl;
  CacheSensorObjects(new_object);
  initialized_ = true;
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

  // remove outdated object
  RemoveOutdatedSensorObjects(new_object->timestamp);

  // print some debug information for debugging
  ADEBUG << "\nPBFIMF: previous state: " << _belief_anchor_point(0) << " "
         << _belief_anchor_point(1) << " " << _belief_velocity(0) << " "
         << _belief_velocity(1);
  ADEBUG << "PBFIMF: previous timestamp: " << std::fixed
         << std::setprecision(15) << last_fuse_timestamp;
  ADEBUG << "PBFIMF: new object information: "
         << new_object->object->ToString();
  ADEBUG << "PBFIMF: new object timestamp: " << std::fixed
         << std::setprecision(15) << new_object->timestamp;
  ADEBUG << "PBFIMF: time diff is " << time_diff;

  // compute priori
  _a_matrix.setIdentity();
  _a_matrix(0, 2) = time_diff;
  _a_matrix(1, 3) = time_diff;
  _q_matrix.setIdentity();
  _q_matrix = _q_matrix / 10;
  _q_matrix = _q_matrix * time_diff;
  _priori_state = _a_matrix * _posteriori_state;
  _omega_matrix =
      (_a_matrix * _omega_matrix.inverse() * _a_matrix.transpose() + _q_matrix);
  _omega_matrix = _omega_matrix.inverse().eval();
  ADEBUG << "PBFIMF:predicted state " << _priori_state(0) << " "
         << _priori_state(1) << " " << _priori_state(2) << " "
         << _priori_state(3);
  _xi = _omega_matrix * _priori_state;

  // sensor level processor noise matrix and trans matrix
  const Eigen::Matrix4d* sensor_processor_noise;
  const Eigen::Matrix4d* sensor_transition_matrix;

  if (new_object->sensor_type == SensorType::CAMERA) {
    _belief_anchor_point = new_object->object->center;
    _belief_velocity = new_object->object->velocity;
    if (!CameraFrameSupplement::state_vars.initialized_) {
      AERROR << "process noise and trans matrix not initialized for camera";
      return;
    }
    sensor_processor_noise = &(CameraFrameSupplement::state_vars.process_noise);
    sensor_transition_matrix =
        &(CameraFrameSupplement::state_vars.trans_matrix);
  } else if (new_object->sensor_type == SensorType::RADAR) {
    _belief_anchor_point = new_object->object->center;
    _belief_velocity = new_object->object->velocity;

    // for radar, we don't set externally yet, just use default value
    /*if (!RadarFrameSupplement::state_vars.initialized_) {
      AERROR << "process noise and trans matrix not initialized for radar";
      return;
    }*/
    sensor_processor_noise = &(RadarFrameSupplement::state_vars.process_noise);
    sensor_transition_matrix = &(RadarFrameSupplement::state_vars.trans_matrix);
  } else if (new_object->sensor_type == SensorType::VELODYNE_64) {
    _belief_anchor_point = new_object->object->center;
    _belief_velocity = new_object->object->velocity;
    if (!LidarFrameSupplement::state_vars.initialized_) {
      AERROR << "process noise and trans matrix not initialized for camera";
      return;
    }
    sensor_processor_noise = &(LidarFrameSupplement::state_vars.process_noise);
    sensor_transition_matrix = &(LidarFrameSupplement::state_vars.trans_matrix);
  } else {
    AERROR << "unsupported sensor type, setting using default value";
    return;
  }

  const PbfSensorObjectPtr sensor_object =
      GetSensorLatestCache(new_object->sensor_type);

  if (sensor_object != nullptr) {
    Eigen::Matrix4d cov_sensor_prev = Eigen::Matrix4d::Identity();
    Eigen::Vector4d state_sensor_prev = Eigen::Vector4d::Zero();
    double timestamp_sensor_prev = sensor_object->timestamp;

    if (!ObtainSensorPrediction(sensor_object->object, timestamp_sensor_prev,
                                *sensor_processor_noise,
                                *sensor_transition_matrix, &state_sensor_prev,
                                &cov_sensor_prev)) {
      AERROR << "obtain previous sensor prediction fails";
      return;
    }

    Eigen::Matrix4d cov_sensor = Eigen::Matrix4d::Identity();
    Eigen::Vector4d state_sensor = Eigen::Vector4d::Zero();
    double timestamp_sensor = new_object->timestamp;

    if (!ObtainSensorPrediction(
            new_object->object, timestamp_sensor, *sensor_processor_noise,
            *sensor_transition_matrix, &state_sensor, &cov_sensor)) {
      AERROR << "obtain current sensor prediction fails";
      return;
    }

    Eigen::Matrix4d cov_sensor_inverse = cov_sensor.inverse();
    Eigen::Matrix4d cov_sensor_prev_inverse = cov_sensor_prev.inverse();
    ADEBUG << "PBFIMF: state sensor " << state_sensor(0) << " "
           << state_sensor(1);
    ADEBUG << "PBFIMF: state sensor prev " << state_sensor_prev(0) << " "
           << state_sensor(1);

    _omega_matrix =
        _omega_matrix + (cov_sensor_inverse - cov_sensor_prev_inverse);

    std::cerr << "PBFIMF: information prediction" << _xi << std::endl;
    std::cerr << "PBFIMF: information delta "
              << (cov_sensor_inverse * state_sensor -
                  cov_sensor_prev_inverse * state_sensor_prev)
              << std::endl;

    _xi = _xi + (cov_sensor_inverse * state_sensor -
                 cov_sensor_prev_inverse * state_sensor_prev);
  } else {
    // this case is weird, might lead to unexpected situation
    Eigen::Matrix4d cov_sensor = Eigen::Matrix4d::Identity();
    Eigen::Vector4d state_sensor = Eigen::Vector4d::Zero();
    double timestamp_sensor = new_object->timestamp;

    if (!ObtainSensorPrediction(
            new_object->object, timestamp_sensor, *sensor_processor_noise,
            *sensor_transition_matrix, &state_sensor, &cov_sensor)) {
      AERROR << "obtain current sensor prediction fails";
      return;
    }
    AWARN
        << "Sensor data deprecation in Fusion, should not see this many times";
    _omega_matrix = 0.5 * _omega_matrix + 0.5 * cov_sensor.inverse();
    _xi = 0.5 * _xi + 0.5 * cov_sensor.inverse() * state_sensor;
  }
  _posteriori_state = _omega_matrix.inverse() * _xi;
  _belief_anchor_point(0) = _posteriori_state(0);
  _belief_anchor_point(1) = _posteriori_state(1);
  _belief_velocity(0) = _posteriori_state(2);
  _belief_velocity(1) = _posteriori_state(3);

  ADEBUG << "PBFIMF: new state is " << _belief_anchor_point(0) << " "
         << _belief_anchor_point(1) << " " << _belief_velocity(0) << " "
         << _belief_velocity(1) << "\n";
  CacheSensorObjects(new_object);
}

/**
 * obtain sensor level prediction to global fusion arrival time
 * @param obj
 * @param sensor_timestamp
 * @param process_noise
 * @param trans_matrix
 * @param state_pre
 * @param cov_pre
 */
bool PbfIMFFusion::ObtainSensorPrediction(ObjectPtr object,
                                          double sensor_timestamp,
                                          const Eigen::Matrix4d& process_noise,
                                          const Eigen::Matrix4d& trans_matrix,
                                          Eigen::Vector4d* state_pre,
                                          Eigen::Matrix4d* cov_pre) {
  Eigen::Matrix<double, 4, 4>& cov = object->state_uncertainty;
  Eigen::Matrix<double, 4, 1> state;

  // state: x,y,vx,vy
  state(0) = object->center(0);
  state(1) = object->center(1);
  state(2) = object->velocity(0);
  state(3) = object->velocity(1);

  double time_diff = fuse_timestamp - sensor_timestamp;
  Eigen::Matrix4d process_noise_time = process_noise * time_diff;

  std::cerr << "PBFIMF: OBTAIN PREDICT: cov is\n " << cov << std::endl;
  std::cerr << "PBFIMF: OBTAIN PREDICT: state is\n " << state << std::endl;
  std::cerr << "PBFIMF: OBTAIN PREDICT: time diff is\n " << time_diff
            << std::endl;
  std::cerr << "PBFIMF: OBTAIN PREDICT: process noise is\n " << process_noise
            << std::endl;

  // trans_matrix is F matrix for state transition
  // p_pre is sensor level covariance prediction P(ki|ki-1)
  // state_pre is sensor level state prediction x(ki|ki-1)
  Eigen::Matrix4d trans_matrix_time = trans_matrix;
  trans_matrix_time(0, 2) = time_diff;
  trans_matrix_time(1, 3) = time_diff;
  (*state_pre) = trans_matrix_time * state;
  std::cerr << "PBFIMF: OBTAIN PREDICT: trans matrix time is\n "
            << trans_matrix_time << std::endl;
  (*cov_pre) = trans_matrix_time * cov * trans_matrix_time.transpose() +
               process_noise_time;
  return true;
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

    if (it->first == SensorType::VELODYNE_64) {
      time_invisible = PbfTrack::GetMaxLidarInvisiblePeriod();
    } else if (it->first == SensorType::RADAR) {
      time_invisible = PbfTrack::GetMaxRadarInvisiblePeriod();
    } else if (it->first == SensorType::CAMERA) {
      time_invisible = PbfTrack::GetMaxCameraInvisiblePeriod();
    } else {
      AERROR << "Unexpected sensor type!";
    }

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

void PbfIMFFusion::SetState(const Eigen::Vector3d& anchor_point,
                            const Eigen::Vector3d& velocity) {
  _belief_anchor_point = anchor_point;
  _belief_velocity = velocity;
}

}  // namespace perception
}  // namespace apollo
