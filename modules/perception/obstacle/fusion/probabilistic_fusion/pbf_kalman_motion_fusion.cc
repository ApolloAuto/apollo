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

#include "modules/common/log.h"
#include "modules/perception/common/geometry_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

PbfKalmanMotionFusion::PbfKalmanMotionFusion() {
  initialized_ = false;
  name_ = "PbfKalmanMotionFusion";
}

PbfKalmanMotionFusion::~PbfKalmanMotionFusion() {}

void PbfKalmanMotionFusion::Initialize(const Eigen::Vector3d &anchor_point,
                                       const Eigen::Vector3d &velocity) {
  belief_anchor_point_ = anchor_point;
  belief_velocity_ = velocity;
  belief_acceleration_ = Eigen::Vector3d(0, 0, 0);
}

void PbfKalmanMotionFusion::Initialize(
    const std::shared_ptr<PbfSensorObject> new_object) {
  ACHECK(new_object != nullptr && new_object->object != nullptr)
      << "Initialize PbfKalmanMotionFusion with null sensor object";

  if (is_lidar(new_object->sensor_type)) {
    belief_anchor_point_ = new_object->object->anchor_point;
    belief_velocity_ = new_object->object->velocity;
    belief_acceleration_ = Eigen::Vector3d(0, 0, 0);
    initialized_ = true;
  } else if (is_radar(new_object->sensor_type)) {
    belief_anchor_point_ = new_object->object->anchor_point;
    belief_velocity_ = new_object->object->velocity;
    belief_acceleration_ = Eigen::Vector3d(0, 0, 0);
    initialized_ = true;
  } else if (is_camera(new_object->sensor_type)) {
    belief_anchor_point_ = new_object->object->anchor_point;
    belief_velocity_ = new_object->object->velocity;
    belief_acceleration_ = Eigen::Vector3d(0, 0, 0);
    initialized_ = true;
  }

  a_matrix_.setIdentity();
  a_matrix_(0, 2) = FLAGS_a_matrix_covariance_coeffcient_1;
  a_matrix_(1, 3) = FLAGS_a_matrix_covariance_coeffcient_2;
  // initialize states to the states of the detected obstacle
  posteriori_state_(0) = belief_anchor_point_(0);
  posteriori_state_(1) = belief_anchor_point_(1);
  posteriori_state_(2) = belief_velocity_(0);
  posteriori_state_(3) = belief_velocity_(1);
  priori_state_ = posteriori_state_;

  q_matrix_.setIdentity();
  q_matrix_ *= FLAGS_q_matrix_coefficient_amplifier;

  r_matrix_.setIdentity();
  r_matrix_.topLeftCorner(2, 2) =
      FLAGS_r_matrix_amplifier *
      new_object->object->position_uncertainty.topLeftCorner(2, 2);
  r_matrix_.block<2, 2>(2, 2) =
      FLAGS_r_matrix_amplifier *
      new_object->object->velocity_uncertainty.topLeftCorner(2, 2);

  p_matrix_.setIdentity();
  p_matrix_.topLeftCorner(2, 2) =
      FLAGS_p_matrix_amplifier *
      new_object->object->position_uncertainty.topLeftCorner(2, 2);
  p_matrix_.block<2, 2>(2, 2) =
      FLAGS_p_matrix_amplifier *
      new_object->object->velocity_uncertainty.topLeftCorner(2, 2);
  c_matrix_.setIdentity();
}

void PbfKalmanMotionFusion::Predict(Eigen::Vector3d *anchor_point,
                                    Eigen::Vector3d *velocity,
                                    const double time_diff) {
  *anchor_point = belief_anchor_point_ + belief_velocity_ * time_diff;
  *velocity = belief_velocity_;
}

void PbfKalmanMotionFusion::UpdateWithObject(
    const std::shared_ptr<PbfSensorObject> new_object, const double time_diff) {
  ACHECK(new_object != nullptr && new_object->object != nullptr)
      << "update PbfKalmanMotionFusion with null sensor object";

  // predict and then correct
  a_matrix_.setIdentity();
  a_matrix_(0, 2) = time_diff;
  a_matrix_(1, 3) = time_diff;

  priori_state_ = a_matrix_ * posteriori_state_;
  priori_state_(2) += belief_acceleration_(0) * time_diff;
  priori_state_(3) += belief_acceleration_(1) * time_diff;

  p_matrix_ = ((a_matrix_ * p_matrix_) * a_matrix_.transpose()) + q_matrix_;
  p_matrix_.block<2, 2>(2, 0) = Eigen::Matrix2d::Zero();
  p_matrix_.block<2, 2>(0, 2) = Eigen::Matrix2d::Zero();

  Eigen::Vector3d measured_acceleration = Eigen::Vector3d::Zero();
  if (new_object->sensor_type == SensorType::VELODYNE_64) {
    belief_anchor_point_ = new_object->object->center;
    belief_velocity_ = new_object->object->velocity;
    if (GetLidarHistoryLength() >= 3) {
      int old_velocity_index = GetLidarHistoryIndex(3);
      Eigen::Vector3d old_velocity = history_velocity_[old_velocity_index];
      double old_timediff =
          GetHistoryTimediff(old_velocity_index, new_object->timestamp);
      measured_acceleration = (belief_velocity_ - old_velocity) / old_timediff;
    }
    if ((GetLidarHistoryLength() >= 3 && GetRadarHistoryLength() >= 3) ||
        history_velocity_.size() > 20) {
      history_velocity_.pop_front();
      history_time_diff_.pop_front();
      history_velocity_is_radar_.pop_front();
    }

    history_velocity_.push_back(belief_velocity_);
    history_time_diff_.push_back(new_object->timestamp);
    history_velocity_is_radar_.push_back(false);
  } else if (new_object->sensor_type == SensorType::RADAR) {
    belief_anchor_point_(0) = new_object->object->center(0);
    belief_anchor_point_(1) = new_object->object->center(1);
    belief_velocity_(0) = new_object->object->velocity(0);
    belief_velocity_(1) = new_object->object->velocity(1);
    if (GetRadarHistoryLength() >= 3) {
      int old_velocity_index = GetRadarHistoryIndex(3);
      Eigen::Vector3d old_velocity = history_velocity_[old_velocity_index];
      double old_timediff =
          GetHistoryTimediff(old_velocity_index, new_object->timestamp);
      measured_acceleration = (belief_velocity_ - old_velocity) / old_timediff;
    }
    if ((GetLidarHistoryLength() >= 3 && GetRadarHistoryLength() >= 3) ||
        history_velocity_.size() > 20) {
      history_velocity_.pop_front();
      history_time_diff_.pop_front();
      history_velocity_is_radar_.pop_front();
    }
    history_velocity_.push_back(belief_velocity_);
    history_time_diff_.push_back(new_object->timestamp);
    history_velocity_is_radar_.push_back(true);
  } else if (new_object->sensor_type == SensorType::CAMERA) {
    belief_anchor_point_(0) = new_object->object->center(0);
    belief_anchor_point_(1) = new_object->object->center(1);
    belief_velocity_(0) = new_object->object->velocity(0);
    belief_velocity_(1) = new_object->object->velocity(1);
    history_velocity_.push_back(belief_velocity_);
    history_time_diff_.push_back(new_object->timestamp);
    history_velocity_is_radar_.push_back(false);
  } else {
    AERROR << "unsupported sensor type for PbfKalmanMotionFusion: "
           << static_cast<int>(new_object->sensor_type);
    return;
  }

  Eigen::Vector4d measurement;
  measurement(0) = belief_anchor_point_(0);
  measurement(1) = belief_anchor_point_(1);
  measurement(2) = belief_velocity_(0);
  measurement(3) = belief_velocity_(1);

  // r_matrix_ = new_object.uncertainty_mat;
  r_matrix_.setIdentity();
  r_matrix_.topLeftCorner(2, 2) =
      new_object->object->position_uncertainty.topLeftCorner(2, 2);
  r_matrix_.block<2, 2>(2, 2) =
      new_object->object->velocity_uncertainty.topLeftCorner(2, 2);

  // Use lidar when there is no radar yet
  if (GetRadarHistoryLength() == 0 && GetLidarHistoryLength() > 1) {
    r_matrix_.setIdentity();
    r_matrix_ *= 0.01;
  }

  k_matrix_ =
      p_matrix_ * c_matrix_.transpose() *
      (c_matrix_ * p_matrix_ * c_matrix_.transpose() + r_matrix_).inverse();

  Eigen::Vector4d predict_measurement(priori_state_(0), priori_state_(1),
                                      priori_state_(2), priori_state_(3));

  posteriori_state_ =
      priori_state_ + k_matrix_ * (measurement - predict_measurement);
  p_matrix_ =
      (Eigen::Matrix4d::Identity() - k_matrix_ * c_matrix_) * p_matrix_ *
          (Eigen::Matrix4d::Identity() - k_matrix_ * c_matrix_).transpose() +
      k_matrix_ * r_matrix_ * k_matrix_.transpose();

  belief_anchor_point_(0) = posteriori_state_(0);
  belief_anchor_point_(1) = posteriori_state_(1);
  belief_velocity_(0) = posteriori_state_(2);
  belief_velocity_(1) = posteriori_state_(3);
  UpdateAcceleration(measured_acceleration);
  if (belief_velocity_.head(2).norm() < 0.05) {
    belief_velocity_ = Eigen::Vector3d(0, 0, 0);
  }
}

void PbfKalmanMotionFusion::UpdateWithoutObject(const double time_diff) {
  belief_anchor_point_ = belief_anchor_point_ + belief_velocity_ * time_diff;
}

void PbfKalmanMotionFusion::GetState(Eigen::Vector3d *anchor_point,
                                     Eigen::Vector3d *velocity) {
  *anchor_point = belief_anchor_point_;
  *velocity = belief_velocity_;
}

void PbfKalmanMotionFusion::SetState(const Eigen::Vector3d &anchor_point,
                                     const Eigen::Vector3d &velocity) {
  belief_anchor_point_ = anchor_point;
  belief_velocity_ = velocity;
}

int PbfKalmanMotionFusion::GetRadarHistoryLength() {
  int history_length = 0;
  for (size_t i = 0; i < history_velocity_is_radar_.size(); ++i) {
    if (history_velocity_is_radar_[i]) {
      history_length++;
    }
  }
  return history_length;
}

int PbfKalmanMotionFusion::GetLidarHistoryLength() {
  int history_length = history_velocity_is_radar_.size();
  history_length -= GetRadarHistoryLength();
  return history_length;
}

int PbfKalmanMotionFusion::GetLidarHistoryIndex(const int &history_seq) {
  int history_index = 0;
  int history_count = 0;
  for (size_t i = 1; i <= history_velocity_is_radar_.size(); ++i) {
    history_index = history_velocity_is_radar_.size() - i;
    if (!history_velocity_is_radar_[history_index]) {
      history_count++;
    }
    if (history_count == history_seq) {
      break;
    }
  }
  return history_index;
}

int PbfKalmanMotionFusion::GetRadarHistoryIndex(const int &history_seq) {
  int history_index = 0;
  int history_count = 0;
  for (size_t i = 1; i <= history_velocity_is_radar_.size(); ++i) {
    history_index = history_velocity_is_radar_.size() - i;
    if (history_velocity_is_radar_[history_index]) {
      history_count++;
    }
    if (history_count == history_seq) {
      break;
    }
  }
  return history_index;
}

double PbfKalmanMotionFusion::GetHistoryTimediff(
    const int &history_index, const double &current_timestamp) {
  double history_timestamp = history_time_diff_[history_index];
  double history_timediff = current_timestamp - history_timestamp;
  return history_timediff;
}

void PbfKalmanMotionFusion::UpdateAcceleration(
    const Eigen::VectorXd &measured_acceleration) {
  Eigen::Matrix2d mat_c = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d mat_q = Eigen::Matrix2d::Identity() * 0.5;
  Eigen::Matrix2d mat_k =
      p_matrix_.block<2, 2>(2, 2) * mat_c.transpose() *
      (mat_c * p_matrix_.block<2, 2>(2, 2) * mat_c.transpose() + mat_q)
          .inverse();
  Eigen::Vector2d acceleration_gain =
      mat_k *
      (measured_acceleration.head(2) - mat_c * belief_acceleration_.head(2));
  // breakdown
  float breakdown_threshold = 2;
  if (acceleration_gain.norm() > breakdown_threshold) {
    acceleration_gain.normalize();
    acceleration_gain *= breakdown_threshold;
  }
  belief_acceleration_(0) += acceleration_gain(0);
  belief_acceleration_(1) += acceleration_gain(1);
}

}  // namespace perception
}  // namespace apollo
