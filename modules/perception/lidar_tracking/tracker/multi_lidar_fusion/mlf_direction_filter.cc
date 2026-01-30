/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_direction_filter.h"

#include "Eigen/Dense"

#include "cyber/common/file.h"

#include "modules/perception/common/util.h"
#include "modules/perception/common/lidar/common/lidar_object_util.h"
#include "modules/perception/common/algorithm/geometry/basic.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

bool MlfDirectionFilter::Init(const MlfFilterInitOptions& options) {
  std::string config_file = "mlf_direction_filter.conf";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MlfDirectionFilterConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  init_direction_variance_ = config.init_direction_variance();
  variance_bound_ = config.variance_bound();
  max_angular_thresh_ = config.max_angular_thresh();
  converged_confidence_minimum_ = config.converged_confidence_minimum();
  angular_scale_ = config.angular_scale();
  angular_window_size_ = config.angular_window_size();
  predict_noise_per_sqrsec_ = config.predict_noise_per_sqrsec();
  angular_confidence_thresh_ = config.angular_confidence_thresh();

  return true;
}

void MlfDirectionFilter::UpdateWithObject(
    const MlfFilterOptions& options, const MlfTrackDataConstPtr& track_data,
    TrackedObjectPtr new_object) {
  Eigen::Vector3d ref_center(0, 0, 1.6);
  if (!track_data->IsForegroundTrack()) {
    new_object->direction_state_covariance =
        Eigen::Matrix3d::Identity() * init_direction_variance_;
    new_object->direction_state(0) = Vec2Angle(new_object->direction);
    return;
  }

  if (track_data->age_ == 0) {
    InitializeTrackState(new_object);
    return;
  }
  // kalman filter for direction
  TrackedObjectConstPtr latest_object = track_data->GetLatestObject().second;
  KalmanFilterUpdateDirection(track_data, latest_object, new_object);
  ComputeConvergenceConfidence(track_data, new_object);
  GetDirectionState(new_object);
}

void MlfDirectionFilter::UpdateWithoutObject(const MlfFilterOptions& options,
                                               double timestamp,
                                               MlfTrackDataPtr track_data) {}

void MlfDirectionFilter::InitializeTrackState(TrackedObjectPtr new_object) {
  new_object->direction_state_covariance =
     Eigen::Matrix3d::Identity() * init_direction_variance_;
  new_object->direction_state(0) = Vec2Angle(new_object->direction);
  new_object->direction_state(1) = 0.0;
  new_object->direction_state(2) = 0.0;
  new_object->output_direction = new_object->direction;
  new_object->output_angular = Eigen::Vector3d::Zero();
}

void MlfDirectionFilter::KalmanFilterUpdateDirection(
    const MlfTrackDataConstPtr& track_data,
    const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object) {
  if (track_data->age_ > 0) {
    if (new_object->direction.dot(latest_object->output_direction) < 0) {
      new_object->direction *= -1;
    }

    double time_diff = new_object->timestamp - latest_object->timestamp;
    double angular = 0.0;
    double direction_measurement_variance = variance_bound_;
    double angular_measurement_variance = variance_bound_;
    ComputeVarianceAndAngular(track_data, new_object,
      &direction_measurement_variance,
      &angular_measurement_variance, &angular);
    // initialize measurement
    Eigen::Vector2d direction_measurement;
    direction_measurement(0) = Vec2Angle(new_object->direction);
    direction_measurement(1) = angular;
    // 1. predict process
    const auto& last_direction_state = latest_object->direction_state;
    const auto& last_direction_state_covariance =
      latest_object->direction_state_covariance;

    Eigen::Matrix3d transition = Eigen::Matrix3d::Identity();
    transition(0, 1) = time_diff;
    transition(0, 2) = 0.5 * time_diff * time_diff;
    transition(1, 2) = time_diff;
    double direction_predict_noise =
      variance_bound_ * (1e-1 * time_diff * time_diff);

    auto& direction_state = new_object->direction_state;
    auto& direction_state_covariance =
      new_object->direction_state_covariance;
    Eigen::Matrix3d predict_noise = Eigen::Matrix3d::Identity() *
      predict_noise_per_sqrsec_ * time_diff;
    predict_noise(0, 0) = direction_predict_noise;

    direction_state = transition * last_direction_state;
    direction_state(0) = LegalizeAngle(direction_state(0));
    direction_state_covariance =
      transition * last_direction_state_covariance *
      transition.transpose() + predict_noise;

    // 2. measurement update
    Eigen::Matrix<double, 2, 3> observation_transform;
    observation_transform.block<2, 2>(0, 0).setIdentity();
    observation_transform.block<2, 1>(0, 2).setZero();
    Eigen::Matrix2d measurement_covariance = Eigen::Matrix2d::Identity();
    measurement_covariance(0, 0) = direction_measurement_variance;
    measurement_covariance(1, 1) = angular_measurement_variance;
    Eigen::Matrix<double, 3, 2> kalman_gain_matrix =
      direction_state_covariance * observation_transform.transpose() *
      (observation_transform * direction_state_covariance *
      observation_transform.transpose() + measurement_covariance).inverse();
    Eigen::Vector2d observation_transform_state =
        observation_transform * direction_state;
    Eigen::Vector2d measurement_residual;
    measurement_residual(0) = LegalizeAngle(direction_measurement(0) -
       observation_transform_state(0));
    measurement_residual(1) = direction_measurement(1) -
       observation_transform_state(1);

    // compute kalman gain
    Eigen::Vector3d direction_state_gain =
        kalman_gain_matrix * measurement_residual;
    direction_state(0) =
        LegalizeAngle(direction_state(0) + direction_state_gain(0));
    direction_state(1) = direction_state(1) + direction_state_gain(1);
    direction_state(2) = (direction_state(2) + direction_state_gain(2));

    direction_state_covariance =
     (Eigen::Matrix3d::Identity() - kalman_gain_matrix *
      observation_transform) * direction_state_covariance;
    // sync
    double filter_theta = Degree2Radian(new_object->direction_state(0));
    double filter_angular = Degree2Radian(new_object->direction_state(1));
    new_object->output_direction(0) = std::cos(filter_theta);
    new_object->output_direction(1) = std::sin(filter_theta);
    new_object->output_direction(2) = 0.0;
    new_object->output_angular(0) = std::cos(filter_angular);
    new_object->output_angular(1) = std::sin(filter_angular);
    new_object->output_angular(2) = 0.0;
  }
}

void MlfDirectionFilter::ComputeVarianceAndAngular(
  const MlfTrackDataConstPtr& track_data, TrackedObjectPtr new_object,
  double* direction_measurement_variance,
  double* angular_measurement_variance,
  double* angular) {
  // get history objects
  std::vector<TrackedObjectConstPtr> history_objects;
  track_data->GetObjectsInInterval(
    new_object->timestamp - 2.0, &history_objects);
  if (history_objects.size() == 0) {
    return;
  }
  history_objects.insert(history_objects.begin(), new_object);
  size_t object_num = history_objects.size();
  size_t history_start_interval = 5;
  size_t history_end_interval = 15;
  size_t angular_window_end = std::min(angular_window_size_, object_num - 1);
  // compute angular in a small window
  double window_time_diff = history_objects[0]->timestamp -
      history_objects[angular_window_end]->timestamp;
  *angular = angular_scale_ * VecDiffAngle(history_objects[0]->direction,
      history_objects[angular_window_end]->direction) / window_time_diff;

  // compute angular in latest two objects
  double time_diff = history_objects[0]->timestamp -
    history_objects[1]->timestamp;
  double latest_adjacent_frame_angular =
    std::fabs(VecDiffAngle(history_objects[0]->direction,
    history_objects[1]->direction)) / time_diff;

  // compute measurement variance
  double direction_confidence = 0.0;
  ComputeMeasurementConfidence(new_object, &direction_confidence);
  *direction_measurement_variance =
    std::pow((1.0 - direction_confidence) * 45.0, 2.0) + 1.0;

  // supress sudden unnormal direction
  if (latest_adjacent_frame_angular > 3.0 * max_angular_thresh_) {
    *direction_measurement_variance = variance_bound_ * variance_bound_;
    *angular_measurement_variance = variance_bound_ * variance_bound_;;
    *angular = 0.0;
    return;
  }

  // compute angular measurement variance and angular confidence
  std::vector<size_t> angular_vec;
  double agreed_dir_num = 0.0;
  for (size_t i = 0; i < angular_window_end; ++i) {
    double cur_time_diff = history_objects[i]->timestamp -
      history_objects[i + 1]->timestamp;
    double cur_angular = VecDiffAngle(history_objects[i]->direction,
      history_objects[i + 1]->direction) / cur_time_diff;
    *angular_measurement_variance +=
       (*angular - cur_angular) * (*angular - cur_angular);

    size_t start = std::min(object_num - 1, i + history_start_interval);
    size_t end = std::min(object_num - 1, i + history_end_interval);
    for (size_t j = start; j < end; ++j) {
      if (j == i) {
        continue;
      }
      double angle_change = algorithm::CalculateTheta2DXY<double>(
        history_objects[i]->direction.head(2),
        history_objects[j]->direction.head(2));
      if (angle_change >= 0.0) {
        angular_vec.push_back(1);
        agreed_dir_num += 1.0;
      } else {
        angular_vec.push_back(-1);
      }
    }
  }
  // use 0.1 to scale angular_measurement_variance
  *angular_measurement_variance /= (angular_window_end * 10.0);

  // use hstory direction trend to compute angular confidence,
  // when angular confidence is small, there should no angular
  agreed_dir_num = agreed_dir_num > 0.5 * angular_vec.size() ?
    agreed_dir_num : angular_vec.size() - agreed_dir_num;
  double agreed_dir_ratio = (agreed_dir_num + 1.0) / (angular_vec.size() + 1.0);
  double angular_confidence = agreed_dir_ratio * agreed_dir_ratio;
  if (angular_confidence < angular_confidence_thresh_) {
    *angular = 0.0;
    *angular_measurement_variance = 1.0;
  }
}

void MlfDirectionFilter::ComputeConvergenceConfidence(
  const MlfTrackDataConstPtr& track_data,
  TrackedObjectPtr new_object) {
  if (track_data->age_ <= 1) {
    new_object->direction_convergence_confidence = 0.0;
    new_object->direction_converged = false;
  }

  double direction_variance = new_object->direction_state_covariance(0, 0);
  new_object->direction_convergence_confidence =
    exp(-0.25 * direction_variance);
  new_object->direction_converged =
    new_object->direction_convergence_confidence >
    converged_confidence_minimum_;
}

void MlfDirectionFilter::ComputeMeasurementConfidence(
  const TrackedObjectPtr& new_object, double* confidence) {
  const base::ObjectPtr& object = new_object->object_ptr;
  Eigen::Vector3f polygon_direction = Eigen::Vector3f::Zero();
  Eigen::Vector3d ref_center = Eigen::Vector3d::Zero();
  ComputePolygonDirection(ref_center, object, &polygon_direction);
  polygon_direction =
    (new_object->sensor_to_local_pose.rotation()).cast<float>()
    * polygon_direction;
  double direction_diff = std::fabs(
    algorithm::CalculateTheta2DXY<double>(new_object->direction.head(2),
    polygon_direction.head(2).cast<double>()));
  direction_diff = direction_diff > M_PI / 2.0 ?
      M_PI - direction_diff : direction_diff;
  direction_diff = direction_diff > M_PI / 4.0 ?
      M_PI / 2.0 - direction_diff : direction_diff;
  *confidence = 1.0 - direction_diff / (0.25 * M_PI);
}

double MlfDirectionFilter::VecDiffAngle(const Eigen::Vector3d& vec0,
                                          const Eigen::Vector3d& vec1) {
  double angle =
      Degree2Angle(std::atan2(vec0(1), vec0(0)) - std::atan2(vec1(1), vec1(0)));
  return LegalizeAngle(angle);
}

double MlfDirectionFilter::Vec2Angle(const Eigen::Vector3d& vec) {
  double degree = std::atan2(vec(1), vec(0));
  return Degree2Angle(degree);
}

double MlfDirectionFilter::LegalizeAngle(double angle) {
  angle = std::fmod(angle, 360.0);
  if (angle > 180.0) {
    return angle - 360.0;
  } else if (angle < -180.0) {
    return angle + 360.0;
  } else {
    return angle;
  }
}

void MlfDirectionFilter::GetDirectionState(TrackedObjectPtr& new_object) {
  new_object->direction = new_object->output_direction;
}

PERCEPTION_REGISTER_MLFFILTER(MlfDirectionFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
