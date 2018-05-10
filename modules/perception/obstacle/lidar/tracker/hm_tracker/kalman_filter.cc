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

#include "modules/perception/obstacle/lidar/tracker/hm_tracker/kalman_filter.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/perception/common/geometry_util.h"

namespace apollo {
namespace perception {

bool KalmanFilter::s_use_adaptive_ = true;
double KalmanFilter::s_association_score_maximum_ = 1.0;
Eigen::Matrix3d KalmanFilter::s_propagation_noise_ =
    10 * Eigen::Matrix3d::Identity();
double KalmanFilter::s_measurement_noise_ = 0.4;
double KalmanFilter::s_initial_velocity_noise_ = 5;
double KalmanFilter::s_breakdown_threshold_maximum_ = 10;
size_t KalmanFilter::s_measurement_cached_history_size_minimum_ = 3;
size_t KalmanFilter::s_measurement_cached_history_size_maximum_ = 6;

void KalmanFilter::SetUseAdaptive(const bool& use_adaptive) {
  s_use_adaptive_ = use_adaptive;
  AINFO << "use adaptive of KalmanFilter is " << s_use_adaptive_;
}

bool KalmanFilter::SetAssociationScoreMaximum(
    const double& association_score_maximum) {
  if (association_score_maximum > 0) {
    s_association_score_maximum_ = association_score_maximum;
    AINFO << "association score maximum of KalmanFilter is "
          << s_association_score_maximum_;
    return true;
  }
  AERROR << "invalid association score maximum of KalmanFilter!";
  return false;
}

bool KalmanFilter::SetBreakdownThresholdMaximum(
    const double& breakdown_threshold_maximum) {
  if (breakdown_threshold_maximum > 0) {
    s_breakdown_threshold_maximum_ = breakdown_threshold_maximum;
    AINFO << "breakdown threshold maximum of KalmanFilter is "
          << s_breakdown_threshold_maximum_;
    return true;
  }
  AERROR << "invalid breakdown threshold maximum of KalmanFilter!";
  return false;
}

bool KalmanFilter::InitParams(const double& measurement_noise,
                              const double& initial_velocity_noise,
                              const double& xy_propagation_noise,
                              const double& z_propagation_noise) {
  if (measurement_noise < 0) {
    AERROR << "invalid measurement noise of KalmanFilter!";
    return false;
  }
  if (initial_velocity_noise < 0) {
    AERROR << "invalid intial velocity noise of KalmanFilter!";
    return false;
  }
  if (xy_propagation_noise < 0) {
    AERROR << "invalid xy propagation noise of KalmanFilter!";
    return false;
  }
  if (z_propagation_noise < 0) {
    AERROR << "invalid z propagation noise of KalmanFilter!";
    return false;
  }
  s_measurement_noise_ = measurement_noise;
  s_initial_velocity_noise_ = initial_velocity_noise;
  s_propagation_noise_(0, 0) = xy_propagation_noise;
  s_propagation_noise_(1, 1) = xy_propagation_noise;
  s_propagation_noise_(2, 2) = z_propagation_noise;
  AINFO << "measurment noise of KalmanFilter is " << s_measurement_noise_;
  AINFO << "initial velocity noise of KalmanFilter is "
        << s_initial_velocity_noise_;
  AINFO << "propagation noise of KalmanFilter is\n" << s_propagation_noise_;
  return true;
}

KalmanFilter::KalmanFilter() {
  name_ = "KalmanFilter";
  age_ = 0;
  measurement_cached_history_size_ = s_measurement_cached_history_size_minimum_;
  s_measurement_cached_history_size_maximum_ =
      s_measurement_cached_history_size_minimum_;
  velocity_covariance_ =
      s_initial_velocity_noise_ * Eigen::Matrix3d::Identity();
  // states
  update_quality_ = 1.0;
  breakdown_threshold_ = s_breakdown_threshold_maximum_;
  belief_velocity_ = Eigen::Vector3d::Zero();
  belief_acceleration_gain_ = Eigen::Vector3d::Zero();
  belief_acceleration_ = Eigen::Vector3d::Zero();
}

void KalmanFilter::Initialize(const Eigen::Vector3f& anchor_point,
                              const Eigen::Vector3f& velocity) {
  update_quality_ = 1.0;
  breakdown_threshold_ = s_breakdown_threshold_maximum_;
  belief_anchor_point_ = anchor_point.cast<double>();
  belief_velocity_ = velocity.cast<double>();
  belief_acceleration_gain_ = Eigen::Vector3d::Zero();
  belief_acceleration_ = Eigen::Vector3d::Zero();
}

Eigen::VectorXf KalmanFilter::Predict(const double& time_diff) {
  // Compute predict states
  Eigen::VectorXf predicted_state;
  predicted_state.resize(6);
  predicted_state(0) =
      belief_anchor_point_(0) + belief_velocity_(0) * time_diff;
  predicted_state(1) =
      belief_anchor_point_(1) + belief_velocity_(1) * time_diff;
  predicted_state(2) =
      belief_anchor_point_(2) + belief_velocity_(2) * time_diff;
  predicted_state(3) = belief_velocity_(0);
  predicted_state(4) = belief_velocity_(1);
  predicted_state(5) = belief_velocity_(2);
  // Compute predicted covariance
  Propagate(time_diff);
  return predicted_state;
}

void KalmanFilter::UpdateWithObject(
    const std::shared_ptr<TrackedObject>& new_object,
    const std::shared_ptr<TrackedObject>& old_object, const double& time_diff) {
  if (time_diff <= DBL_EPSILON) {
    AWARN << "Time diff is too limited to updating KalmanFilter!";
    return;
  }
  // A. Compute update quality if needed
  if (s_use_adaptive_) {
    ComputeUpdateQuality(new_object, old_object);
  } else {
    update_quality_ = 1.0;
  }

  // B. Compute measurements
  Eigen::Vector3f measured_anchor_point = new_object->anchor_point;
  Eigen::Vector3f measured_velocity =
      ComputeMeasuredVelocity(new_object, old_object, time_diff);

  // debug: compute measured acceleration
  Eigen::Vector3f measured_acceleration =
      ComputeMeasuredAcceleration(measured_velocity, time_diff);

  // C. Update model
  UpdateVelocity(measured_anchor_point, measured_velocity, time_diff);
  UpdateAcceleration(measured_acceleration);

  // Cache measurement history
  if (history_measured_velocity_.size() >= measurement_cached_history_size_) {
    history_measured_velocity_.pop_front();
    history_time_diff_.pop_front();
  }
  history_measured_velocity_.push_back(measured_velocity);
  history_time_diff_.push_back(time_diff);

  EvaluateOnlineCovariance();

  age_ += 1;
}

void KalmanFilter::UpdateWithoutObject(const double& time_diff) {
  // Only update belief anchor point
  belief_anchor_point_ += belief_velocity_ * time_diff;
  age_ += 1;
}

void KalmanFilter::GetState(Eigen::Vector3f* anchor_point,
                            Eigen::Vector3f* velocity) {
  (*anchor_point) = belief_anchor_point_.cast<float>();
  (*velocity) = belief_velocity_.cast<float>();
}

void KalmanFilter::GetState(Eigen::Vector3f* anchor_point,
                            Eigen::Vector3f* velocity,
                            Eigen::Vector3f* acceleration) {
  (*anchor_point) = belief_anchor_point_.cast<float>();
  (*velocity) = belief_velocity_.cast<float>();
  (*acceleration) = belief_acceleration_.cast<float>();
}

void KalmanFilter::GetAccelerationGain(Eigen::Vector3f* acceleration_gain) {
  (*acceleration_gain) = belief_acceleration_gain_.cast<float>();
}

void KalmanFilter::Propagate(const double& time_diff) {
  // Only propagate tracked motion
  if (age_ <= 0) {
    return;
  }
  velocity_covariance_ += s_propagation_noise_ * time_diff * time_diff;
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredVelocity(
    const std::shared_ptr<TrackedObject>& new_object,
    const std::shared_ptr<TrackedObject>& old_object, const double& time_diff) {
  // Compute 2D velocity measurment for filtering
  // Obtain robust measurment via observation redundency

  // Observation I: anchor point velocity measurment
  Eigen::Vector3f measured_anchor_point_velocity =
      ComputeMeasuredAnchorPointVelocity(new_object, old_object, time_diff);
  // Observation II: bbox center velocity measurment
  Eigen::Vector3f measured_bbox_center_velocity =
      ComputeMeasuredBboxCenterVelocity(new_object, old_object, time_diff);
  // Observation III: bbox corner velocity measurment
  Eigen::Vector3f measured_bbox_corner_velocity =
      ComputeMeasuredBboxCornerVelocity(new_object, old_object, time_diff);

  std::vector<Eigen::Vector3f> measured_candidates;
  measured_candidates.push_back(measured_anchor_point_velocity);
  measured_candidates.push_back(measured_bbox_center_velocity);
  measured_candidates.push_back(measured_bbox_corner_velocity);
  Eigen::Vector3f measured_velocity =
      SelectMeasuredVelocity(measured_candidates);
  return measured_velocity;
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredAnchorPointVelocity(
    const std::shared_ptr<TrackedObject>& new_object,
    const std::shared_ptr<TrackedObject>& old_object, const double& time_diff) {
  // Compute 2D anchor point velocity measurment
  Eigen::Vector3f measured_anchor_point_velocity =
      new_object->anchor_point - old_object->anchor_point;
  measured_anchor_point_velocity /= time_diff;
  measured_anchor_point_velocity(2) = 0.0;
  return measured_anchor_point_velocity;
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredBboxCenterVelocity(
    const std::shared_ptr<TrackedObject>& new_object,
    const std::shared_ptr<TrackedObject>& old_object, const double& time_diff) {
  // Compute 2D bbox center velocity measurment
  Eigen::Vector3d old_dir = old_object->direction.cast<double>();
  Eigen::Vector3d old_size = old_object->size.cast<double>();
  Eigen::Vector3d old_center = old_object->center.cast<double>();
  Eigen::Vector3d new_size = old_size;
  Eigen::Vector3d new_center = old_center;
  ComputeBboxSizeCenter<pcl_util::Point>(new_object->object_ptr->cloud, old_dir,
                                         &new_size, &new_center);
  Eigen::Vector3f measured_bbox_center_velocity_with_old_dir =
      (new_center - old_center).cast<float>();
  measured_bbox_center_velocity_with_old_dir /= time_diff;
  measured_bbox_center_velocity_with_old_dir(2) = 0.0;
  Eigen::Vector3f measured_bbox_center_velocity =
      measured_bbox_center_velocity_with_old_dir;
  Eigen::Vector3f project_dir =
      new_object->anchor_point - old_object->anchor_point;
  if (measured_bbox_center_velocity.dot(project_dir) <= 0) {
    measured_bbox_center_velocity = Eigen::Vector3f::Zero();
  }
  return measured_bbox_center_velocity;
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredBboxCornerVelocity(
    const std::shared_ptr<TrackedObject>& new_object,
    const std::shared_ptr<TrackedObject>& old_object, const double& time_diff) {
  // Compute 2D bbox corner velocity measurment
  Eigen::Vector3f project_dir =
      new_object->anchor_point - old_object->anchor_point;
  project_dir.normalize();
  Eigen::Vector3d old_dir = old_object->direction.cast<double>();
  Eigen::Vector3d old_size = old_object->size.cast<double>();
  Eigen::Vector3d old_center = old_object->center.cast<double>();
  Eigen::Vector3d new_size = old_size;
  Eigen::Vector3d new_center = old_center;
  ComputeBboxSizeCenter<pcl_util::Point>(new_object->object_ptr->cloud, old_dir,
                                         &new_size, &new_center);
  Eigen::Vector3d ortho_old_dir(-old_dir(1), old_dir(0), 0.0);

  Eigen::Vector3d old_bbox_corner_list[4];
  Eigen::Vector3d new_bbox_corner_list[4];
  Eigen::Vector3d old_bbox_corner = old_center + old_dir * old_size(0) * 0.5 +
                                    ortho_old_dir * old_size(1) * 0.5;
  Eigen::Vector3d new_bbox_corner = new_center + old_dir * new_size(0) * 0.5 +
                                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[0] = old_bbox_corner;
  new_bbox_corner_list[0] = new_bbox_corner;
  old_bbox_corner = old_center - old_dir * old_size(0) * 0.5 +
                    ortho_old_dir * old_size(1) * 0.5;
  new_bbox_corner = new_center - old_dir * new_size(0) * 0.5 +
                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[1] = old_bbox_corner;
  new_bbox_corner_list[1] = new_bbox_corner;
  old_bbox_corner = old_center + old_dir * old_size(0) * 0.5 -
                    ortho_old_dir * old_size(1) * 0.5;
  new_bbox_corner = new_center + old_dir * new_size(0) * 0.5 -
                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[2] = old_bbox_corner;
  new_bbox_corner_list[2] = new_bbox_corner;
  old_bbox_corner = old_center - old_dir * old_size(0) * 0.5 -
                    ortho_old_dir * old_size(1) * 0.5;
  new_bbox_corner = new_center - old_dir * new_size(0) * 0.5 -
                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[3] = old_bbox_corner;
  new_bbox_corner_list[3] = new_bbox_corner;

  Eigen::Vector3f min_bbox_corner_velocity_on_project_dir =
      Eigen::Vector3f(100, 100, 0);
  float min_bbox_corner_velocity_on_project_dir_gain_norm =
      min_bbox_corner_velocity_on_project_dir.norm();
  for (size_t i = 0; i < 4; ++i) {
    old_bbox_corner = old_bbox_corner_list[i];
    new_bbox_corner = new_bbox_corner_list[i];
    Eigen::Vector3f bbox_corner_velocity =
        ((new_bbox_corner - old_bbox_corner) / time_diff).cast<float>();
    float bbox_corner_velocity_project_dir_inner_product =
        bbox_corner_velocity(0) * project_dir(0) +
        bbox_corner_velocity(1) * project_dir(1);
    float bbox_corner_velocity_project_dir_angle_cos =
        bbox_corner_velocity_project_dir_inner_product /
        (bbox_corner_velocity.head(2).norm() * project_dir.head(2).norm());
    float bbox_corner_velocity_norm_on_project_dir =
        bbox_corner_velocity.head(2).norm() *
        bbox_corner_velocity_project_dir_angle_cos;
    Eigen::Vector3f bbox_corner_velocity_on_project_dir =
        project_dir * bbox_corner_velocity_norm_on_project_dir;
    bbox_corner_velocity_on_project_dir(2) = 0.0;
    if (bbox_corner_velocity_on_project_dir(0) * project_dir(0) <= 0) {
      bbox_corner_velocity_on_project_dir = Eigen::Vector3f::Zero();
    }
    Eigen::Vector3f bbox_corner_velocity_on_project_dir_gain =
        bbox_corner_velocity_on_project_dir - belief_velocity_.cast<float>();
    if (bbox_corner_velocity_on_project_dir_gain.norm() <
        min_bbox_corner_velocity_on_project_dir_gain_norm) {
      min_bbox_corner_velocity_on_project_dir =
          bbox_corner_velocity_on_project_dir;
      min_bbox_corner_velocity_on_project_dir_gain_norm =
          bbox_corner_velocity_on_project_dir_gain.norm();
    }
  }
  return min_bbox_corner_velocity_on_project_dir;
}

Eigen::Vector3f KalmanFilter::SelectMeasuredVelocity(
    const std::vector<Eigen::Vector3f>& candidates) {
  // Select measured velocity among candidates
  // Strategy I: accoridng motion consistency
  return SelectMeasuredVelocityAccordingMotionConsistency(candidates);
}

Eigen::Vector3f KalmanFilter::SelectMeasuredVelocityAccordingMotionConsistency(
    const std::vector<Eigen::Vector3f>& candidates) {
  // Select measured velocity among candidates according motion consistency
  if (candidates.size() <= 0) return Eigen::Vector3f::Zero();
  Eigen::Vector3f measured_velocity = candidates[0];
  Eigen::Vector3f measured_velocity_gain =
      measured_velocity - belief_velocity_.cast<float>();
  for (size_t i = 1; i < candidates.size(); ++i) {
    Eigen::Vector3f candidate_velocity_gain =
        candidates[i] - belief_velocity_.cast<float>();
    if (candidate_velocity_gain.norm() < measured_velocity_gain.norm()) {
      measured_velocity = candidates[i];
      measured_velocity_gain = candidate_velocity_gain;
    }
  }
  return measured_velocity;
}

void KalmanFilter::UpdateVelocity(const Eigen::VectorXf& measured_anchor_point,
                                  const Eigen::VectorXf& measured_velocity,
                                  const double& time_diff) {
  // Compute kalman gain
  Eigen::Matrix3d mat_c = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_q = s_measurement_noise_ * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_k =
      velocity_covariance_ * mat_c.transpose() *
      (mat_c * velocity_covariance_ * mat_c.transpose() + mat_q).inverse();

  // Compute posterior belief
  Eigen::Vector3d measured_anchor_point_d =
      measured_anchor_point.cast<double>();
  Eigen::Vector3d measured_velocity_d = measured_velocity.cast<double>();
  Eigen::Vector3d priori_velocity =
      belief_velocity_ + belief_acceleration_gain_ * time_diff;
  Eigen::Vector3d velocity_gain =
      mat_k * (measured_velocity_d - mat_c * priori_velocity);

  // Breakdown
  ComputeBreakdownThreshold();
  if (velocity_gain.norm() > breakdown_threshold_) {
    velocity_gain.normalize();
    velocity_gain *= breakdown_threshold_;
  }

  belief_anchor_point_ = measured_anchor_point_d;
  belief_velocity_ = priori_velocity + velocity_gain;
  belief_acceleration_gain_ = velocity_gain / time_diff;

  // Adaptive
  if (s_use_adaptive_) {
    belief_velocity_ -= belief_acceleration_gain_ * time_diff;
    belief_acceleration_gain_ *= update_quality_;
    belief_velocity_ += belief_acceleration_gain_ * time_diff;
  }

  // Compute posterior covariance
  velocity_covariance_ =
      (Eigen::Matrix3d::Identity() - mat_k * mat_c) * velocity_covariance_;
}

void KalmanFilter::ComputeUpdateQuality(
    const std::shared_ptr<TrackedObject>& new_object,
    const std::shared_ptr<TrackedObject>& old_object) {
  // Compute update quality for adaptive filtering
  // Strategy A: according to association score
  float update_quality_according_association_score =
      ComputeUpdateQualityAccordingAssociationScore(new_object);
  // Strategy B: according to point number change
  float update_quality_according_point_num_change =
      ComputeUpdateQualityAccordingPointNumChange(new_object, old_object);
  // Pick a smaller one to control possible filter distraction of noises
  update_quality_ = update_quality_according_association_score;
  if (update_quality_according_association_score >
      update_quality_according_point_num_change) {
    update_quality_ = update_quality_according_point_num_change;
  }
}

float KalmanFilter::ComputeUpdateQualityAccordingAssociationScore(
    const std::shared_ptr<TrackedObject>& new_object) {
  // Compute update quality according association score
  float association_score = new_object->association_score;
  float update_quality = 1;
  if (s_association_score_maximum_ == 0) {
    return update_quality;
  }
  update_quality = 1 - (association_score / s_association_score_maximum_);
  update_quality = update_quality > 1 ? 1 : update_quality;
  update_quality = update_quality < 0 ? 0 : update_quality;
  update_quality = update_quality * update_quality;
  return update_quality;
}

float KalmanFilter::ComputeUpdateQualityAccordingPointNumChange(
    const std::shared_ptr<TrackedObject>& new_object,
    const std::shared_ptr<TrackedObject>& old_object) {
  // Compute updaet quality according point number change
  int new_pt_num = new_object->object_ptr->cloud->size();
  int old_pt_num = old_object->object_ptr->cloud->size();
  if (new_pt_num <= 0 || old_pt_num <= 0) {
    return 0;
  }
  float update_quality =
      1 - std::abs(new_pt_num - old_pt_num) / std::max(new_pt_num, old_pt_num);
  return update_quality;
}

void KalmanFilter::ComputeBreakdownThreshold() {
  // Set breakdown threshold as 2 times of velocity covariance, and smooth
  // it somehow.
  breakdown_threshold_ += velocity_covariance_(0, 0) * 2;
  breakdown_threshold_ /= 2;
  if (breakdown_threshold_ > s_breakdown_threshold_maximum_) {
    breakdown_threshold_ = s_breakdown_threshold_maximum_;
  }
}

Eigen::Vector3f KalmanFilter::ComputeMeasuredAcceleration(
    const Eigen::Vector3f& measured_velocity, const double& time_diff) {
  if (history_measured_velocity_.size() < 3) {
    return Eigen::Vector3f::Zero();
  }
  int history_index = history_measured_velocity_.size() - 3;
  Eigen::Vector3f history_measurement =
      history_measured_velocity_[history_index];
  double accumulated_time_diff = time_diff;
  for (size_t i = history_index + 1; i < history_measured_velocity_.size();
       ++i) {
    accumulated_time_diff += history_time_diff_[i];
  }
  Eigen::Vector3f measured_acceleration =
      measured_velocity - history_measurement;
  measured_acceleration /= accumulated_time_diff;
  return measured_acceleration;
}

void KalmanFilter::EvaluateOnlineCovariance() {
  Eigen::Matrix3d online_covariance = Eigen::Matrix3d::Zero();
  int evaluate_window = history_measured_velocity_.size() >
                                s_measurement_cached_history_size_maximum_
                            ? history_measured_velocity_.size()
                            : s_measurement_cached_history_size_maximum_;
  for (int i = 0; i < evaluate_window; ++i) {
    int history_index = history_measured_velocity_.size() - i - 1;
    Eigen::Vector3d velocity_resisual = Eigen::Vector3d(5, 5, 0);
    if (history_index >= 0) {
      velocity_resisual =
          history_measured_velocity_[history_index].cast<double>() -
          belief_velocity_;
    }
    online_covariance(0, 0) += velocity_resisual(0) * velocity_resisual(0);
    online_covariance(0, 1) += velocity_resisual(0) * velocity_resisual(1);
    online_covariance(1, 0) += velocity_resisual(1) * velocity_resisual(0);
    online_covariance(1, 1) += velocity_resisual(1) * velocity_resisual(1);
  }
  online_velocity_covariance_ = online_covariance / evaluate_window;
}

void KalmanFilter::GetOnlineCovariance(Eigen::Matrix3f* online_covariance) {
  *online_covariance = online_velocity_covariance_.cast<float>();
}

void KalmanFilter::UpdateAcceleration(
    const Eigen::VectorXf& measured_acceleration) {
  // Compute kalman gain
  Eigen::Matrix3d mat_c = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_q =
      s_measurement_noise_ * Eigen::Matrix3d::Identity() * 3;
  Eigen::Matrix3d mat_k =
      velocity_covariance_ * mat_c.transpose() *
      (mat_c * velocity_covariance_ * mat_c.transpose() + mat_q).inverse();
  // Compute posterior belief
  Eigen::Vector3d measured_acceleration_d =
      measured_acceleration.cast<double>();
  Eigen::Vector3d acceleration_gain =
      mat_k * (measured_acceleration_d - mat_c * belief_acceleration_);
  // Adaptive
  acceleration_gain *= update_quality_;
  // Breakdonw
  float breakdown_threshold = 2;
  if (acceleration_gain.norm() > breakdown_threshold) {
    acceleration_gain.normalize();
    acceleration_gain *= breakdown_threshold;
  }
  // simple add
  belief_acceleration_ += acceleration_gain;
}

}  // namespace perception
}  // namespace apollo
