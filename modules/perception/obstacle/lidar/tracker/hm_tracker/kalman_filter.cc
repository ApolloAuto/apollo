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

#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/kalman_filter.h"

namespace apollo {
namespace perception {

bool KalmanFilter::s_use_adaptive_ = true;
double KalmanFilter::s_max_adaptive_score_ = 1.0;
double KalmanFilter::s_measurement_noise_ = 0.4;
double KalmanFilter::s_init_velocity_variance_ = 5;
double KalmanFilter::s_propagation_variance_xy_ = 10;
double KalmanFilter::s_propagation_variance_z_ = 10;

void KalmanFilter::SetUseAdaptive(const bool use_adaptive) {
  s_use_adaptive_ = use_adaptive;
}

void KalmanFilter::SetMaxAdaptiveScore(
  const double max_adaptive_score) {
  s_max_adaptive_score_ = max_adaptive_score;
}

void KalmanFilter::InitParams(const double measurement_noise,
  const double init_velocity_variance,
  const double propagation_variance_xy,
  const double propagation_variance_z) {
  s_measurement_noise_ = measurement_noise;
  s_init_velocity_variance_ = init_velocity_variance;
  s_propagation_variance_xy_ = propagation_variance_xy;
  s_propagation_variance_z_ = propagation_variance_z;
}

KalmanFilter::KalmanFilter() {
  name_ = "KalmanFilter";
  age_ = 0;
  covariance_velocity_ = s_init_velocity_variance_ *
    Eigen::Matrix3d::Identity();
  covariance_propagation_uncertainty_ = Eigen::Matrix3d::Zero();
  covariance_propagation_uncertainty_(0, 0) = s_propagation_variance_xy_;
  covariance_propagation_uncertainty_(1, 1) = s_propagation_variance_xy_;
  covariance_propagation_uncertainty_(2, 2) = s_propagation_variance_z_;

  update_quality_ = 1.0;
  belief_velocity_ = Eigen::Vector3d::Zero();
  belief_velocity_accelaration_ = Eigen::Vector3d::Zero();
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Initialize(const Eigen::Vector3f& anchor_point,
  const Eigen::Vector3f& velocity) {
  update_quality_ = 1.0;
  belief_anchor_point_ = anchor_point.cast<double>();
  belief_velocity_ = velocity.cast<double>();
  belief_velocity_accelaration_ = Eigen::Vector3d::Zero();
}

Eigen::VectorXf KalmanFilter::Predict(const double time_diff) {
  // Compute predict states
  Eigen::VectorXf predicted_state;
  predicted_state.resize(6);
  predicted_state(0) = belief_anchor_point_(0) +
    belief_velocity_(0) * time_diff;
  predicted_state(1) = belief_anchor_point_(1) +
    belief_velocity_(1) * time_diff;
  predicted_state(2) = belief_anchor_point_(2) +
    belief_velocity_(2) * time_diff;
  predicted_state(3) = belief_velocity_(0);
  predicted_state(4) = belief_velocity_(1);
  predicted_state(5) = belief_velocity_(2);

  // Compute predicted covariance
  Propagate(time_diff);

  return predicted_state;
}

void KalmanFilter::UpdateWithObject(const TrackedObjectPtr& new_object,
  const TrackedObjectPtr& old_object,
  const double time_diff) {
  // Compute update quality if needed
  // Use adaptive filtering after first measurement could help filter
  // converge more faster
  if (s_use_adaptive_ && (age_ > 0)) {
    ComputeUpdateQuality(new_object, old_object);
  } else {
    update_quality_ = 1.0;
  }

  // Compute measurements
  Eigen::Vector3f measured_anchor_point = new_object->anchor_point;
  Eigen::Vector3f measured_velocity = ComputeMeasuredVelocity(new_object,
    old_object, time_diff);

  // Update model
  UpdateModel(measured_anchor_point, measured_velocity, time_diff);
  age_ += 1;
}

void KalmanFilter::UpdateWithoutObject(const double time_diff) {
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
  Eigen::Vector3f* velocity_accelaration) {
  (*anchor_point) = belief_anchor_point_.cast<float>();
  (*velocity) = belief_velocity_.cast<float>();
  (*velocity_accelaration) = belief_velocity_accelaration_.cast<float>();
}

void KalmanFilter::Propagate(const double time_diff) {
  // Only propagate tracked motion
  if (age_ <= 0) {
    return;
  }
  covariance_velocity_ += covariance_propagation_uncertainty_ * (time_diff *
    time_diff);
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredVelocity(
  const TrackedObjectPtr& new_object,
  const TrackedObjectPtr& old_object,
  const double time_diff) {
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
  Eigen::Vector3f measured_velocity = SelectMeasuredVelocity(
    measured_candidates);
  return measured_velocity;
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredAnchorPointVelocity(
  const TrackedObjectPtr& new_object,
  const TrackedObjectPtr& old_object,
  const double time_diff) {
  // Compute 2D anchor point velocity measurment
  Eigen::Vector3f measured_anchor_point_velocity = new_object->anchor_point -
    old_object->anchor_point;
  measured_anchor_point_velocity /= time_diff;
  measured_anchor_point_velocity(2) = 0.0;
  return measured_anchor_point_velocity;
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredBboxCenterVelocity(
  const TrackedObjectPtr& new_object,
  const TrackedObjectPtr& old_object,
  const double time_diff) {
  // Compute 2D bbox center velocity measurment
  Eigen::Vector3d old_dir = old_object->direction.cast<double>();
  Eigen::Vector3d old_size = old_object->size.cast<double>();
  Eigen::Vector3d old_center = old_object->center.cast<double>();
  Eigen::Vector3d new_size = old_size;
  Eigen::Vector3d new_center = old_center;
  ComputeBboxSizeCenter<pcl_util::Point>(new_object->object_ptr->cloud,
    old_dir, new_size, new_center);
  Eigen::Vector3f measured_bbox_center_velocity_with_old_dir = (new_center -
    old_center).cast<float>();
  measured_bbox_center_velocity_with_old_dir /= time_diff;
  measured_bbox_center_velocity_with_old_dir(2) = 0.0;
  Eigen::Vector3f measured_bbox_center_velocity =
    measured_bbox_center_velocity_with_old_dir;
  Eigen::Vector3f project_dir = new_object->anchor_point -
      old_object->anchor_point;
  if (measured_bbox_center_velocity.dot(project_dir) <= 0) {
    measured_bbox_center_velocity = Eigen::Vector3f::Zero();
  }
  return measured_bbox_center_velocity;
}

Eigen::VectorXf KalmanFilter::ComputeMeasuredBboxCornerVelocity(
  const TrackedObjectPtr& new_object,
  const TrackedObjectPtr& old_object,
  const double time_diff) {
  // Compute 2D bbox corner velocity measurment
  Eigen::Vector3f project_dir = new_object->anchor_point -
    old_object->anchor_point;
  project_dir.normalize();
  Eigen::Vector3d old_dir = old_object->direction.cast<double>();
  Eigen::Vector3d old_size = old_object->size.cast<double>();
  Eigen::Vector3d old_center = old_object->center.cast<double>();
  Eigen::Vector3d new_size = old_size;
  Eigen::Vector3d new_center = old_center;
  ComputeBboxSizeCenter<pcl_util::Point>(new_object->object_ptr->cloud,
    old_dir, new_size, new_center);
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
    Eigen::Vector3f bbox_corner_velocity_on_project_dir = project_dir *
      bbox_corner_velocity_norm_on_project_dir;
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
  if (candidates.size() <= 0) {
    return Eigen::Vector3f::Zero();
  }
  Eigen::Vector3f measured_velocity = candidates[0];
  Eigen::Vector3f measured_velocity_gain = measured_velocity -
    belief_velocity_.cast<float>();
  for (size_t i = 1; i < candidates.size(); ++i) {
    Eigen::Vector3f candidate_velocity_gain = candidates[i] -
      belief_velocity_.cast<float>();
    if (candidate_velocity_gain.norm() < measured_velocity_gain.norm()) {
      measured_velocity = candidates[i];
      measured_velocity_gain = candidate_velocity_gain;
    }
  }
  return measured_velocity;
}

void KalmanFilter::UpdateModel(const Eigen::VectorXf measured_anchor_point,
  const Eigen::VectorXf measured_velocity,
  const double time_diff) {
  // Compute kalman gain
  Eigen::Matrix3d mat_c = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_q = s_measurement_noise_ * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_k = covariance_velocity_ * mat_c.transpose() *
    (mat_c * covariance_velocity_ * mat_c.transpose() + mat_q).inverse();

  // Compute posterior belief
  Eigen::Vector3d measured_anchor_point_d =
    measured_anchor_point.cast<double>();
  Eigen::Vector3d measured_velocity_d = measured_velocity.cast<double>();
  Eigen::Vector3d velocity_gain = mat_k * (measured_velocity_d - mat_c *
    belief_velocity_);

  // Breakdown
  float breakdown_threshold = ComputeBreakdownThreshold();
  if (velocity_gain.norm() > breakdown_threshold) {
    velocity_gain.normalize();
    velocity_gain *= breakdown_threshold;
  }

  belief_anchor_point_ = measured_anchor_point_d;
  belief_velocity_ += velocity_gain;
  belief_velocity_accelaration_ = velocity_gain / time_diff;

  // Adaptive
  if (s_use_adaptive_) {
    belief_velocity_ -= belief_velocity_accelaration_ * time_diff;
    belief_velocity_accelaration_ *= update_quality_;
    belief_velocity_ += belief_velocity_accelaration_ * time_diff;
  }

  // Compute posterior covariance
  covariance_velocity_ = (Eigen::Matrix3d::Identity() - mat_k * mat_c) *
    covariance_velocity_;
}

void KalmanFilter::ComputeUpdateQuality(const TrackedObjectPtr& new_object,
  const TrackedObjectPtr& old_object) {
  // Compute update quality for adaptive filtering
  // Strategy I: define update quality by use association score
  ComputeUpdateQualityByAssociationScore(new_object);
}

void KalmanFilter::ComputeUpdateQualityByAssociationScore(
  const TrackedObjectPtr& new_object) {
  // Compute update quality by using association score
  float association_score = new_object->association_score;
  if (s_max_adaptive_score_ == 0) {
    update_quality_ = 1;
    return;
  }
  double update_quality_ = 1 - (association_score / s_max_adaptive_score_);
  update_quality_ = update_quality_ > 1 ? 1 : update_quality_;
  update_quality_ = update_quality_ < 0 ? 0 : update_quality_;
  update_quality_ = update_quality_ * update_quality_;
}

float KalmanFilter::ComputeBreakdownThreshold() {
  // Compute breakdown threshold
  // Strategy I: define breakdown threshold as a linear
  float breakdown_threshold = 10 - age_ * 2;
  breakdown_threshold = breakdown_threshold > 0.5 ? breakdown_threshold : 0.5;
  return breakdown_threshold;
}

}  // namespace perception
}  // namespace apollo
