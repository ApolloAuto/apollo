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

#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_motion_filter.h"

#include <algorithm>
#include <vector>

#include "cyber/common/file.h"
#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

bool MlfMotionFilter::Init(const MlfFilterInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "mlf_motion_filter.conf");
  MlfMotionFilterConfig config;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config));
  use_adaptive_ = config.use_adaptive();
  use_breakdown_ = config.use_breakdown();
  use_convergence_boostup_ = config.use_convergence_boostup();
  init_velocity_variance_ = config.init_velocity_variance();
  init_acceleration_variance_ = config.init_acceleration_variance();
  measured_velocity_variance_ = config.measured_velocity_variance();
  predict_variance_per_sqrsec_ = config.predict_variance_per_sqrsec();
  boostup_history_size_minimum_ = config.boostup_history_size_minimum();
  boostup_history_size_maximum_ = config.boostup_history_size_maximum();
  converged_confidence_minimum_ = config.converged_confidence_minimum();
  noise_maximum_ = config.noise_maximum();
  trust_orientation_range_ = config.trust_orientation_range();

  motion_measurer_.reset(new MlfMotionMeasurement);

  motion_refiner_.reset(new MlfMotionRefiner);
  CHECK(motion_refiner_->Init());
  return true;
}

void MlfMotionFilter::UpdateWithObject(const MlfFilterOptions& options,
                                       const MlfTrackDataConstPtr& track_data,
                                       TrackedObjectPtr new_object) {
  if (track_data->age_ == 0) {
    InitializeTrackState(new_object);
    return;
  }
  if (new_object->is_background) {
    new_object->output_velocity.setZero();
    new_object->output_velocity_uncertainty = Eigen::Matrix3d::Identity();
    return;
  }
  // 1. compute measurement
  motion_measurer_->ComputeMotionMeasurment(track_data, new_object);
  if (!use_adaptive_) {
    new_object->update_quality = 1.0;
  }
  TrackedObjectConstPtr latest_object = track_data->GetLatestObject().second;
  // 2. kalman filter update
  KalmanFilterUpdateWithPartialObservation(track_data, latest_object,
                                           new_object);
  StateToBelief(new_object);
  // 3. boostup state and static object velocity clapping
  if (use_convergence_boostup_) {
    ConvergenceEstimationAndBoostUp(track_data, latest_object, new_object);
  }
  ClipingState(new_object);
  StateToBelief(new_object);
  // 4. online covariance estimation
  BeliefToOutput(new_object);
  // 5. post refine and recompute online covariance if necessary
  if (motion_refiner_->Refine(track_data, new_object)) {
    // use output velocity to recompute convergence confidence
    ComputeConvergenceConfidence(track_data, new_object, false);
    UpdateConverged(track_data, new_object);
  }
  OnlineCovarianceEstimation(track_data, new_object);
}

void MlfMotionFilter::UpdateWithoutObject(const MlfFilterOptions& options,
                                          double timestamp,
                                          MlfTrackDataPtr track_data) {
  return;
}

void MlfMotionFilter::InitializeTrackState(TrackedObjectPtr new_object) {
  new_object->boostup_need_history_size =
      static_cast<int>(boostup_history_size_minimum_);
  new_object->convergence_confidence = use_convergence_boostup_ ? 0.0 : 1.0;
  new_object->converged = !use_convergence_boostup_;
  new_object->update_quality = 1.0;
  new_object->selected_measured_velocity = Eigen::Vector3d::Zero();
  new_object->selected_measured_acceleration = Eigen::Vector3d::Zero();
  new_object->belief_anchor_point = new_object->anchor_point;
  new_object->belief_velocity = Eigen::Vector3d::Zero();
  new_object->belief_acceleration = Eigen::Vector3d::Zero();
  new_object->belief_velocity_gain = Eigen::Vector3d::Zero();
  new_object->state = Eigen::Vector4d::Zero();
  new_object->measurement_covariance = Eigen::Matrix4d();
  new_object->state_covariance = Eigen::Matrix4d();

  // covariances initialize
  new_object->velocity_covariance =
      Eigen::Matrix3d::Identity() * init_velocity_variance_;
  new_object->belief_velocity_online_covariance =
      new_object->velocity_covariance;

  new_object->state_covariance = Eigen::Matrix4d::Identity();
  new_object->state_covariance.block<2, 2>(0, 0) *= init_velocity_variance_;
  new_object->state_covariance.block<2, 2>(2, 2) *= init_acceleration_variance_;

  StateToBelief(new_object);
  BeliefToOutput(new_object);
}

void MlfMotionFilter::KalmanFilterUpdateWithPartialObservation(
    const MlfTrackDataConstPtr& track_data,
    const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object) {
  double range = new_object->object_ptr->center.norm();

  const Eigen::Vector4d& last_state = latest_object->state;
  const Eigen::Matrix4d& last_state_covariance =
      latest_object->state_covariance;

  double time_diff = new_object->object_ptr->latest_tracked_time -
                     latest_object->object_ptr->latest_tracked_time;
  if (time_diff < EPSION_TIME) {  // Very small time than assign
    time_diff = DEFAULT_FPS;
  }
  Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
  transition(0, 2) = transition(1, 3) = time_diff;

  // composition with rotation
  if (new_object->type != base::ObjectType::PEDESTRIAN &&
      range < trust_orientation_range_) {
    Eigen::Vector2d cur_dir = new_object->direction.head<2>();
    Eigen::Vector2d pre_dir = latest_object->direction.head<2>();
    cur_dir.normalize();
    pre_dir.normalize();
    double cos_theta = cur_dir.dot(pre_dir);
    double sin_theta = pre_dir(0) * cur_dir(1) - pre_dir(1) * cur_dir(0);
    Eigen::Matrix2d rot;
    rot << cos_theta, -sin_theta, sin_theta, cos_theta;
    Eigen::Matrix4d rot_extend = Eigen::Matrix4d::Zero();
    rot_extend.block<2, 2>(0, 0) = rot;
    rot_extend.block<2, 2>(2, 2) = rot;
    transition = rot_extend * transition;
  }

  auto& state = new_object->state;
  auto& state_covariance = new_object->state_covariance;
  auto measurement_covariance =
      new_object->measurement_covariance.block<2, 2>(0, 0);
  // 1. prediction stage
  Eigen::Matrix4d predict_covariance = Eigen::Matrix4d::Identity() *
                                       predict_variance_per_sqrsec_ *
                                       time_diff * time_diff;
  state = transition * last_state;
  state_covariance =
      transition * last_state_covariance * transition.transpose() +
      predict_covariance;

  // 2. measurement update stage
  Eigen::Vector2d measurement;
  measurement << new_object->selected_measured_velocity.head<2>();

  Eigen::Vector2d direction = new_object->direction.head<2>();
  direction.normalize();
  Eigen::Vector2d odirection(direction(1), -direction(0));
  if (new_object->type == base::ObjectType::PEDESTRIAN &&
      range < trust_orientation_range_) {
    measurement_covariance = Eigen::Matrix2d::Identity();
    measurement_covariance *= measured_velocity_variance_;
  } else {
    const double kVarianceAmplifier = 9.0;
    measurement_covariance =
        measured_velocity_variance_ * direction * direction.transpose() +
        (measured_velocity_variance_ +
         fabs(measurement.dot(odirection)) * kVarianceAmplifier) *
            odirection * odirection.transpose();
  }

  Eigen::Matrix<double, 2, 4> observation_transform;
  observation_transform.block<2, 2>(0, 0).setIdentity();
  observation_transform.block<2, 2>(0, 2).setZero();
  Eigen::Matrix<double, 4, 2> kalman_gain_matrix =
      static_cast<Eigen::Matrix<double, 4, 2, 0, 4, 2>>(
          state_covariance * observation_transform.transpose() *
          (observation_transform * state_covariance *
               observation_transform.transpose() +
           measurement_covariance)
              .inverse());
  Eigen::Vector4d state_gain =
      static_cast<Eigen::Matrix<double, 4, 1, 0, 4, 1>>(
          kalman_gain_matrix * (measurement - observation_transform * state));

  // 3. gain adjustment and estimate posterior
  StateGainAdjustment(track_data, latest_object, new_object, &state_gain);

  state = state + state_gain;
  state_covariance = (Eigen::Matrix4d::Identity() -
                      kalman_gain_matrix * observation_transform) *
                     state_covariance;

  // 4. finally, state to belief and output to keep consistency
  new_object->belief_velocity_gain << state_gain.head<2>(), 0;
}

void MlfMotionFilter::StateGainAdjustment(
    const MlfTrackDataConstPtr& track_data,
    const TrackedObjectConstPtr& latest_object,
    const TrackedObjectConstPtr& new_object, Eigen::Vector4d* gain) {
  // 1 quality penalize and gain break down
  if (use_adaptive_) {
    *gain *= new_object->update_quality;
  }
  // 2 breakdown the constrained the max allowed change of state
  if (use_breakdown_) {
    // velocity breakdown threshold
    double velocity_breakdown_threshold = 10.0 - (track_data->age_ - 1) * 2;
    velocity_breakdown_threshold = std::max(velocity_breakdown_threshold, 0.3);
    double velocity_gain = gain->head<2>().norm();
    if (velocity_gain > velocity_breakdown_threshold) {
      gain->head<2>() *= velocity_breakdown_threshold / velocity_gain;
    }
    // acceleration breakdown threshold
    double acceleration_breakdown_threshold = 2.0;
    double acceleration_gain = gain->tail<2>().norm();
    if (acceleration_gain > acceleration_breakdown_threshold) {
      gain->tail<2>() *= acceleration_breakdown_threshold / acceleration_gain;
    }
  }
}

void MlfMotionFilter::StateToBelief(TrackedObjectPtr object) {
  object->belief_velocity << object->state.head<2>(), 0;
  object->belief_acceleration << object->state.tail<2>(), 0;
}

void MlfMotionFilter::BeliefToOutput(TrackedObjectPtr object) {
  object->output_velocity = object->belief_velocity;
  object->output_velocity_uncertainty =
      object->belief_velocity_online_covariance;
}

void MlfMotionFilter::ClipingState(TrackedObjectPtr object) {
  if (object->state.head<2>().norm() < noise_maximum_) {
    object->state.setZero();
    object->belief_velocity_gain.setZero();
    object->state_covariance.block<2, 2>(0, 2).setZero();
    object->state_covariance.block<2, 2>(2, 0).setZero();
  }
}

void MlfMotionFilter::OnlineCovarianceEstimation(
    const MlfTrackDataConstPtr& track_data, TrackedObjectPtr object) {
  object->belief_velocity_online_covariance = Eigen::Matrix3d::Zero();
  size_t evaluate_window =
      std::min(track_data->history_objects_.size(),
               static_cast<size_t>(object->boostup_need_history_size));
  if (evaluate_window <= 0) {
    // a default large covariance
    object->belief_velocity_online_covariance = Eigen::Matrix3d::Identity() *
                                                predict_variance_per_sqrsec_ *
                                                predict_variance_per_sqrsec_;
    return;
  }
  // compute online covariance
  auto cur_obj_pair = track_data->history_objects_.rbegin();
  for (size_t i = 0; i < evaluate_window; ++i) {
    Eigen::Vector3d velocity_resisual =
        cur_obj_pair->second->selected_measured_velocity -
        object->output_velocity;
    if (velocity_resisual.head(2).norm() < DBL_EPSILON) {
      velocity_resisual =
          Eigen::Vector3d::Identity() * noise_maximum_ * noise_maximum_;
    }
    ++cur_obj_pair;
    object->belief_velocity_online_covariance.block<2, 2>(0, 0) =
        velocity_resisual.head<2>() * velocity_resisual.head<2>().transpose();
  }
  object->belief_velocity_online_covariance /=
      static_cast<double>(evaluate_window);
  object->belief_velocity_online_covariance +=
      Eigen::Matrix3d::Identity() * noise_maximum_ * noise_maximum_;
  object->output_velocity_uncertainty =
      object->belief_velocity_online_covariance;
}

void MlfMotionFilter::ConvergenceEstimationAndBoostUp(
    const MlfTrackDataConstPtr& track_data,
    const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object) {
  new_object->boostup_need_history_size =
      latest_object->boostup_need_history_size;
  new_object->converged = latest_object->converged;
  new_object->convergence_confidence = latest_object->convergence_confidence;

  // +1 means new object measure velocity
  // -1 means first object without measure velocity not considered
  size_t window_size = 1 + track_data->history_objects_.size() - 1;

  // Boostup convergence when its confidence is samll than minimum
  ComputeConvergenceConfidence(track_data, new_object, true);
  UpdateConverged(track_data, new_object);

  // do not boostup belief if usable measure velocity is not enough
  if (window_size <
      static_cast<size_t>(new_object->boostup_need_history_size)) {
    return;
  }
  // boostup belief if not converged yet
  if (!new_object->converged) {
    BoostupState(track_data, new_object);
    ComputeConvergenceConfidence(track_data, new_object, true);
    UpdateConverged(track_data, new_object);
  }
}

void MlfMotionFilter::ComputeConvergenceConfidence(
    const MlfTrackDataConstPtr& track_data, TrackedObjectPtr new_object,
    bool velocity_source_is_belief) {
  // Compute convergence score list
  std::vector<double> convergence_score_list;
  int boostup_need_history_size = new_object->boostup_need_history_size;
  auto& history_objects = track_data->history_objects_;

  convergence_score_list.resize(boostup_need_history_size, 0.0);
  double base_convergence_noise = 2 * measured_velocity_variance_;

  // new object score
  Eigen::Vector3d velocity_residual =
      new_object->selected_measured_velocity -
      (velocity_source_is_belief ? new_object->belief_velocity
                                 : new_object->output_velocity);
  convergence_score_list[0] =
      base_convergence_noise /
      std::max(base_convergence_noise, velocity_residual.norm());

  // history objects score
  size_t visible_obj_idx = 1;
  for (auto cur_object_pair = history_objects.rbegin();
       cur_object_pair != history_objects.rend(); ++cur_object_pair) {
    TrackedObjectPtr cur_object = cur_object_pair->second;
    if (visible_obj_idx >= convergence_score_list.size() ||
        visible_obj_idx >= history_objects.size()) {
      break;
    }
    velocity_residual =
        cur_object->selected_measured_velocity -
        (velocity_source_is_belief ? new_object->belief_velocity
                                   : new_object->output_velocity);
    convergence_score_list[visible_obj_idx] =
        base_convergence_noise /
        std::max(base_convergence_noise, velocity_residual.norm());
    ++visible_obj_idx;
  }
  // Compute convergence confidence
  if (!new_object->converged) {
    double min_convergence_confidence_score = 1.0;  // maximum
    for (size_t i = 0; i < static_cast<size_t>(boostup_need_history_size);
         ++i) {
      if (min_convergence_confidence_score > convergence_score_list[i]) {
        min_convergence_confidence_score = convergence_score_list[i];
      }
    }
    new_object->convergence_confidence =
        static_cast<float>(min_convergence_confidence_score);
  } else {
    double mean_convergence_confidence_score = 0;  // mean
    for (size_t i = 0; i < static_cast<size_t>(boostup_need_history_size);
         ++i) {
      mean_convergence_confidence_score += convergence_score_list[i];
    }
    mean_convergence_confidence_score /= boostup_need_history_size;
    new_object->convergence_confidence =
        static_cast<float>(mean_convergence_confidence_score);
  }
}

void MlfMotionFilter::BoostupState(const MlfTrackDataConstPtr& track_data,
                                   TrackedObjectPtr new_object) {
  // Compute min & max boosted velocity
  Eigen::Vector3d& new_obj_belief_velocity = new_object->belief_velocity;
  Eigen::Vector3d min_boosted_velocity = new_obj_belief_velocity;
  double min_boosted_velocity_norm = DBL_MAX;
  Eigen::Vector3d max_boosted_velocity = new_obj_belief_velocity;
  double max_boosted_velocity_norm = DBL_MIN;
  Eigen::Vector3d project_dir = min_boosted_velocity;
  project_dir(2) = 0.0;
  project_dir.normalize();

  int boostup_need_history_size = new_object->boostup_need_history_size;
  int actual_boostup_history_size =
      static_cast<size_t>(boostup_need_history_size) >
              track_data->history_objects_.size()
          ? static_cast<int>(track_data->history_objects_.size())
          : boostup_need_history_size;
  int boostup_used_history_size = 0;
  auto& history_objects = track_data->history_objects_;
  auto cur_object_pair = history_objects.rbegin();
  TrackedObjectPtr cur_object = new_object;
  while (boostup_used_history_size < actual_boostup_history_size &&
         cur_object_pair != history_objects.rend()) {
    ++boostup_used_history_size;
    Eigen::Vector3d measured_velocity = cur_object->selected_measured_velocity;
    Eigen::Vector3d measured_velocity_on_project_dir =
        common::Calculate2DXYProjectVector<double>(measured_velocity,
                                                   new_obj_belief_velocity);
    if (measured_velocity_on_project_dir.dot(new_obj_belief_velocity) <= 0) {
      measured_velocity_on_project_dir = Eigen::Vector3d::Zero();
    }
    double measured_velocity_norm_on_project_dir =
        measured_velocity_on_project_dir.norm();
    if (measured_velocity_norm_on_project_dir < min_boosted_velocity_norm) {
      min_boosted_velocity = measured_velocity_on_project_dir;
      min_boosted_velocity_norm = measured_velocity_norm_on_project_dir;
    }
    if (measured_velocity_norm_on_project_dir > max_boosted_velocity_norm) {
      max_boosted_velocity = measured_velocity_on_project_dir;
      max_boosted_velocity_norm = measured_velocity_norm_on_project_dir;
    }
    if (boostup_used_history_size > 1) {
      ++cur_object_pair;
    }
    if (cur_object_pair != history_objects.rend()) {
      cur_object = cur_object_pair->second;
    }
  }
  // Increase belief when belief less than min boosted velocity
  // Decrease belief when belief greater than max boosted velocity
  // now boosted_accelaration not used in original version, maybe use later
  if (min_boosted_velocity_norm > new_obj_belief_velocity.norm()) {
    // Eigen::Vector3d boosted_accelaration =
    // (min_boosted_velocity - new_obj_belief_velocity)/new_latest_time_diff_;
    new_obj_belief_velocity = min_boosted_velocity;
  } else if (max_boosted_velocity_norm < new_obj_belief_velocity.norm()) {
    // Eigen::Vector3d boosted_accelaration =
    // (max_boosted_velocity - new_obj_belief_velocity)/new_latest_time_diff_;
    new_obj_belief_velocity = max_boosted_velocity;
  }
  new_object->state.head<2>() = new_obj_belief_velocity.head<2>();
}

void MlfMotionFilter::UpdateConverged(const MlfTrackDataConstPtr& track_data,
                                      TrackedObjectPtr object) {
  if (object->convergence_confidence > converged_confidence_minimum_) {
    // set converged true
    object->converged = true;
    // increase cached history size if necessary
    if (track_data->history_objects_.size() <
        static_cast<size_t>(object->boostup_need_history_size)) {
      return;
    }
    if (static_cast<size_t>(object->boostup_need_history_size) <
        boostup_history_size_maximum_) {
      object->boostup_need_history_size += 1;
    }
  } else {
    object->converged = false;
  }
}

PERCEPTION_REGISTER_MLFFILTER(MlfMotionFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
