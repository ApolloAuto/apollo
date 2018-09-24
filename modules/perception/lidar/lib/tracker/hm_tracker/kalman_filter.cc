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
#include "modules/perception/lidar/lib/tracker/hm_tracker/kalman_filter.h"

#include "modules/perception/base/log.h"
#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lib/io/protobuf_util.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/proto/hm_tracker_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

// noise maximum
double KalmanFilter::s_noise_maximum_ = 0.1;

// adaptive
bool KalmanFilter::s_use_adaptive_ = false;

// parameters
double KalmanFilter::s_centroid_measurement_noise_ = 0.4;
double KalmanFilter::s_centroid_init_velocity_variance_ = 5;
double KalmanFilter::s_propagation_variance_xy_ = 10;
double KalmanFilter::s_propagation_variance_z_ = 10;
Eigen::Matrix3d KalmanFilter::s_covariance_propagation_uncertainty_ =
    Eigen::Matrix3d::Identity();

// convergence
bool KalmanFilter::s_use_convergence_boostup_ = false;
size_t KalmanFilter::s_boostup_history_size_minimum_ = 0;
size_t KalmanFilter::s_boostup_history_size_maximum_ = 0;
double KalmanFilter::s_converged_confidence_minimum_ = 0.9;

// motion score
int KalmanFilter::s_motion_score_window_size_ = 5;

void KalmanFilter::SetUseAdaptive(bool use_adaptive) {
  s_use_adaptive_ = use_adaptive;
}

void KalmanFilter::SetUseConvergenceBoostup(bool use_convergence_boostup) {
  s_use_convergence_boostup_ = use_convergence_boostup;
}

void KalmanFilter::SetConvergedConfidenceMinimum(
    double converged_confidence_minimum) {
  s_converged_confidence_minimum_ = converged_confidence_minimum;
  if (s_converged_confidence_minimum_ < 0.0) {
    s_converged_confidence_minimum_ = 0.0;
  }
  if (s_converged_confidence_minimum_ > 1.0) {
    s_converged_confidence_minimum_ = 1.0;
  }
}

void KalmanFilter::SetParams(double centroid_measurement_noise,
                             double init_velocity_variance,
                             double propagation_variance_xy,
                             double propagation_variance_z) {
  if (centroid_measurement_noise >= 0.0) {
    s_centroid_measurement_noise_ = centroid_measurement_noise;
  }
  if (init_velocity_variance >= 0.0) {
    s_centroid_init_velocity_variance_ = init_velocity_variance;
  }
  if (propagation_variance_xy >= 0.0) {
    s_propagation_variance_xy_ = propagation_variance_xy;
  }
  if (propagation_variance_z >= 0.0) {
    s_propagation_variance_z_ = propagation_variance_z;
  }
}

void KalmanFilter::SetBoostupHistorySizeMinmax(
    size_t boostup_history_size_minimum) {
  s_boostup_history_size_minimum_ = boostup_history_size_minimum;
  s_boostup_history_size_maximum_ = boostup_history_size_minimum * 2;
}

KalmanFilter::KalmanFilter()
    : track_data_(nullptr),
      new_object_(nullptr),
      new_object_time_(0.0),
      latest_object_(nullptr),
      latest_object_time_(0.0),
      new_latest_time_diff_(0.0),
      temporal_info_filled_(false) {}

bool KalmanFilter::Init(const FilterOption& option) {
  lib::ConfigManager* config_manager =
      lib::Singleton<lib::ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig("KalmanFilter", &model_config))
      << "Failed to get model config: KalmanFilter";

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";
  std::string config_file;
  config_file = lib::FileUtil::GetAbsolutePath(work_root, root_path);
  config_file =
      lib::FileUtil::GetAbsolutePath(config_file, "kalman_filter.conf");
  // get config params
  KalmanFilterConfig config_params;
  CHECK(lib::ParseProtobufFromFile<KalmanFilterConfig>(config_file,
                                                       &config_params))
      << "Failed to parse KalmanFilterConfig config file.";

  s_noise_maximum_ = config_params.noise_maximum();

  s_use_adaptive_ = config_params.use_adaptive();

  s_centroid_measurement_noise_ = config_params.centroid_measurement_noise();
  s_centroid_init_velocity_variance_ =
      config_params.centroid_init_velocity_variance();
  s_propagation_variance_xy_ = config_params.propagation_variance_xy();
  s_propagation_variance_z_ = config_params.propagation_variance_z();
  s_covariance_propagation_uncertainty_ = Eigen::Matrix3d::Zero();
  s_covariance_propagation_uncertainty_(0, 0) = s_propagation_variance_xy_;
  s_covariance_propagation_uncertainty_(1, 1) = s_propagation_variance_xy_;
  s_covariance_propagation_uncertainty_(2, 2) = s_propagation_variance_z_;

  s_use_convergence_boostup_ = config_params.use_convergence_boostup();
  s_boostup_history_size_minimum_ =
      config_params.boostup_history_size_minimum();
  s_boostup_history_size_maximum_ = 2 * s_boostup_history_size_minimum_;
  s_converged_confidence_minimum_ =
      config_params.converged_confidence_minimum();
  return true;
}

Eigen::VectorXd KalmanFilter::Predict(TrackDataPtr track_data, double time) {
  // Compute predict states
  Eigen::VectorXd predicted_state;
  predicted_state.resize(6);
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track_data->GetLatestObject();
  double latest_time = latest_obj_pair.first;
  TrackedObjectPtr latest_obj = latest_obj_pair.second;
  if (latest_obj) {
    double time_diff = time - latest_time;
    Eigen::Vector3d movement = latest_obj->belief_anchor_point +
                               latest_obj->belief_velocity * time_diff;
    predicted_state(0) = movement(0);
    predicted_state(1) = movement(1);
    predicted_state(2) = movement(2);
    predicted_state(3) = latest_obj->belief_velocity(0);
    predicted_state(4) = latest_obj->belief_velocity(1);
    predicted_state(5) = latest_obj->belief_velocity(2);
  }
  return predicted_state;
}

bool KalmanFilter::FillTemporaryInfo(TrackDataPtr track_data,
                                     TrackedObjectPtr new_object, double time) {
  temporal_info_filled_ = false;
  if (!track_data || !new_object) {
    return temporal_info_filled_;
  }
  track_data_ = track_data;
  new_object_ = new_object;
  new_object_time_ = time;
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track_data_->GetLatestObject();
  latest_object_time_ = latest_obj_pair.first;
  latest_object_ = latest_obj_pair.second;
  new_latest_time_diff_ = new_object_time_ - latest_object_time_;
  temporal_info_filled_ = true;
  return temporal_info_filled_;
}

void KalmanFilter::UpdateWithObject(TrackDataPtr track_data,
                                    TrackedObjectPtr new_object,
                                    double new_time) {
  if (!FillTemporaryInfo(track_data, new_object, new_time)) {
    return;
  }
  if (track_data_->age_ <= 0) {
    InitializeTrackedObject();
  } else if (new_object_->is_fake) {
    UpdateWithFakeObject();
  } else {
    UpdateWithTrueObject();
  }
  BeliefToOutput();
}

void KalmanFilter::InitializeTrackedObject() {
  // state initialize
  new_object_->boostup_need_history_size = s_boostup_history_size_minimum_;
  new_object_->convergence_confidence = s_use_convergence_boostup_ ? 0.0 : 1.0;
  new_object_->converged = s_use_convergence_boostup_ ? false : true;
  new_object_->update_quality = 1.0;
  new_object_->selected_measured_velocity = Eigen::Vector3d::Zero();
  new_object_->selected_measured_acceleration = Eigen::Vector3d::Zero();
  new_object_->belief_anchor_point = new_object_->anchor_point;
  new_object_->belief_velocity = Eigen::Vector3d::Zero();
  new_object_->belief_acceleration = Eigen::Vector3d::Zero();
  new_object_->belief_velocity_gain = Eigen::Vector3d::Zero();

  // covariances initialize
  new_object_->covariance_velocity =
      Eigen::Matrix3d::Identity() * s_centroid_init_velocity_variance_;
  new_object_->belief_velocity_online_covariance =
      new_object_->covariance_velocity;

  // output initialize
  BeliefToOutput();
}

void KalmanFilter::UpdateWithTrueObject() {
  NewObjectPropagate();

  ComputeMotionScore();
  ComputeNewObjectUpdateQuality();
  ComputeNewObjectSelectMeasuredVelocity();
  ComputeNewObjectSelectMeasuredAcceleration();
  ComputeNewObjectBeliefAnchorPoint();
  ComputeNewObjectBeliefVelocityAndCov();
  ComputeNewObjectBeliefAcceleration();

  // Compute convergence confidence
  if (s_use_convergence_boostup_) {
    ComputeNewObjectConvergenceAndBoostupBelief();
  }
  ClapingNewObjectBelief();
  ComputeVelocityOnlineCovariance();
}

void KalmanFilter::UpdateWithFakeObject() { NewObjectPropagate(); }

void KalmanFilter::NewObjectPropagate() {
  // Nothing to propagate!
  if (!latest_object_->valid) {
    new_object_->covariance_velocity = latest_object_->covariance_velocity;
    return;
  }
  new_object_->covariance_velocity = latest_object_->covariance_velocity +
                                     s_covariance_propagation_uncertainty_ *
                                         new_latest_time_diff_ *
                                         new_latest_time_diff_;
}

void KalmanFilter::ComputeNewObjectSelectMeasuredVelocity() {
  SelectMeasuredVelocityAccordingMotionConsistency();
}

void KalmanFilter::SelectMeasuredVelocityAccordingMotionConsistency() {
  // First determine use nearest_corner_velocity or not
  const int window_size = 3;
  Eigen::Vector3d average_corner_velocity;
  CalculateAverageCornerVelocity(window_size, &average_corner_velocity);

  // determine use nearest_corner_velocity or select four corner_velocities
  // nearest_corner_velocity > threshold && in accord with history velocity
  bool use_nearest_corner_velocity = false;
  const double corner_velocity_thresh = 0.25;
  double corner_velocity_norm = average_corner_velocity.norm();
  if (corner_velocity_norm > corner_velocity_thresh) {
    average_corner_velocity.normalize();
    double project_velocity_norm =
        new_object_->measured_nearest_corner_velocity.dot(
            average_corner_velocity);
    if (project_velocity_norm > 0 &&
        project_velocity_norm > corner_velocity_norm / 2.0 &&
        project_velocity_norm < corner_velocity_norm * 1.5) {
      use_nearest_corner_velocity = true;
    }
  }

  // Select measured velocity among candidates according motion consistency
  int corner_index = 0;
  float corner_velocity_gain = 0;
  if (!use_nearest_corner_velocity) {
    std::vector<float> corner_velocity_gain_norms(4);
    for (int i = 0; i < 4; ++i) {
      corner_velocity_gain_norms[i] =
          (new_object_->measured_corners_velocity[i] -
           latest_object_->belief_velocity)
              .norm();
    }
    std::vector<float>::iterator corener_min_gain =
        std::min_element(std::begin(corner_velocity_gain_norms),
                         std::end(corner_velocity_gain_norms));
    corner_velocity_gain = *corener_min_gain;
    corner_index = corener_min_gain - corner_velocity_gain_norms.begin();
  } else {
    corner_velocity_gain = (new_object_->measured_nearest_corner_velocity -
                            latest_object_->belief_velocity)
                               .norm();
  }

  std::vector<float> velocity_gain_norms(3);

  velocity_gain_norms[0] = corner_velocity_gain;
  velocity_gain_norms[1] = (new_object_->measured_barycenter_velocity -
                            latest_object_->belief_velocity)
                               .norm();
  velocity_gain_norms[2] =
      (new_object_->measured_center_velocity - latest_object_->belief_velocity)
          .norm();
  std::vector<float>::iterator min_gain = std::min_element(
      std::begin(velocity_gain_norms), std::end(velocity_gain_norms));
  int min_gain_pos = min_gain - velocity_gain_norms.begin();
  if (min_gain_pos == 0) {
    if (use_nearest_corner_velocity) {
      new_object_->selected_measured_velocity =
          new_object_->measured_nearest_corner_velocity;
    } else {
      new_object_->selected_measured_velocity =
          new_object_->measured_corners_velocity[corner_index];
    }
  }
  if (min_gain_pos == 1) {
    new_object_->selected_measured_velocity =
        new_object_->measured_barycenter_velocity;
  }
  if (min_gain_pos == 2) {
    new_object_->selected_measured_velocity =
        new_object_->measured_center_velocity;
  }
}

void KalmanFilter::ComputeNewObjectSelectMeasuredAcceleration() {
  std::map<double, TrackedObjectPtr>& history_objects =
      track_data_->history_objects_;
  if (history_objects.size() < 4) {
    new_object_->selected_measured_acceleration = Eigen::Vector3d::Zero();
    return;
  }

  std::pair<double, TrackedObjectPtr> old_object_pair =
      track_data_->GetHistoryObject(-2);
  double time_diff = new_object_time_ - old_object_pair.first;

  Eigen::Vector3d& new_object_measure_velocity =
      new_object_->selected_measured_velocity;
  Eigen::Vector3d& old_object_measure_velocity =
      old_object_pair.second->selected_measured_velocity;
  new_object_->selected_measured_acceleration =
      (new_object_measure_velocity - old_object_measure_velocity) / time_diff;
}

void KalmanFilter::ComputeNewObjectBeliefAnchorPoint() {
  new_object_->belief_anchor_point = new_object_->anchor_point;
}

void KalmanFilter::ComputeNewObjectBeliefVelocityAndCov() {
  // Compute kalman gain
  Eigen::Matrix3d mat_c = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_q =
      s_centroid_measurement_noise_ * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_k =
      new_object_->covariance_velocity * mat_c.transpose() *
      (mat_c * new_object_->covariance_velocity * mat_c.transpose() + mat_q)
          .inverse();

  // Compute priori velocity
  Eigen::Vector3d priori_velocity =
      latest_object_->belief_velocity +
      latest_object_->belief_acceleration * new_latest_time_diff_;

  // Compute posterior belief
  Eigen::Vector3d velocity_gain =
      mat_k *
      (new_object_->selected_measured_velocity - mat_c * priori_velocity);
  // Adaptive
  if (s_use_adaptive_) {
    velocity_gain *= new_object_->update_quality;
  }

  // Breakdown
  float breakdown_threshold = ComputeBreakdownThreshold();
  if (velocity_gain.norm() > breakdown_threshold) {
    velocity_gain.normalize();
    velocity_gain *= breakdown_threshold;
  }
  new_object_->belief_velocity = priori_velocity + velocity_gain;
  new_object_->belief_velocity_gain = velocity_gain;

  // Compute posterior covariance
  new_object_->covariance_velocity =
      (Eigen::Matrix3d::Identity() - mat_k * mat_c) *
      new_object_->covariance_velocity;

  // After the first measurement, the filter is valid
  new_object_->valid = true;
}

void KalmanFilter::ComputeNewObjectBeliefAcceleration() {
  // Compute kalman gain based on velocity
  Eigen::Matrix3d mat_c = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mat_q =
      s_centroid_measurement_noise_ * Eigen::Matrix3d::Identity() * 3;
  Eigen::Matrix3d mat_k =
      new_object_->covariance_velocity * mat_c.transpose() *
      (mat_c * new_object_->covariance_velocity * mat_c.transpose() + mat_q)
          .inverse();

  // Compute posterior belief
  Eigen::Vector3d acceleration_gain =
      mat_k * (new_object_->selected_measured_acceleration -
               mat_c * latest_object_->belief_acceleration);

  // Adaptive
  if (s_use_adaptive_) {
    acceleration_gain *= new_object_->update_quality;
  }

  // Breakdown
  float breakdown_threshold = 2;
  if (acceleration_gain.norm() > breakdown_threshold) {
    acceleration_gain.normalize();
    acceleration_gain *= breakdown_threshold;
  }

  new_object_->belief_acceleration =
      latest_object_->belief_acceleration + acceleration_gain;

  // Logic check
  float logic_threshold = 5;
  if (new_object_->belief_acceleration.norm() > logic_threshold) {
    new_object_->belief_acceleration.normalize();
    new_object_->belief_acceleration *= logic_threshold;
  }
}

void KalmanFilter::ComputeNewObjectUpdateQuality() {
  if (!s_use_adaptive_) {
    new_object_->update_quality = 1.0;
    return;
  }
  // Compute update quality for adaptive filtering
  // Strategy I: define update quality by use association score
  float update_quality_by_association_score =
      ComputeUpdateQualityByAssociationScore();
  float update_quality_by_point_number_diff =
      ComputeUpdateQualityByPointNumberDiff();
  float update_quality = update_quality_by_association_score;
  if (update_quality_by_association_score >
      update_quality_by_point_number_diff) {
    update_quality = update_quality_by_point_number_diff;
  }
  new_object_->update_quality = update_quality;
}

float KalmanFilter::ComputeUpdateQualityByAssociationScore() {
  // Compute update quality by using association score
  float association_score = new_object_->association_score;
  double update_quality = 1 - association_score;
  update_quality *= update_quality;
  return update_quality;
}

float KalmanFilter::ComputeUpdateQualityByPointNumberDiff() {
  // Compute update quality by using point number diff
  int old_pt_num = latest_object_->object_ptr->lidar_supplement.cloud.size();
  int new_pt_num = new_object_->object_ptr->lidar_supplement.cloud.size();
  if (old_pt_num <= 0 && new_pt_num <= 0) {
    return 0;
  }
  float pt_diff_ratio =
      1 - fabs(old_pt_num - new_pt_num) / std::max(old_pt_num, new_pt_num);
  return pt_diff_ratio;
}

float KalmanFilter::ComputeBreakdownThreshold() {
  // Compute breakdown threshold
  // Strategy I: define breakdown threshold as a linear
  float breakdown_threshold = 10 - (track_data_->age_ - 1) * 2;
  breakdown_threshold = breakdown_threshold > 0.3 ? breakdown_threshold : 0.3;
  return breakdown_threshold;
}

void KalmanFilter::ComputeNewObjectConvergenceAndBoostupBelief() {
  new_object_->boostup_need_history_size =
      latest_object_->boostup_need_history_size;
  new_object_->converged = latest_object_->converged;
  new_object_->convergence_confidence = latest_object_->convergence_confidence;

  // +1 means new object measure velocity
  // -1 means first object without measure velocity not considered
  int useable_measure_velocity_size = 1 + track_data_->total_visible_count_ - 1;

  // Boostup convergence when its confidence is samll than minimum
  ComputeConvergenceConfidence(useable_measure_velocity_size);
  UpdateConverged(useable_measure_velocity_size);

  // do not boostup belief if useable measure velocity is not enought
  if (useable_measure_velocity_size < new_object_->boostup_need_history_size) {
    return;
  }
  // boostup belief if not converged yet
  if (!new_object_->converged) {
    LOG_INFO << "boostup unconverged filter!";
    BoostupBelief(useable_measure_velocity_size);
    ComputeConvergenceConfidence(useable_measure_velocity_size);
    UpdateConverged(useable_measure_velocity_size);
  }
}

void KalmanFilter::UpdateConverged(int useable_measure_velocity_size) {
  if (new_object_->convergence_confidence > s_converged_confidence_minimum_) {
    // set converged true
    new_object_->converged = true;
    // increase cached history size if necessary
    if (useable_measure_velocity_size <
        new_object_->boostup_need_history_size) {
      return;
    }
    if (new_object_->boostup_need_history_size <
        s_boostup_history_size_maximum_) {
      new_object_->boostup_need_history_size += 1;
    }
  } else {
    new_object_->converged = false;
  }
}

void KalmanFilter::ComputeConvergenceConfidence(
    int useable_measure_velocity_size) {
  // Compute convergence score list
  std::vector<double> convergence_score_list;
  int boostup_need_history_size = new_object_->boostup_need_history_size;
  std::map<double, TrackedObjectPtr>& history_object =
      track_data_->history_objects_;

  convergence_score_list.resize(boostup_need_history_size, 0.0);
  double base_convergence_noise = 2 * s_centroid_measurement_noise_;

  // new object score
  Eigen::Vector3d velocity_residual =
      new_object_->selected_measured_velocity - new_object_->belief_velocity;
  convergence_score_list[0] =
      base_convergence_noise /
      std::max(base_convergence_noise, velocity_residual.norm());

  // history objects score
  int visible_obj_idx = 1;
  std::map<double, TrackedObjectPtr>::reverse_iterator cur_object_pair;
  for (cur_object_pair = history_object.rbegin();
       cur_object_pair != history_object.rend(); ++cur_object_pair) {
    TrackedObjectPtr cur_object = cur_object_pair->second;
    if (cur_object->is_fake) {
      continue;
    }
    if (visible_obj_idx >= convergence_score_list.size() ||
        visible_obj_idx >= useable_measure_velocity_size) {
      break;
    }
    velocity_residual =
        cur_object->selected_measured_velocity - new_object_->belief_velocity;
    convergence_score_list[visible_obj_idx] =
        base_convergence_noise /
        std::max(base_convergence_noise, velocity_residual.norm());
    ++visible_obj_idx;
  }
  // Compute convergence confidence
  if (!new_object_->converged) {
    double min_convergence_confidence_score = 1;  // maximum
    for (int i = 0; i < boostup_need_history_size; ++i) {
      if (min_convergence_confidence_score > convergence_score_list[i]) {
        min_convergence_confidence_score = convergence_score_list[i];
      }
    }
    new_object_->convergence_confidence = min_convergence_confidence_score;
  } else {
    double mean_convergence_confidence_score = 0;  // minimum
    for (int i = 0; i < boostup_need_history_size; ++i) {
      mean_convergence_confidence_score += convergence_score_list[i];
    }
    mean_convergence_confidence_score /= boostup_need_history_size;
    new_object_->convergence_confidence = mean_convergence_confidence_score;
  }
}

void KalmanFilter::BoostupBelief(int useable_measure_velocity_size) {
  // Compute min & max boosted velocity
  Eigen::Vector3d& new_obj_belief_velocity = new_object_->belief_velocity;
  Eigen::Vector3d min_boosted_velocity = new_obj_belief_velocity;
  double min_boosted_velocity_norm = DBL_MAX;
  Eigen::Vector3d max_boosted_velocity = new_obj_belief_velocity;
  double max_boosted_velocity_norm = DBL_MIN;
  Eigen::Vector3d project_dir = min_boosted_velocity;
  project_dir(2) = 0.0;
  project_dir.normalize();

  int boostup_need_history_size = new_object_->boostup_need_history_size;
  int actual_boostup_history_size =
      boostup_need_history_size > useable_measure_velocity_size
          ? useable_measure_velocity_size
          : boostup_need_history_size;
  int boostup_used_history_size = 0;
  std::map<double, TrackedObjectPtr>& history_objects =
      track_data_->history_objects_;
  std::map<double, TrackedObjectPtr>::reverse_iterator cur_object_pair =
      history_objects.rbegin();
  TrackedObjectPtr cur_object = new_object_;
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
    for (; cur_object_pair != history_objects.rend(); ++cur_object_pair) {
      cur_object = cur_object_pair->second;
      if (!cur_object->is_fake) {
        break;  // break for, not while
      }
    }
  }
  // Increase belief when belief less than min boosted velocity
  // Decrease belief when belief greater than max boosted velocity
  // now boosted_accelaration not used in orignal version, maybe use later
  if (min_boosted_velocity_norm > new_obj_belief_velocity.norm()) {
    // Eigen::Vector3d boosted_accelaration =
    // (min_boosted_velocity - new_obj_belief_velocity)/new_latest_time_diff_;
    new_obj_belief_velocity = min_boosted_velocity;
  } else if (max_boosted_velocity_norm < new_obj_belief_velocity.norm()) {
    // Eigen::Vector3d boosted_accelaration =
    // (max_boosted_velocity - new_obj_belief_velocity)/new_latest_time_diff_;
    new_obj_belief_velocity = max_boosted_velocity;
  }
}

void KalmanFilter::ClapingNewObjectBelief() {
  // Claping noise
  if (new_object_->belief_velocity.norm() < s_noise_maximum_) {
    new_object_->belief_velocity = Eigen::Vector3d::Zero();
    new_object_->belief_acceleration = Eigen::Vector3d::Zero();
    new_object_->belief_velocity_gain = Eigen::Vector3d::Zero();
  }
  if (new_object_->belief_acceleration.norm() < s_noise_maximum_ * 4) {
    new_object_->belief_acceleration = Eigen::Vector3d::Zero();
  }
}

void KalmanFilter::ComputeVelocityOnlineCovariance() {
  new_object_->belief_velocity_online_covariance = Eigen::Matrix3d::Zero();
  size_t evaluate_window =
      std::min(track_data_->history_objects_.size(),
               static_cast<size_t>(new_object_->boostup_need_history_size));
  if (evaluate_window <= 0) {
    new_object_->belief_velocity_online_covariance =
        Eigen::Matrix3d::Identity() * s_propagation_variance_xy_ *
        s_propagation_variance_xy_;
    return;
  }
  // compute online covariance
  std::map<double, TrackedObjectPtr>::reverse_iterator cur_obj_pair =
      track_data_->history_objects_.rbegin();
  for (size_t i = 0; i < evaluate_window; ++i) {
    Eigen::Vector3d velocity_resisual =
        cur_obj_pair->second->selected_measured_velocity -
        new_object_->belief_velocity;
    if (velocity_resisual.head(2).norm() < DBL_EPSILON) {
      velocity_resisual =
          Eigen::Vector3d::Identity() * s_noise_maximum_ * s_noise_maximum_;
    }
    ++cur_obj_pair;
    new_object_->belief_velocity_online_covariance(0, 0) +=
        velocity_resisual(0) * velocity_resisual(0);
    new_object_->belief_velocity_online_covariance(0, 1) +=
        velocity_resisual(0) * velocity_resisual(1);
    new_object_->belief_velocity_online_covariance(1, 0) +=
        velocity_resisual(1) * velocity_resisual(0);
    new_object_->belief_velocity_online_covariance(1, 1) +=
        velocity_resisual(1) * velocity_resisual(1);
  }
  new_object_->belief_velocity_online_covariance /= evaluate_window;
}

void KalmanFilter::BeliefToOutput() {
  new_object_->output_velocity = new_object_->belief_velocity;
  new_object_->output_velocity_uncertainty =
      new_object_->belief_velocity_online_covariance;
  new_object_->output_center = new_object_->center;
  new_object_->output_direction = new_object_->direction;
  new_object_->output_size = new_object_->size;
}

void KalmanFilter::CalculateAverageCornerVelocity(
    int window_size, Eigen::Vector3d* average_velocity) {
  average_velocity->setZero();

  size_t use_num = 0;
  Eigen::Vector3d sum_corner_velocity = Eigen::Vector3d::Zero();
  for (auto history_object_pair = track_data_->history_objects_.rbegin();
       history_object_pair != track_data_->history_objects_.rend();
       ++history_object_pair) {
    sum_corner_velocity +=
        history_object_pair->second->measured_nearest_corner_velocity;
    if (++use_num == window_size) {
      break;
    }
  }
  // use_num impossible to be zero
  *average_velocity = sum_corner_velocity / use_num;
}

void KalmanFilter::ComputeMotionScore() {
  // compute theta diff, result is in [0, pi)
  auto compute_theta_diff = [](double theta1, double theta2) {
    double theta_diff = fabs(theta1 - theta2);
    theta_diff = theta_diff > M_PI ? 2 * M_PI - theta_diff : theta_diff;
    return theta_diff;
  };

  // the first s_motion_score_window_size frames measurement is unstable
  if (track_data_->history_objects_.size() < s_motion_score_window_size_ - 1) {
    new_object_->motion_score = Eigen::Vector3d(1, 1, 1);
    return;
  }

  Eigen::Vector3d ave_velocity;
  CalculateAverageCornerVelocity(s_motion_score_window_size_, &ave_velocity);

  // the first is current frame measurement
  double average_theta = std::atan2(ave_velocity[1], ave_velocity[0]);
  double velocity_norm = ave_velocity.norm();

  const Eigen::Vector3d& new_corner_velocity =
      new_object_->measured_nearest_corner_velocity;
  // variance of velocity norm
  double new_obj_norm_diff = velocity_norm - new_corner_velocity.norm();
  double variance_sum = new_obj_norm_diff * new_obj_norm_diff;
  // diff of velocity theta
  double new_obj_theta =
      std::atan2(new_corner_velocity[1], new_corner_velocity[0]);
  double variance_theta = compute_theta_diff(new_obj_theta, average_theta);

  size_t use_num = 1;
  // add history frame measurement
  for (auto history_object_pair = track_data_->history_objects_.rbegin();
       history_object_pair != track_data_->history_objects_.rend();
       ++history_object_pair) {
    const Eigen::Vector3d& velocity_history =
        history_object_pair->second->measured_nearest_corner_velocity;
    // variance of velocity norm
    double norm_diff = velocity_norm - velocity_history.norm();
    variance_sum += (norm_diff * norm_diff);

    // diff of velocity theta
    double new_theta = std::atan2(velocity_history[1], velocity_history[0]);
    double theta_diff = compute_theta_diff(new_theta, average_theta);
    variance_theta += theta_diff;

    if (++use_num == s_motion_score_window_size_) {
      break;
    }
  }

  // average of variance of norm & diff of theta
  double norm_variance =
      sqrt(variance_sum / s_motion_score_window_size_) / velocity_norm;
  double theta_variance = variance_theta / s_motion_score_window_size_;

  std::deque<double>& history_norm_variance =
      track_data_->history_norm_variance_;
  std::deque<double>& history_theta_variance =
      track_data_->history_theta_variance_;
  if (history_norm_variance.size() >= s_motion_score_window_size_) {
    history_norm_variance.pop_front();
    history_theta_variance.pop_front();
  }
  history_norm_variance.push_back(norm_variance);
  history_theta_variance.push_back(theta_variance);

  // average of history of score
  double norm_mean_multi_frame =
      std::accumulate(std::begin(history_norm_variance),
                      std::end(history_norm_variance), 0.0) /
      history_norm_variance.size();
  double theta_mean_multi_frame =
      std::accumulate(std::begin(history_theta_variance),
                      std::end(history_theta_variance), 0.0) /
      history_theta_variance.size();

  // variance of first score
  double norm_variance_multi_frame = 0;
  for (size_t i = 0; i < history_norm_variance.size(); ++i) {
    double norm_mean_diff = history_norm_variance[i] - norm_mean_multi_frame;
    norm_variance_multi_frame += norm_mean_diff * norm_mean_diff;
  }
  norm_variance_multi_frame /= history_norm_variance.size();

  new_object_->motion_score(0) = norm_mean_multi_frame;
  new_object_->motion_score(1) = theta_mean_multi_frame;
  new_object_->motion_score(2) = norm_variance_multi_frame;
}

PERCEPTION_REGISTER_TRACKFILTER(KalmanFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
