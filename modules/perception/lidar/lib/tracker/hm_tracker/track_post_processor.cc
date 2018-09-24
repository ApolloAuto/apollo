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
#include <algorithm>
#include <utility>

#include "modules/perception/base/object_types.h"
#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/track_post_processor.h"

namespace apollo {
namespace perception {
namespace lidar {

// temporary solution, please check following params are same with kalman_filter
double TrackPostProcessor::s_centroid_measurement_noise_ = 0.4;
double TrackPostProcessor::s_converged_confidence_minimum_ = 0.5;
double TrackPostProcessor::s_propagation_variance_xy_ = 10;
double TrackPostProcessor::s_noise_maximum_ = 0.1;

void TrackPostProcessor::PostProcess(TrackDataPtr track_data) {
  SmoothTrackVelocity(track_data);
  SmoothTrackOrientation(track_data);
}

void TrackPostProcessor::SmoothTrackVelocity(TrackDataPtr track_data) {
  // Smooth velocity over track history
  // 1. keep motion if acceleration of filter is greater than a threshold
  std::pair<double, TrackedObjectPtr> current_object_pair =
      track_data->GetLatestObject();
  TrackedObjectPtr current_object = current_object_pair.second;

  std::pair<double, TrackedObjectPtr> previous_object_pair(
      0.0, TrackedObjectPtr(nullptr));
  if ((track_data->history_objects_).size() > 1) {
    previous_object_pair = track_data->GetHistoryObject(-1);
  } else {
    return;
  }

  TrackedObjectPtr previous_object = previous_object_pair.second;

  double current_timestamp = current_object_pair.first;
  double pervious_timestamp = previous_object_pair.first;
  double time_diff = current_timestamp - pervious_timestamp;
  Eigen::Vector3d filter_velocity_gain = current_object->belief_velocity_gain;
  // _filter->get_velocity_gain(&filter_velocity_gain);
  double filter_acceleration_gain = filter_velocity_gain.norm() / time_diff;
  bool need_keep_motion =
      filter_acceleration_gain > s_claping_acceleration_threshold_;
  // use tighter threshold for pedestrian
  if (filter_acceleration_gain > s_claping_acceleration_threshold_ / 2 &&
      current_object->type == base::ObjectType::PEDESTRIAN) {
    need_keep_motion = true;
  }

  if (need_keep_motion && (!current_object->converged)) {
    // LOG_INFO << "track " << track_data->track_id_ << " filterrrr keep
    // motion";
    Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
    current_velocity = previous_object->output_velocity;
    current_object->output_velocity = current_velocity;
    UpdateTrackConvergenceState(track_data);
    UpdateTrackVelocityOnlineCovariance(track_data);
    LOG_INFO << "TrackPostProcessor::keep motion because of extraodinary "
                "acceleration.";
    // keep static hypothesis
    return;
  }
  // 2. static hypothesis check
  // first check motion score, then check state consistency
  bool is_static_hypothesis = CheckStaticHypothesisByMotionScore(track_data) ||
                              CheckStaticHypothesisByState(track_data);

  if (is_static_hypothesis) {
    // LOG_INFO << "track " << track_data->track_id_ << " filterrrr static";
    current_object->output_velocity = Eigen::Vector3d::Zero();
    UpdateTrackConvergenceState(track_data);
    UpdateTrackVelocityOnlineCovariance(track_data);
    LOG_INFO
        << "TrackPostProcessor::set velocity to zero because of noise claping.";
  }
}

void TrackPostProcessor::SmoothTrackOrientation(TrackDataPtr track_data) {
  // Smooth orientation over track history
  std::pair<double, TrackedObjectPtr> current_object_pair =
      track_data->GetLatestObject();
  TrackedObjectPtr current_object = current_object_pair.second;
  Eigen::Vector3d current_object_dir = current_object->output_direction;
  current_object_dir.normalize();
  Eigen::Vector3d current_motion_dir = current_object->output_velocity;
  current_motion_dir(2) = 0.0f;
  if (current_motion_dir.norm() > FLT_EPSILON) {
    current_motion_dir.normalize();
  }
  double current_speed = current_object->output_velocity.head(2).norm();
  double speed_threshold = current_object->output_size[0] / 2.0;
  bool speed_is_obvious = current_speed > speed_threshold;

  if (speed_is_obvious) {
    current_object_dir = current_motion_dir;
  } else {
    std::pair<double, TrackedObjectPtr> previous_object_pair(
        0.0, TrackedObjectPtr(nullptr));
    if ((track_data->history_objects_).size() > 1) {
      previous_object_pair = track_data->GetHistoryObject(-1);
    } else {
      return;
    }
    TrackedObjectPtr previous_object = previous_object_pair.second;
    Eigen::Vector3d previous_object_dir = previous_object->output_direction;
    common::CalculateMostConsistentBBoxDir2DXY(previous_object_dir,
                                               &current_object_dir);
    // compute_most_consistent_bbox_direction(previous_object_dir,
    // &current_object_dir);
    current_object_dir = previous_object_dir + current_object_dir * 0.1;
    if (current_object_dir.norm() < FLT_EPSILON) {
      current_object_dir = previous_object_dir;
    }
    current_object_dir.normalize();
  }

  Eigen::Vector3d bk_size = current_object->output_size;
  Eigen::Vector3d bk_dir = current_object->output_direction;
  Eigen::Vector3d bk_center = current_object->output_center;
  Eigen::Vector3f new_size_tmp;
  Eigen::Vector3d new_center;
  double minimum_edge_length = 0.01;
  base::PointDCloud& cloud =
      (current_object->object_ptr->lidar_supplement).cloud_world;
  common::CalculateBBoxSizeCenter2DXY(cloud, current_object_dir.cast<float>(),
                                      &new_size_tmp, &new_center,
                                      minimum_edge_length);
  Eigen::Vector3d new_size = new_size_tmp.cast<double>();

  if (new_size[0] * new_size[1] < 2 * bk_size[0] * bk_size[1]) {
    current_object->output_direction = current_object_dir;
    current_object->output_center = new_center.cast<double>();
    current_object->output_size = new_size;
  }
}

bool TrackPostProcessor::CheckStaticHypothesisByState(TrackDataPtr track_data) {
  std::pair<double, TrackedObjectPtr> current_object_pair =
      track_data->GetLatestObject();
  TrackedObjectPtr current_object = current_object_pair.second;

  std::pair<double, TrackedObjectPtr> previous_object_pair(
      0.0, TrackedObjectPtr(nullptr));
  if ((track_data->history_objects_).size() > 1) {
    previous_object_pair = track_data->GetHistoryObject(-1);
  } else {
    return false;
  }
  TrackedObjectPtr previous_object = previous_object_pair.second;
  // Check whether track is static or not
  // evaluate speed noise level, the less the level is the
  // greater the probability of noise is
  // double speed = _belief_velocity.head(2).norm();
  double speed = current_object->output_velocity.head(2).norm();
  bool velocity_noise_level_is_0 = speed < (s_claping_speed_threshold_ / 8);
  bool velocity_noise_level_is_1 = speed < (s_claping_speed_threshold_ / 4);
  bool velocity_noise_level_is_2 = speed < (s_claping_speed_threshold_ / 2);
  bool velocity_noise_level_is_3 = speed < (s_claping_speed_threshold_ / 1);
  // believe track is staic if velocity noise level is 0
  // use loose threshold for object is not pedestrian
  if (velocity_noise_level_is_1 &&
      current_object->type != base::ObjectType::PEDESTRIAN) {
    velocity_noise_level_is_0 = true;
  }
  if (velocity_noise_level_is_0) {
    return true;
  }
  // believe track is static if velocity noise level is
  // 1 && angle change level is 0
  // use loose threshold for object is not pedstrian
  if (velocity_noise_level_is_2 &&
      current_object->type != base::ObjectType::PEDESTRIAN) {
    velocity_noise_level_is_1 = true;
  }
  double reasonable_angle_change_maximum_0 = M_PI / 6;
  bool velocity_angle_change_level_is_0 =
      CheckStaticHypothesisByVelocityAngleChange(
          track_data, reasonable_angle_change_maximum_0);
  if (velocity_noise_level_is_1 && velocity_angle_change_level_is_0) {
    return true;
  }
  // believe track is static if velocity noise level is
  // 2 && angle change level is 1
  // use loose threshold for object is not pedestrian
  if (velocity_noise_level_is_3 &&
      current_object->type != base::ObjectType::PEDESTRIAN) {
    velocity_noise_level_is_2 = true;
  }
  double reasonable_angle_change_maximum_1 = M_PI / 4;
  bool velocity_angle_change_level_is_1 =
      CheckStaticHypothesisByVelocityAngleChange(
          track_data, reasonable_angle_change_maximum_1);
  if (velocity_noise_level_is_2 && velocity_angle_change_level_is_1) {
    return true;
  }
  // Need to notice: claping small velocity may not reasonable
  // when the true velocity of target
  // object is really small. e.g. a moving out vehicle in a parking lot.
  // Thus, instead of clapping all the small velocity,
  // we clap those whose history trajectory
  // or performance is close to a static one.
  return false;
}

bool TrackPostProcessor::CheckStaticHypothesisByVelocityAngleChange(
    TrackDataPtr track_data, const double& reasonable_angle_change_maximum) {
  // Sub-strategy of checking whether track is static or
  // not: let track be static if its angle
  // change of consecutive velocity estimation pair is
  // greater than a given reasonable maximum
  // use default threshold if given one is illegal
  double local_reasonable_angle_change_maximum =
      reasonable_angle_change_maximum;
  if (reasonable_angle_change_maximum < 0 ||
      reasonable_angle_change_maximum > M_PI) {
    LOG_WARN
        << "illegal angle change maximum! use default one (PI / 4) instead";
    local_reasonable_angle_change_maximum = M_PI / 4.0;
  }
  // believe angle change is obvious if one of estimation pair is
  // extrodinary small
  std::pair<double, TrackedObjectPtr> current_object_pair =
      track_data->GetLatestObject();
  TrackedObjectPtr current_object = current_object_pair.second;

  std::pair<double, TrackedObjectPtr> previous_object_pair(
      0.0, TrackedObjectPtr(nullptr));
  if ((track_data->history_objects_).size() > 1) {
    previous_object_pair = track_data->GetHistoryObject(-1);
  } else {
    return false;
  }
  TrackedObjectPtr previous_object = previous_object_pair.second;

  Eigen::Vector3d previous_velocity = previous_object->output_velocity;
  Eigen::Vector3d current_velocity = current_object->output_velocity;
  if (previous_velocity.norm() < FLT_EPSILON) {
    return false;
  }
  if (current_velocity.norm() < FLT_EPSILON) {
    return true;
  }
  // believe angle change is obvious if it is greater than threshold

  // double velocity_angle_change = vector_theta_2d_xy(previous_velocity,
  // current_velocity);
  double velocity_angle_change = common::CalculateTheta2DXY<double>(
      previous_velocity.cast<double>(), current_velocity.cast<double>());
  if (fabs(velocity_angle_change) > local_reasonable_angle_change_maximum) {
    return true;
  }
  // Let track be static if angle change of current velocity
  // & previous heading is larger
  // than given reasonable maximum.
  Eigen::Vector3d previous_direction = previous_object->output_direction;
  //  double velocity_heading_angle_change_1 =
  //          vector_theta_2d_xy(previous_direction, current_velocity);
  double velocity_heading_angle_change_1 =
      common::CalculateTheta2DXY<double>(previous_direction, current_velocity);
  previous_direction *= -1;
  //  double velocity_heading_angle_change_2 =
  //          vector_theta_2d_xy(previous_direction, current_velocity);
  double velocity_heading_angle_change_2 =
      common::CalculateTheta2DXY<double>(previous_direction, current_velocity);
  velocity_angle_change = std::min(fabs(velocity_heading_angle_change_1),
                                   fabs(velocity_heading_angle_change_2));
  if (fabs(velocity_angle_change) > local_reasonable_angle_change_maximum) {
    return true;
  }
  return false;
}

bool TrackPostProcessor::CheckStaticHypothesisByMotionScore(
    TrackDataPtr track_data) {
  std::pair<double, TrackedObjectPtr> current_object_pair =
      track_data->GetLatestObject();
  TrackedObjectPtr current_object = current_object_pair.second;

  // do not check static score when
  // 1. track history size < thresh (in case of unstable measurement)
  // 2. object velocity > thresh (assumed abnormal velocity is relatively small)
  size_t kHistorySizeTh = 5;
  double kAbnormalVelocityTh = 2.0;
  if (track_data->history_objects_.size() <= kHistorySizeTh ||
      current_object->output_velocity.norm() > kAbnormalVelocityTh) {
    return false;
  }

  // the following parameters set by visulization result
  const double kNormMeanTh = 0.2, kNormMeanScale = 0.05;
  const double kThetaMeanTh = 0.1, kThetaMeanScale = 0.05;
  const double kNormVarianceTh = 0.03, kNormVarianceScale = 0.01;

  const Eigen::Vector3d& motion_score = current_object->motion_score;
  double norm_mean_score =
      WelshVarLoss(motion_score(0), kNormMeanTh, kNormMeanScale);
  double theta_mean_score =
      WelshVarLoss(motion_score(1), kThetaMeanTh, kThetaMeanScale);
  double norm_variance_score =
      WelshVarLoss(motion_score(2), kNormVarianceTh, kNormVarianceScale);

  std::vector<double> probs;
  probs.push_back(norm_mean_score);
  probs.push_back(theta_mean_score);
  probs.push_back(norm_variance_score);
  double final_score = FusedMultipleProb(probs);

  bool current_frame_motion_state = false;
  const double kPubScoreTh = 0.8;
  if (final_score > kPubScoreTh) {
    current_frame_motion_state = true;
  }

  // state transition
  // case 1: TRUSTED_MOVE
  //         do not set static
  // case 2: SKEPTICAL_MOVE
  //         1) current frame is in motion (do not set static)
  //            if) continuous_motion_frames_ >= 10 -> TRUSTED_MOVE
  //         2) current frame is static (set static by pub_remain_frames_)
  //            if) continuous_static_frames_ >= 5 -> STATIC
  // case 3: STATIC
  //         1) current frame is in motion (do not set static)
  //            if) continuous_motion_frames_ >= 5 -> SKEPTICAL_MOVE
  //         2) current frame is static (set static by pub_remain_frames_)
  bool motion_state = false;
  if (track_data->motion_state_ == MotionState::TRUSTED_MOVE) {
    motion_state = true;
  } else if (track_data->motion_state_ == MotionState::SKEPTICAL_MOVE) {
    if (current_frame_motion_state) {
      track_data->continuous_static_frames_ = 0;
      ++track_data->continuous_motion_frames_;
      track_data->pub_remain_frames_ = 2;
      motion_state = true;
      if (track_data->continuous_motion_frames_ >= 10) {
        track_data->motion_state_ = MotionState::TRUSTED_MOVE;
      }
    } else {
      ++track_data->continuous_static_frames_;
      track_data->continuous_motion_frames_ = 0;
      track_data->pub_remain_frames_ = 0;
      if (track_data->continuous_static_frames_ >= 5) {
        track_data->motion_state_ = MotionState::STATIC;
      } else {
        motion_state = true;
      }
    }
  } else {
    if (current_frame_motion_state) {
      track_data->continuous_static_frames_ = 0;
      ++track_data->continuous_motion_frames_;
      track_data->pub_remain_frames_ = 2;
      motion_state = true;
      if (track_data->continuous_motion_frames_ >= 5) {
        track_data->motion_state_ = MotionState::SKEPTICAL_MOVE;
      }
    } else {
      ++track_data->continuous_static_frames_;
      track_data->continuous_motion_frames_ = 0;
      if (track_data->pub_remain_frames_ > 0) {
        --track_data->pub_remain_frames_;
        motion_state = true;
      }
    }
  }

  // check consistency when measurement is unstable
  if (!motion_state && track_data->should_check_velocity_consistency_) {
    motion_state = CheckVelocityConsistency(track_data);
  } else {
    // means measurement is stable, there's no need to check consistency
    track_data->should_check_velocity_consistency_ = false;
  }

  return !motion_state;
}

bool TrackPostProcessor::CheckVelocityConsistency(TrackDataPtr track_data) {
  std::pair<double, TrackedObjectPtr> current_object_pair =
      track_data->GetLatestObject();
  const Eigen::Vector3d& current_velocity =
      current_object_pair.second->output_velocity;

  std::pair<double, TrackedObjectPtr> previous_object_pair =
      track_data->GetHistoryObject(-1);
  const Eigen::Vector3d& previous_velocity =
      previous_object_pair.second->output_velocity;

  // main purpose is to check whether two velocity is similar or not, only for
  // moving object
  if (current_velocity.isZero() || previous_velocity.isZero()) {
    track_data->should_check_velocity_consistency_ = false;
    return false;
  }

  double new_theta = std::atan2(current_velocity[1], current_velocity[0]);
  double old_theta = std::atan2(previous_velocity[1], previous_velocity[0]);
  double theta_diff = fabs(new_theta - old_theta);
  theta_diff = theta_diff > M_PI ? 2 * M_PI - theta_diff : theta_diff;

  const double kNormLower = 1, kNormUpper = 3;
  const double kThetaTh = M_PI / 12, kThetaScale = M_PI / 18;

  // due to there may be considerable diff with velocities for different frame
  // use velocity norm score instead of norm diff score
  double norm_score =
      WelshBoundLoss(previous_velocity.norm(), kNormLower, kNormUpper);
  double theta_score = WelshVarLoss(theta_diff, kThetaTh, kThetaScale);

  double prob = (norm_score * theta_score) /
                (2 * norm_score * theta_score + 1 - norm_score - theta_score);

  if (prob > 0.95) {
    return true;
  }

  // velocity is unconsistency, means there's high probability of static
  // hypothesis,
  // no need to check consistency (to avoid velocity disturbance)
  track_data->should_check_velocity_consistency_ = false;
  return false;
}

double TrackPostProcessor::WelshVarLoss(double dist, double th, double scale) {
  double p = 1e-6;
  if (dist < th) {
    p = 1 - 1e-6;
  } else {
    dist -= th;
    dist /= scale;
    p = std::exp(-dist * dist);
  }
  return p;
}

double TrackPostProcessor::WelshBoundLoss(double dist, double lower,
                                          double upper) {
  double p = 1e-6;
  if (dist < lower) {
    p = 1e-6;
  } else if (dist > upper) {
    p = 1 - 1e-6;
  } else {
    double square_scale = 9.0 / (2.0 * (upper - lower) * (upper - lower));
    double diff = dist - upper;
    p = std::exp(-diff * diff * square_scale);
  }
  return p;
}

double TrackPostProcessor::FusedMultipleProb(const std::vector<double>& probs) {
  std::vector<double> log_odd_probs = probs;
  auto prob_to_log_odd = [](double p) { return std::log(p / (1 - p)); };
  auto log_odd_to_prob = [](double log_odd_p) {
    double tmp = std::exp(log_odd_p);
    return tmp / (tmp + 1);
  };
  for (auto& log_odd_prob : log_odd_probs) {
    log_odd_prob = prob_to_log_odd(log_odd_prob);
  }
  double log_odd_probs_sum =
      std::accumulate(log_odd_probs.begin(), log_odd_probs.end(), 0.0);
  return log_odd_to_prob(log_odd_probs_sum);
}

void TrackPostProcessor::UpdateTrackConvergenceState(TrackDataPtr track_data) {
  // Compute convergence score list
  TrackedObjectPtr current_object = track_data->GetLatestObject().second;

  int boostup_need_history_size = current_object->boostup_need_history_size;
  std::map<double, TrackedObjectPtr>& history_object =
      track_data->history_objects_;

  std::vector<double> convergence_score_list;
  convergence_score_list.resize(boostup_need_history_size, 0.0);

  int useable_measure_velocity_size = 1 + track_data->total_visible_count_ - 1;
  double base_convergence_noise = 2 * s_centroid_measurement_noise_;

  int visible_obj_idx = 0;
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
    Eigen::Vector3d velocity_residual = cur_object->selected_measured_velocity -
                                        current_object->output_velocity;
    convergence_score_list[visible_obj_idx] =
        base_convergence_noise /
        std::max(base_convergence_noise, velocity_residual.norm());
    ++visible_obj_idx;
  }
  // Compute convergence confidence
  if (!current_object->converged) {
    double min_convergence_confidence_score = 1;  // maximum
    for (int i = 0; i < boostup_need_history_size; ++i) {
      if (min_convergence_confidence_score > convergence_score_list[i]) {
        min_convergence_confidence_score = convergence_score_list[i];
      }
    }
    current_object->convergence_confidence = min_convergence_confidence_score;
  } else {
    double mean_convergence_confidence_score = 0;  // minimum
    for (int i = 0; i < boostup_need_history_size; ++i) {
      mean_convergence_confidence_score += convergence_score_list[i];
    }
    mean_convergence_confidence_score /= boostup_need_history_size;
    current_object->convergence_confidence = mean_convergence_confidence_score;
  }

  if (current_object->convergence_confidence >
      s_converged_confidence_minimum_) {
    current_object->converged = true;
  } else {
    current_object->converged = false;
  }
}

void TrackPostProcessor::UpdateTrackVelocityOnlineCovariance(
    TrackDataPtr track_data) {
  TrackedObjectPtr current_object = track_data->GetLatestObject().second;
  TrackedObjectPtr last_object = track_data->GetHistoryObject(-2).second;
  if (last_object == nullptr) {
    /* if history length is 1, use defaulat intialization as online-cov */
    return;
  }
  current_object->belief_velocity_online_covariance = Eigen::Matrix3d::Zero();
  std::map<double, TrackedObjectPtr>& history_objects =
      track_data->history_objects_;
  size_t evaluate_window =
      std::min(track_data->history_objects_.size(),
               static_cast<size_t>(last_object->boostup_need_history_size));
  if (evaluate_window <= 0) {
    current_object->belief_velocity_online_covariance =
        Eigen::Matrix3d::Identity() * s_propagation_variance_xy_ *
        s_propagation_variance_xy_;
  }
  // compute velocity online covariance
  std::map<double, TrackedObjectPtr>::reverse_iterator cur_object_pair =
      history_objects.rbegin();
  for (size_t i = 0; i < evaluate_window; ++i) {
    Eigen::Vector3d velocity_resisual =
        cur_object_pair->second->selected_measured_velocity -
        current_object->belief_velocity;
    if (velocity_resisual.head(2).norm() < DBL_EPSILON) {
      velocity_resisual =
          Eigen::Vector3d::Identity() * s_noise_maximum_ * s_noise_maximum_;
    }
    ++cur_object_pair;
    current_object->belief_velocity_online_covariance(0, 0) +=
        velocity_resisual(0) * velocity_resisual(0);
    current_object->belief_velocity_online_covariance(0, 1) +=
        velocity_resisual(0) * velocity_resisual(1);
    current_object->belief_velocity_online_covariance(1, 0) +=
        velocity_resisual(1) * velocity_resisual(0);
    current_object->belief_velocity_online_covariance(1, 1) +=
        velocity_resisual(1) * velocity_resisual(1);
  }
  current_object->belief_velocity_online_covariance /= evaluate_window;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
