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

#include "modules/prediction/container/obstacles/obstacle.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <unordered_set>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacle;
using apollo::common::math::KalmanFilter;
using apollo::common::ErrorCode;
using apollo::common::Point3D;

std::mutex Obstacle::mutex_;

namespace {

double Damp(const double x, const double sigma) {
  return 1 / (1 + exp(1 / (std::fabs(x) + sigma)));
}

}  // namespace

Obstacle::Obstacle()
    : id_(-1),
      type_(PerceptionObstacle::UNKNOWN_MOVABLE),
      feature_history_(0),
      kf_motion_tracker_(),
      kf_motion_tracker_enabled_(false),
      kf_lane_trackers_(0) {}

Obstacle::~Obstacle() {
  id_ = -1;
  type_ = PerceptionObstacle::UNKNOWN_UNMOVABLE;
  feature_history_.clear();
  kf_lane_trackers_.clear();
  kf_motion_tracker_enabled_ = false;
  // TODO(author) current_lanes_.clear();
}

int Obstacle::id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return id_;
}

double Obstacle::timestamp() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (feature_history_.size() > 0) {
    return feature_history_.front().timestamp();
  } else {
    return 0.0;
  }
}

const Feature& Obstacle::feature(size_t i) {
  std::lock_guard<std::mutex> lock(mutex_);
  CHECK(i < feature_history_.size());
  return feature_history_[i];
}

Feature* Obstacle::mutable_feature(size_t i) {
  std::lock_guard<std::mutex> lock(mutex_);
  CHECK(i < feature_history_.size());
  return &feature_history_[i];
}

const Feature& Obstacle::latest_feature() {
  std::lock_guard<std::mutex> lock(mutex_);
  CHECK_GT(feature_history_.size(), 0);
  return feature_history_.front();
}

Feature* Obstacle::mutable_latest_feature() {
  std::lock_guard<std::mutex> lock(mutex_);

  CHECK_GT(feature_history_.size(), 0);
  return &(feature_history_.front());
}

size_t Obstacle::history_size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return feature_history_.size();
}

const KalmanFilter<double, 4, 2, 0>& Obstacle::kf_lane_tracker(
    const std::string& lane_id) {
  CHECK(kf_lane_trackers_.find(lane_id) != kf_lane_trackers_.end());
  return kf_lane_trackers_[lane_id];
}

void Obstacle::Insert(const PerceptionObstacle& perception_obstacle,
                      const double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (feature_history_.size() > 0 &&
      timestamp <= feature_history_.front().timestamp()) {
    AINFO << "Obstacle [" << id_ << "] received an older frame [" << timestamp
          << "] than the most recent timestamp [ "
          << feature_history_.front().timestamp() << "].";
    return;
  }

  Feature feature;
  if (SetId(perception_obstacle, &feature) == ErrorCode::PREDICTION_ERROR) {
    return;
  }
  if (SetType(perception_obstacle) == ErrorCode::PREDICTION_ERROR) {
    return;
  }
  SetTimestamp(perception_obstacle, timestamp, &feature);
  SetPosition(perception_obstacle, &feature);
  SetVelocity(perception_obstacle, &feature);
  SetAcceleration(&feature);
  SetTheta(perception_obstacle, &feature);
  if (!kf_motion_tracker_enabled_) {
    InitKFMotionTracker(&feature);
  }
  UpdateKFMotionTracker(&feature);
  SetCurrentLanes(&feature);
  SetNearbyLanes(&feature);
  UpdateKFLaneTrackers(&feature);
  InsertFeatureToHistory(&feature);
  SetMotionStatus();
}

ErrorCode Obstacle::SetId(const PerceptionObstacle& perception_obstacle,
                          Feature* feature) {
  if (!perception_obstacle.has_id()) {
    AERROR << "Obstacle has no ID.";
    return ErrorCode::PREDICTION_ERROR;
  }

  int id = perception_obstacle.id();
  if (id_ < 0) {
    id_ = id;
    AINFO << "Obstacle set id [" << id_ << "].";
  } else {
    if (id_ != id) {
      AERROR << "Obstacle [" << id_ << "] has a mismatched ID [" << id
             << "] from perception obstacle.";
      return ErrorCode::PREDICTION_ERROR;
    } else {
      feature->set_id(id);
    }
  }
  return ErrorCode::OK;
}

ErrorCode Obstacle::SetType(const PerceptionObstacle& perception_obstacle) {
  if (perception_obstacle.has_type()) {
    type_ = perception_obstacle.type();
    ADEBUG << "Obstacle [" << id_ << "] set type [" << type_ << "].";
  } else {
    AERROR << "Obstacle [" << id_ << "] has no type.";
    return ErrorCode::PREDICTION_ERROR;
  }
  return ErrorCode::OK;
}

void Obstacle::SetTimestamp(const PerceptionObstacle& perception_obstacle,
                            const double timestamp, Feature* feature) {
  double ts = timestamp;
  if (perception_obstacle.has_timestamp() &&
      perception_obstacle.timestamp() > 0.0) {
    ts = perception_obstacle.timestamp();
  }
  feature->set_timestamp(ts);

  ADEBUG << "Obstacle [" << id_ << "] set timestamp [" << std::fixed
         << std::setprecision(6) << ts << "].";
}

void Obstacle::SetPosition(const PerceptionObstacle& perception_obstacle,
                           Feature* feature) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  if (perception_obstacle.has_position()) {
    if (perception_obstacle.position().has_x()) {
      x = perception_obstacle.position().x();
    }
    if (perception_obstacle.position().has_y()) {
      y = perception_obstacle.position().y();
    }
    if (perception_obstacle.position().has_z()) {
      z = perception_obstacle.position().z();
    }
  }

  feature->mutable_position()->set_x(x);
  feature->mutable_position()->set_y(y);
  feature->mutable_position()->set_z(z);

  ADEBUG << "Obstacle [" << id_ << "] set position [" << std::fixed
         << std::setprecision(6) << x << ", " << std::fixed
         << std::setprecision(6) << y << ", " << std::fixed
         << std::setprecision(6) << z << "].";
}

void Obstacle::SetVelocity(const PerceptionObstacle& perception_obstacle,
                           Feature* feature) {
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double velocity_z = 0.0;

  if (perception_obstacle.has_velocity()) {
    if (perception_obstacle.velocity().has_x()) {
      velocity_x = perception_obstacle.velocity().x();
    }
    if (perception_obstacle.velocity().has_y()) {
      velocity_y = perception_obstacle.velocity().y();
    }
    if (perception_obstacle.velocity().has_z()) {
      velocity_z = perception_obstacle.velocity().z();
    }
  }

  feature->mutable_velocity()->set_x(velocity_x);
  feature->mutable_velocity()->set_y(velocity_y);
  feature->mutable_velocity()->set_z(velocity_z);

  double speed = std::hypot(std::hypot(velocity_x, velocity_y), velocity_z);
  double velocity_heading = std::atan2(velocity_y, velocity_x);
  feature->set_velocity_heading(velocity_heading);
  feature->set_speed(speed);

  ADEBUG << "Obstacle [" << id_ << "] set velocity [" << std::fixed
         << std::setprecision(6) << velocity_x << ", " << std::fixed
         << std::setprecision(6) << velocity_y << ", " << std::fixed
         << std::setprecision(6) << velocity_z << "], "
         << "velocity heading [" << velocity_heading << "] and speed [" << speed
         << "].";
}

void Obstacle::SetAcceleration(Feature* feature) {
  double acc_x = 0.0;
  double acc_y = 0.0;
  double acc_z = 0.0;

  if (feature_history_.size() > 0) {
    double curr_ts = feature->timestamp();
    double prev_ts = feature_history_.front().timestamp();

    const Point3D& curr_velocity = feature->velocity();
    const Point3D& prev_velocity = feature_history_.front().velocity();

    if (apollo::common::math::DoubleCompare(curr_ts, prev_ts) == 1) {
      double damping_x = Damp(curr_velocity.x(), 0.001);
      double damping_y = Damp(curr_velocity.y(), 0.001);
      double damping_z = Damp(curr_velocity.z(), 0.001);

      acc_x = (curr_velocity.x() - prev_velocity.x()) / (curr_ts - prev_ts);
      acc_y = (curr_velocity.y() - prev_velocity.y()) / (curr_ts - prev_ts);
      acc_z = (curr_velocity.z() - prev_velocity.z()) / (curr_ts - prev_ts);

      acc_x = apollo::common::math::Clamp(acc_x * damping_x, FLAGS_min_acc,
                                          FLAGS_max_acc);
      acc_y = apollo::common::math::Clamp(acc_y * damping_y, FLAGS_min_acc,
                                          FLAGS_max_acc);
      acc_z = apollo::common::math::Clamp(acc_z * damping_z, FLAGS_min_acc,
                                          FLAGS_max_acc);
    }
  }

  feature->mutable_acceleration()->set_x(acc_x);
  feature->mutable_acceleration()->set_y(acc_y);
  feature->mutable_acceleration()->set_z(acc_z);
  double acc = std::hypot(std::hypot(acc_x, acc_y), acc_z);
  feature->set_acc(acc);

  ADEBUG << "Obstacle [" << id_ << "] set acc [" << std::fixed
         << std::setprecision(6) << acc_x << ", " << std::fixed
         << std::setprecision(6) << acc_y << ", " << std::fixed
         << std::setprecision(6) << acc_z << "], "
         << "and acc [" << acc << "].";
}

void Obstacle::SetTheta(const PerceptionObstacle& perception_obstacle,
                        Feature* feature) {
  double theta = 0.0;
  if (perception_obstacle.has_theta()) {
    theta = perception_obstacle.theta();
  }
  feature->set_theta(theta);

  ADEBUG << "Obstacle [" << id_ << "] set theta [" << std::fixed
         << std::setprecision(6) << theta << "].";
}

void Obstacle::SetLengthWidthHeight(
    const PerceptionObstacle& perception_obstacle, Feature* feature) {
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  if (perception_obstacle.has_length()) {
    length = perception_obstacle.length();
  }
  if (perception_obstacle.has_width()) {
    width = perception_obstacle.width();
  }
  if (perception_obstacle.has_height()) {
    height = perception_obstacle.height();
  }

  feature->set_length(length);
  feature->set_width(width);
  feature->set_height(height);

  ADEBUG << "Obstacle [" << id_ << "] set dimension [" << std::fixed
         << std::setprecision(6) << length << ", " << std::fixed
         << std::setprecision(6) << width << ", " << std::fixed
         << std::setprecision(6) << height << "].";
}

void Obstacle::InitKFMotionTracker(Feature* feature) {
  // Set transition matrix F
  Eigen::Matrix<double, 6, 6> F;
  F.setIdentity();
  kf_motion_tracker_.SetTransitionMatrix(F);

  // Set observation matrix H
  Eigen::Matrix<double, 2, 6> H;
  H.setZero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  kf_motion_tracker_.SetObservationMatrix(H);

  // Set covariance of transition noise matrix Q
  Eigen::Matrix<double, 6, 6> Q;
  Q.setIdentity();
  Q *= FLAGS_q_var;
  kf_motion_tracker_.SetTransitionNoise(Q);

  // Set observation noise matrix R
  Eigen::Matrix<double, 2, 2> R;
  R.setIdentity();
  R *= FLAGS_r_var;
  kf_motion_tracker_.SetObservationNoise(R);

  // Set current state covariance matrix P
  Eigen::Matrix<double, 6, 6> P;
  P.setIdentity();
  P *= FLAGS_p_var;

  // Set initial state
  Eigen::Matrix<double, 6, 1> x;
  x(0, 0) = feature->position().x();
  x(1, 0) = feature->position().y();
  x(2, 0) = feature->velocity().x();
  x(3, 0) = feature->velocity().y();
  x(4, 0) = feature->acceleration().x();
  x(5, 0) = feature->acceleration().y();

  kf_motion_tracker_.SetStateEstimate(x, P);

  kf_motion_tracker_enabled_ = true;
}

void Obstacle::UpdateKFMotionTracker(Feature* feature) {
  double delta_ts = 0.0;
  if (feature_history_.size() > 0) {
    delta_ts = feature->timestamp() - feature_history_.front().timestamp();
  }
  if (delta_ts <= FLAGS_double_precision) {
    return;
  }
  // Set tansition matrix and predict
  auto F = kf_motion_tracker_.GetTransitionMatrix();
  F(0, 2) = delta_ts;
  F(0, 4) = delta_ts;
  F(1, 3) = 0.5 * delta_ts * delta_ts;
  F(1, 5) = 0.5 * delta_ts * delta_ts;
  F(2, 4) = delta_ts;
  F(3, 5) = delta_ts;
  kf_motion_tracker_.SetTransitionMatrix(F);
  kf_motion_tracker_.Predict();

  // Set observation and correct
  Eigen::Matrix<double, 2, 1> z;
  z(0, 0) = feature->position().x();
  z(1, 0) = feature->position().y();
  kf_motion_tracker_.Correct(z);

  UpdateMotionBelief(feature);
}

void Obstacle::UpdateMotionBelief(Feature* feature) {
  auto state = kf_motion_tracker_.GetStateEstimate();
  feature->mutable_t_position()->set_x(state(0, 0));
  feature->mutable_t_position()->set_y(state(1, 0));
  feature->mutable_t_velocity()->set_x(state(2, 0));
  feature->mutable_t_velocity()->set_y(state(3, 0));
  feature->set_t_velocity_heading(std::atan2(state(3, 0), state(2, 0)));
  double acc_x =
      apollo::common::math::Clamp(state(4, 0), FLAGS_min_acc, FLAGS_max_acc);
  double acc_y =
      apollo::common::math::Clamp(state(5, 0), FLAGS_min_acc, FLAGS_max_acc);
  feature->mutable_t_acceleration()->set_x(acc_x);
  feature->mutable_t_acceleration()->set_y(acc_y);
  ADEBUG << "Obstacle [" << id_ << "] "
         << "set tracked position [" << feature->t_position().x() << ", "
         << feature->t_position().y() << "] "
         << "and tracked velocity [" << feature->t_velocity().x() << ", "
         << feature->t_velocity().y() << "] "
         << "and tracked acc [" << feature->t_acceleration().x() << ", "
         << feature->t_acceleration().y() << "] "
         << "and tracked velocity heading [" << feature->t_velocity_heading()
         << "].";
}

void Obstacle::InitKFLaneTracker(const std::string& lane_id,
                                 const double beta) {
  KalmanFilter<double, 4, 2, 0> kf;

  // transition matrix: update delta_t at each processing step
  Eigen::Matrix<double, 4, 4> F;
  F.setIdentity();
  F(1, 1) = beta;
  kf.SetTransitionMatrix(F);

  // observation matrix
  Eigen::Matrix<double, 2, 4> H;
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  kf.SetObservationMatrix(H);

  // Set covariance of transition noise matrix Q
  Eigen::Matrix<double, 4, 4> Q;
  Q.setIdentity();
  Q *= FLAGS_q_var;
  kf.SetTransitionNoise(Q);

  // Set observation noise matrix R
  Eigen::Matrix<double, 2, 2> R;
  R.setIdentity();
  R *= FLAGS_r_var;
  kf.SetObservationNoise(R);

  // Set current state covariance matrix P
  Eigen::Matrix<double, 4, 4> P;
  P.setIdentity();
  P *= FLAGS_p_var;
  kf.SetStateCovariance(P);

  kf_lane_trackers_.emplace(lane_id, std::move(kf));
}

void Obstacle::UpdateKFLaneTrackers(Feature* feature) {
  if (!feature->has_lane()) {
    return;
  }
  std::unordered_set<std::string> lane_ids;
  for (auto& lane_feature : feature->lane().current_lane_feature()) {
    if (!lane_feature.lane_id().empty()) {
      lane_ids.insert(lane_feature.lane_id());
    }
  }
  for (auto& lane_feature : feature->lane().nearby_lane_feature()) {
    if (!lane_feature.lane_id().empty()) {
      lane_ids.insert(lane_feature.lane_id());
    }
  }
  if (lane_ids.empty()) {
    return;
  }

  auto iter = kf_lane_trackers_.begin();
  while (iter != kf_lane_trackers_.end()) {
    if (lane_ids.find(iter->first) == lane_ids.end()) {
      iter = kf_lane_trackers_.erase(iter);
    } else {
      ++iter;
    }
  }

  double ts = feature->timestamp();
  double speed = feature->timestamp();
  double acc = feature->acc();
  for (auto& lane_feature : feature->lane().current_lane_feature()) {
    std::string lane_id = lane_feature.lane_id();
    if (lane_id.empty()) {
      continue;
    }
    double s = lane_feature.lane_s();
    double l = lane_feature.lane_l();
    UpdateKFLaneTracker(lane_id, s, l, speed, acc, ts, FLAGS_go_approach_rate);
  }
  for (auto& nearby_lane_feature : feature->lane().nearby_lane_feature()) {
    std::string lane_id = nearby_lane_feature.lane_id();
    if (lane_id.empty()) {
      continue;
    }
    double s = nearby_lane_feature.lane_s();
    double l = nearby_lane_feature.lane_l();
    UpdateKFLaneTracker(lane_id, s, l, speed, acc, ts,
                        FLAGS_cutin_approach_rate);
  }

  UpdateLaneBelief(feature);
}

void Obstacle::UpdateKFLaneTracker(const std::string& lane_id,
                                   const double lane_s, const double lane_l,
                                   const double lane_speed,
                                   const double lane_acc,
                                   const double timestamp, const double beta) {
  KalmanFilter<double, 4, 2, 0>* kf_ptr = nullptr;
  if (kf_lane_trackers_.find(lane_id) != kf_lane_trackers_.end()) {
    kf_ptr = &kf_lane_trackers_[lane_id];
    if (kf_ptr != nullptr) {
      double delta_ts = 0.0;
      if (feature_history_.size() > 0) {
        delta_ts = timestamp - feature_history_.front().timestamp();
      }
      if (delta_ts > FLAGS_double_precision) {
        auto F = kf_ptr->GetTransitionMatrix();
        F(0, 2) = delta_ts;
        F(0, 3) = 0.5 * delta_ts * delta_ts;
        F(2, 3) = delta_ts;
        kf_ptr->SetTransitionMatrix(F);
        kf_ptr->Predict();

        Eigen::Matrix<double, 2, 1> z;
        z(0, 0) = lane_s;
        z(1, 0) = lane_l;
        kf_ptr->Correct(z);
      }
    } else {
      kf_lane_trackers_.erase(lane_id);
    }
  }

  if (kf_lane_trackers_.find(lane_id) == kf_lane_trackers_.end()) {
    InitKFLaneTracker(lane_id, beta);
    kf_ptr = &kf_lane_trackers_[lane_id];
    if (kf_ptr != nullptr) {
      Eigen::Matrix<double, 4, 1> state;
      state(0, 0) = lane_s;
      state(1, 0) = lane_l;
      state(2, 0) = lane_speed;
      state(3, 0) = lane_acc;

      auto P = kf_ptr->GetStateCovariance();
      kf_ptr->SetStateEstimate(state, P);
    }
  }
}

void Obstacle::UpdateLaneBelief(Feature* feature) {
  if ((!feature->has_lane()) || (!feature->lane().has_lane_feature())) {
    return;
  }
  const std::string& lane_id = feature->lane().lane_feature().lane_id();
  if (lane_id.empty()) {
    return;
  }

  KalmanFilter<double, 4, 2, 0>* kf_ptr = nullptr;
  if (kf_lane_trackers_.find(lane_id) != kf_lane_trackers_.end()) {
    kf_ptr = &kf_lane_trackers_[lane_id];
  }
  if (kf_ptr == nullptr) {
    return;
  }

  double lane_speed = kf_ptr->GetStateEstimate()(2, 0);
  double lane_acc = apollo::common::math::Clamp(
      kf_ptr->GetStateEstimate()(2, 0), FLAGS_min_acc, FLAGS_max_acc);
  feature->set_t_speed(lane_speed);
  feature->set_t_acc(lane_acc);

  ADEBUG << "Obstacle [" << id_ << "] set tracked speed [" << lane_speed
         << " and tracked acc [" << lane_acc << "]";
}

void Obstacle::SetCurrentLanes(Feature* feature) {
  // TODO(kechxu) implement
}

void Obstacle::SetNearbyLanes(Feature* feature) {
  // TODO(kechxu) implement
}

void Obstacle::SetMotionStatus() {
  bool is_still = true;
  int history_size = static_cast<int>(feature_history_.size());
  if (history_size < 2) {
    ADEBUG << "Obstacle [" << id_ << "] has no history and "
           << "is considered still [default = true].";
    if (history_size > 0) {
      feature_history_.front().set_is_still(is_still);
      ADEBUG << "Obstacle [" << id_ << "] has stillness status [" << is_still
             << "].";
    }
    return;
  }

  double start_x = 0.0;
  double start_y = 0.0;
  double avg_drift_x = 0.0;
  double avg_drift_y = 0.0;
  int len = std::min(history_size, FLAGS_still_obstacle_history_length);
  CHECK_GT(len, 1);

  auto feature_riter = feature_history_.rbegin();
  if (FLAGS_enable_kf_tracking) {
    start_x = feature_riter->t_position().x();
    start_y = feature_riter->t_position().y();
  } else {
    start_x = feature_riter->position().x();
    start_y = feature_riter->position().y();
  }
  ++feature_riter;
  while (feature_riter != feature_history_.rend()) {
    if (FLAGS_enable_kf_tracking) {
      avg_drift_x += (feature_riter->t_position().x() - start_x) / (len - 1);
      avg_drift_y += (feature_riter->t_position().y() - start_y) / (len - 1);
    } else {
      avg_drift_x += (feature_riter->position().x() - start_x) / (len - 1);
      avg_drift_y += (feature_riter->position().y() - start_y) / (len - 1);
    }
    ++feature_riter;
  }

  double delta_ts = feature_history_.front().timestamp() -
                    feature_history_.back().timestamp();
  double std = FLAGS_still_obstacle_position_std;
  double speed_sensibility =
      std::sqrt(2 * history_size) * 4 * std / ((history_size + 1) * delta_ts);
  double speed = (FLAGS_enable_kf_tracking ? feature_history_.front().t_speed()
                                           : feature_history_.front().speed());
  double speed_threshold = FLAGS_still_obstacle_speed_threshold;
  if (apollo::common::math::DoubleCompare(speed, speed_threshold) < 0) {
    is_still = true;
    ADEBUG << "Obstacle [" << id_
           << "] has a small speed and is considered still.";
  } else if (apollo::common::math::DoubleCompare(speed_sensibility,
                                                 speed_threshold) < 0) {
    is_still = false;
    ADEBUG << "Obstacle [" << id_ << "] has a too short history ["
           << history_size
           << "] and is considered moving [sensibility = " << speed_sensibility
           << "]";
  } else {
    double distance = std::hypot(avg_drift_x, avg_drift_y);
    double distance_std = std::sqrt(2.0 / len) * std;
    if (apollo::common::math::DoubleCompare(distance, 2.0 * distance_std) > 0) {
      is_still = false;
      ADEBUG << "Obstacle [" << id_ << "] is moving.";
    } else {
      is_still = true;
      ADEBUG << "Obstacle [" << id_ << "] is still.";
    }
  }
}

void Obstacle::InsertFeatureToHistory(Feature* feature) {
  feature_history_.push_front(std::move(*feature));
  ADEBUG << "Obstacle [" << id_ << "] inserted a frame into the history.";
}

}  // namespace prediction
}  // namespace apollo
