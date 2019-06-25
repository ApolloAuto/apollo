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
#include <iomanip>
#include <limits>
#include <list>
#include <unordered_set>

#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/container/obstacles/obstacle_clusters.h"
#include "modules/prediction/network/rnn_model/rnn_model.h"

namespace apollo {
namespace prediction {

using common::ErrorCode;
using common::PathPoint;
using common::Point3D;
using common::math::KalmanFilter;
using hdmap::JunctionInfo;
using hdmap::LaneInfo;
using perception::PerceptionObstacle;

namespace {

double Damp(const double x, const double sigma) {
  return 1.0 / (1.0 + std::exp(1.0 / (std::fabs(x) + sigma)));
}

bool IsClosed(const double x0, const double y0, const double theta0,
              const double x1, const double y1, const double theta1) {
  double angle_diff = std::fabs(common::math::AngleDiff(theta0, theta1));
  double distance = std::hypot(x0 - x1, y0 - y1);
  return distance < FLAGS_distance_threshold_to_junction_exit &&
         angle_diff < FLAGS_angle_threshold_to_junction_exit;
}

}  // namespace

PerceptionObstacle::Type Obstacle::type() const { return type_; }

int Obstacle::id() const { return id_; }

double Obstacle::timestamp() const {
  CHECK(!feature_history_.empty());
  return feature_history_.front().timestamp();
}

const Feature& Obstacle::feature(const size_t i) const {
  CHECK(i < feature_history_.size());
  return feature_history_[i];
}

Feature* Obstacle::mutable_feature(const size_t i) {
  CHECK(!feature_history_.empty());
  CHECK(i < feature_history_.size());
  return &feature_history_[i];
}

const Feature& Obstacle::latest_feature() const {
  CHECK(!feature_history_.empty());
  return feature_history_.front();
}

const Feature& Obstacle::earliest_feature() const {
  CHECK(!feature_history_.empty());
  return feature_history_.back();
}

Feature* Obstacle::mutable_latest_feature() {
  CHECK(!feature_history_.empty());
  return &(feature_history_.front());
}

size_t Obstacle::history_size() const { return feature_history_.size(); }

const KalmanFilter<double, 6, 2, 0>& Obstacle::kf_motion_tracker() const {
  return kf_motion_tracker_;
}

const KalmanFilter<double, 2, 2, 4>& Obstacle::kf_pedestrian_tracker() const {
  return kf_pedestrian_tracker_;
}

bool Obstacle::IsStill() {
  if (feature_history_.size() > 0) {
    return feature_history_.front().is_still();
  }
  return true;
}

bool Obstacle::IsSlow() {
  const Feature& feature = latest_feature();
  return feature.speed() < FLAGS_slow_obstacle_speed_threshold;
}

bool Obstacle::IsOnLane() const {
  if (feature_history_.empty() ||
      !latest_feature().has_lane() ||
      latest_feature().lane().current_lane_feature().empty()) {
    return false;
  }
  for (const auto& curr_lane :
       latest_feature().lane().current_lane_feature()) {
    if (curr_lane.lane_type() != hdmap::Lane::CITY_DRIVING) {
      return false;
    }
  }

  ADEBUG << "Obstacle [" << id_ << "] is on lane.";
  return true;
}

bool Obstacle::ToIgnore() {
  if (feature_history_.empty()) {
    return true;
  }
  return latest_feature().priority().priority() == ObstaclePriority::IGNORE;
}

bool Obstacle::IsNearJunction() {
  if (feature_history_.empty()) {
    return false;
  }
  double pos_x = latest_feature().position().x();
  double pos_y = latest_feature().position().y();
  return PredictionMap::NearJunction({pos_x, pos_y},
                                     FLAGS_junction_search_radius);
}

bool Obstacle::Insert(const PerceptionObstacle& perception_obstacle,
                      const double timestamp,
                      const int prediction_obstacle_id) {
  if (!perception_obstacle.has_id() || !perception_obstacle.has_type()) {
    AERROR << "Perception obstacle has incomplete information; "
              "skip insertion";
    return false;
  }

  if (ReceivedNewerMessage(timestamp)) {
    AERROR << "Obstacle [" << id_ << "] received an older frame ["
           << std::setprecision(20) << timestamp
           << "] than the most recent timestamp [ " << this->timestamp()
           << "].";
    return false;
  }

  // Set ID, Type, and Status of the feature.
  Feature feature;
  if (!SetId(perception_obstacle, &feature, prediction_obstacle_id)) {
    return false;
  }

  SetType(perception_obstacle, &feature);

  SetStatus(perception_obstacle, timestamp, &feature);

  // Set obstacle observation for KF tracking
  if (!FLAGS_use_navigation_mode) {
    // Update KF
    if (!kf_motion_tracker_.IsInitialized()) {
      InitKFMotionTracker(feature);
    }
    UpdateKFMotionTracker(feature);
    if (type_ == PerceptionObstacle::PEDESTRIAN) {
      if (!kf_pedestrian_tracker_.IsInitialized()) {
        InitKFPedestrianTracker(feature);
      }
      UpdateKFPedestrianTracker(feature);
    }
    // Update obstacle status based on KF if enabled
    if (FLAGS_enable_kf_tracking) {
      UpdateStatus(&feature);
    }
  }

  // Set obstacle lane features
  if (type_ != PerceptionObstacle::PEDESTRIAN) {
    SetCurrentLanes(&feature);
    SetNearbyLanes(&feature);
  }

  if (FLAGS_adjust_vehicle_heading_by_lane &&
      type_ == PerceptionObstacle::VEHICLE) {
    AdjustHeadingByLane(&feature);
  }

  // Insert obstacle feature to history
  InsertFeatureToHistory(feature);

  // Set obstacle motion status
  if (FLAGS_use_navigation_mode) {
    SetMotionStatusBySpeed();
  } else {
    SetMotionStatus();
  }

  // Trim historical features
  DiscardOutdatedHistory();
  return true;
}

bool Obstacle::InsertFeature(const Feature& feature) {
  InsertFeatureToHistory(feature);
  type_ = feature.type();
  id_ = feature.id();
  return true;
}

bool Obstacle::IsInJunction(const std::string& junction_id) {
  // TODO(all) Consider if need to use vehicle front rather than position
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
    return false;
  }
  std::shared_ptr<const JunctionInfo> junction_info_ptr =
      PredictionMap::JunctionById(junction_id);
  const auto& position = latest_feature().position();
  return PredictionMap::IsPointInJunction(position.x(), position.y(),
                                          junction_info_ptr);
}

void Obstacle::BuildJunctionFeature() {
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
    return;
  }
  const std::string& junction_id = JunctionAnalyzer::GetJunctionId();
  if (!IsInJunction(junction_id)) {
    ADEBUG << "Obstacle [" << id_ << "] is not in junction [" << junction_id
           << "]";
    return;
  }
  Feature* latest_feature_ptr = mutable_latest_feature();
  if (feature_history_.size() == 1) {
    SetJunctionFeatureWithoutEnterLane(latest_feature_ptr);
    return;
  }
  const Feature& prev_feature = feature(1);
  if (prev_feature.junction_feature().has_enter_lane()) {
    CHECK(prev_feature.junction_feature().enter_lane().has_lane_id());
    std::string enter_lane_id =
        prev_feature.junction_feature().enter_lane().lane_id();
    // TODO(all) use enter lane when tracking is better
    SetJunctionFeatureWithoutEnterLane(latest_feature_ptr);
  } else {
    SetJunctionFeatureWithoutEnterLane(latest_feature_ptr);
  }
}

bool Obstacle::IsCloseToJunctionExit() {
  if (!HasJunctionFeatureWithExits()) {
    AERROR << "No junction feature found";
    return false;
  }
  CHECK_GT(history_size(), 0);
  const Feature& latest_feature = feature_history_.front();
  double position_x = latest_feature.position().x();
  double position_y = latest_feature.position().y();
  double raw_velocity_heading = std::atan2(latest_feature.raw_velocity().y(),
                                           latest_feature.raw_velocity().x());
  for (const JunctionExit& junction_exit :
       latest_feature.junction_feature().junction_exit()) {
    double exit_x = junction_exit.exit_position().x();
    double exit_y = junction_exit.exit_position().y();
    double exit_heading = junction_exit.exit_heading();
    if (IsClosed(position_x, position_y, raw_velocity_heading, exit_x, exit_y,
                 exit_heading)) {
      return true;
    }
  }
  return false;
}

void Obstacle::SetJunctionFeatureWithEnterLane(const std::string& enter_lane_id,
                                               Feature* const feature_ptr) {
  feature_ptr->mutable_junction_feature()->CopyFrom(
      JunctionAnalyzer::GetJunctionFeature(enter_lane_id));
}

void Obstacle::SetJunctionFeatureWithoutEnterLane(Feature* const feature_ptr) {
  if (!feature_ptr->has_lane()) {
    ADEBUG << "Obstacle [" << id_ << "] has no lane.";
    return;
  }
  std::vector<std::string> start_lane_ids;
  if (feature_ptr->lane().current_lane_feature_size() > 0) {
    for (const auto& lane_feature :
         feature_ptr->lane().current_lane_feature()) {
      start_lane_ids.push_back(lane_feature.lane_id());
    }
  }
  if (feature_ptr->lane().nearby_lane_feature_size() > 0) {
    for (const auto& lane_feature : feature_ptr->lane().nearby_lane_feature()) {
      start_lane_ids.push_back(lane_feature.lane_id());
    }
  }
  if (start_lane_ids.empty()) {
    ADEBUG << "Obstacle [" << id_ << "] has no lane in junction";
    return;
  }
  // TODO(kechxu) Maybe output all exits if no start lane found
  feature_ptr->mutable_junction_feature()->CopyFrom(
      JunctionAnalyzer::GetJunctionFeature(start_lane_ids));
}

void Obstacle::SetStatus(const PerceptionObstacle& perception_obstacle,
                         const double timestamp, Feature* feature) {
  SetTimestamp(perception_obstacle, timestamp, feature);
  SetPolygonPoints(perception_obstacle, feature);
  SetPosition(perception_obstacle, feature);
  SetVelocity(perception_obstacle, feature);
  SetAcceleration(feature);
  SetTheta(perception_obstacle, feature);
  SetLengthWidthHeight(perception_obstacle, feature);
  SetIsNearJunction(perception_obstacle, feature);
}

void Obstacle::UpdateStatus(Feature* feature) {
  // Update motion belief
  if (!kf_motion_tracker_.IsInitialized()) {
    ADEBUG << "Obstacle [" << id_ << "] has not initialized motion tracker.";
    return;
  }
  auto state = kf_motion_tracker_.GetStateEstimate();

  feature->mutable_t_position()->set_x(state(0, 0));
  feature->mutable_t_position()->set_y(state(1, 0));
  feature->mutable_t_position()->set_z(feature->position().z());

  double velocity_x = state(2, 0);
  double velocity_y = state(3, 0);
  double speed = std::hypot(velocity_x, velocity_y);
  double velocity_heading = std::atan2(velocity_y, velocity_x);
  if (FLAGS_adjust_velocity_by_position_shift) {
    UpdateVelocity(feature->theta(), &velocity_x, &velocity_y,
                   &velocity_heading, &speed);
  }
  feature->mutable_velocity()->set_x(velocity_x);
  feature->mutable_velocity()->set_y(velocity_y);
  feature->mutable_velocity()->set_z(velocity_heading);
  feature->set_speed(speed);
  feature->set_velocity_heading(std::atan2(state(3, 0), state(2, 0)));

  double acc_x = common::math::Clamp(state(4, 0), FLAGS_vehicle_min_linear_acc,
                                     FLAGS_vehicle_max_linear_acc);
  double acc_y = common::math::Clamp(state(5, 0), FLAGS_vehicle_min_linear_acc,
                                     FLAGS_vehicle_max_linear_acc);
  double acc =
      acc_x * std::cos(velocity_heading) + acc_y * std::sin(velocity_heading);
  feature->mutable_acceleration()->set_x(acc_x);
  feature->mutable_acceleration()->set_y(acc_y);
  feature->mutable_acceleration()->set_z(feature->acceleration().z());
  feature->set_acc(acc);

  ADEBUG << "Obstacle [" << id_ << "] has tracked position [" << std::fixed
         << std::setprecision(6) << feature->t_position().x() << ", "
         << std::fixed << std::setprecision(6) << feature->t_position().y()
         << ", " << std::fixed << std::setprecision(6)
         << feature->t_position().z() << "]";
  ADEBUG << "Obstacle [" << id_ << "] has tracked velocity [" << std::fixed
         << std::setprecision(6) << feature->velocity().x() << ", "
         << std::fixed << std::setprecision(6) << feature->velocity().y()
         << ", " << std::fixed << std::setprecision(6)
         << feature->velocity().z() << "]";
  ADEBUG << "Obstacle [" << id_ << "] has tracked acceleration [" << std::fixed
         << std::setprecision(6) << feature->acceleration().x() << ", "
         << std::fixed << std::setprecision(6) << feature->acceleration().y()
         << ", " << std::fixed << std::setprecision(6)
         << feature->acceleration().z() << "]";
  ADEBUG << "Obstacle [" << id_ << "] has tracked velocity heading ["
         << std::fixed << std::setprecision(6) << feature->velocity_heading()
         << "].";

  // Update pedestrian motion belief
  if (type_ == PerceptionObstacle::PEDESTRIAN) {
    if (!kf_pedestrian_tracker_.IsInitialized()) {
      ADEBUG << "Obstacle [" << id_
             << "] has not initialized pedestrian tracker.";
      return;
    }
    feature->mutable_t_position()->set_x(
        kf_pedestrian_tracker_.GetStateEstimate()(0, 0));
    feature->mutable_t_position()->set_y(
        kf_pedestrian_tracker_.GetStateEstimate()(1, 0));
    ADEBUG << "Obstacle [" << id_ << "] has tracked pedestrian position ["
           << std::setprecision(6) << feature->t_position().x() << std::fixed
           << ", " << std::setprecision(6) << feature->t_position().y()
           << std::fixed << ", " << std::setprecision(6)
           << feature->t_position().z() << std::fixed << "]";
  }
}

bool Obstacle::SetId(const PerceptionObstacle& perception_obstacle,
                     Feature* feature, const int prediction_obstacle_id) {
  int id = prediction_obstacle_id > 0 ? prediction_obstacle_id
                                      : perception_obstacle.id();
  if (id_ < 0) {
    id_ = id;
    ADEBUG << "Obstacle has id [" << id_ << "].";
  } else {
    if (id_ != id) {
      AERROR << "Obstacle [" << id_ << "] has a mismatched ID [" << id
             << "] from perception obstacle.";
      return false;
    }
  }
  feature->set_id(id);
  return true;
}

void Obstacle::SetType(const PerceptionObstacle& perception_obstacle,
                       Feature* feature) {
  type_ = perception_obstacle.type();
  ADEBUG << "Obstacle [" << id_ << "] has type [" << type_ << "].";
  feature->set_type(type_);
}

void Obstacle::SetTimestamp(const PerceptionObstacle& perception_obstacle,
                            const double timestamp, Feature* feature) {
  double ts = timestamp;
  if (perception_obstacle.has_timestamp() &&
      perception_obstacle.timestamp() > 0.0) {
    ts = perception_obstacle.timestamp();
  }
  feature->set_timestamp(ts);

  ADEBUG << "Obstacle [" << id_ << "] has timestamp [" << std::fixed
         << std::setprecision(6) << ts << "].";
}

void Obstacle::SetPolygonPoints(const PerceptionObstacle& perception_obstacle,
                                Feature* feature) {
  for (const auto& polygon_point : perception_obstacle.polygon_point()) {
    *feature->add_polygon_point() = polygon_point;
    ADEBUG << "Obstacle [" << id_
           << "] has new corner point:" << polygon_point.DebugString();
  }
}

void Obstacle::SetPosition(const PerceptionObstacle& perception_obstacle,
                           Feature* feature) {
  *feature->mutable_position() = perception_obstacle.position();
  ADEBUG << "Obstacle [" << id_
         << "] has position:" << perception_obstacle.position().DebugString();
}

void Obstacle::SetVelocity(const PerceptionObstacle& perception_obstacle,
                           Feature* feature) {
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double velocity_z = 0.0;

  if (perception_obstacle.has_velocity()) {
    if (perception_obstacle.velocity().has_x()) {
      velocity_x = perception_obstacle.velocity().x();
      if (std::isnan(velocity_x)) {
        AERROR << "Found nan velocity_x from perception obstacle";
        velocity_x = 0.0;
      } else if (velocity_x > 50.0 || velocity_x < -50.0) {
        AERROR << "Found unreasonable velocity_x from perception obstacle";
      }
    }
    if (perception_obstacle.velocity().has_y()) {
      velocity_y = perception_obstacle.velocity().y();
      if (std::isnan(velocity_y)) {
        AERROR << "Found nan velocity_y from perception obstacle";
        velocity_y = 0.0;
      } else if (velocity_y > 50.0 || velocity_y < -50.0) {
        AERROR << "Found unreasonable velocity_y from perception obstacle";
      }
    }
    if (perception_obstacle.velocity().has_z()) {
      velocity_z = perception_obstacle.velocity().z();
      if (std::isnan(velocity_z)) {
        AERROR << "Found nan velocity z from perception obstacle";
        velocity_z = 0.0;
      } else if (velocity_z > 50.0 || velocity_z < -50.0) {
        AERROR << "Found unreasonable velocity_z from perception obstacle";
      }
    }
  }

  feature->mutable_raw_velocity()->set_x(velocity_x);
  feature->mutable_raw_velocity()->set_y(velocity_y);
  feature->mutable_raw_velocity()->set_z(velocity_z);

  double speed = std::hypot(velocity_x, velocity_y);
  double velocity_heading = std::atan2(velocity_y, velocity_x);
  if (FLAGS_adjust_velocity_by_obstacle_heading || speed < 0.1) {
    velocity_heading = perception_obstacle.theta();
  }

  if (!FLAGS_use_navigation_mode && FLAGS_adjust_velocity_by_position_shift &&
      history_size() > 0) {
    double diff_x =
        feature->position().x() - feature_history_.front().position().x();
    double diff_y =
        feature->position().y() - feature_history_.front().position().y();
    double prev_obstacle_size = std::fmax(feature_history_.front().length(),
                                          feature_history_.front().width());
    double obstacle_size =
        std::fmax(perception_obstacle.length(), perception_obstacle.width());
    double size_diff = std::abs(obstacle_size - prev_obstacle_size);
    double shift_thred =
        std::fmax(obstacle_size * FLAGS_valid_position_diff_rate_threshold,
                  FLAGS_valid_position_diff_threshold);
    double size_diff_thred =
        FLAGS_split_rate * std::min(obstacle_size, prev_obstacle_size);
    if (std::fabs(diff_x) > shift_thred && std::fabs(diff_y) > shift_thred &&
        size_diff < size_diff_thred) {
      double shift_heading = std::atan2(diff_y, diff_x);
      double angle_diff =
          common::math::NormalizeAngle(shift_heading - velocity_heading);
      if (std::fabs(angle_diff) > FLAGS_max_lane_angle_diff) {
        ADEBUG << "Shift velocity heading to be " << shift_heading;
        velocity_heading = shift_heading;
      }
    }
    velocity_x = speed * std::cos(velocity_heading);
    velocity_y = speed * std::sin(velocity_heading);
  }

  feature->mutable_velocity()->set_x(velocity_x);
  feature->mutable_velocity()->set_y(velocity_y);
  feature->mutable_velocity()->set_z(velocity_z);
  feature->set_velocity_heading(velocity_heading);
  feature->set_speed(speed);

  ADEBUG << "Obstacle [" << id_ << "] has velocity [" << std::fixed
         << std::setprecision(6) << velocity_x << ", " << std::fixed
         << std::setprecision(6) << velocity_y << ", " << std::fixed
         << std::setprecision(6) << velocity_z << "]";
  ADEBUG << "Obstacle [" << id_ << "] has velocity heading [" << std::fixed
         << std::setprecision(6) << velocity_heading << "] ";
  ADEBUG << "Obstacle [" << id_ << "] has speed [" << std::fixed
         << std::setprecision(6) << speed << "].";
}

void Obstacle::AdjustHeadingByLane(Feature* feature) {
  if (!feature->has_lane() || !feature->lane().has_lane_feature()) {
    return;
  }
  double velocity_heading = feature->velocity_heading();
  double lane_heading = feature->lane().lane_feature().lane_heading();
  double angle_diff = feature->lane().lane_feature().angle_diff();
  if (std::abs(angle_diff) < FLAGS_max_angle_diff_to_adjust_velocity) {
    velocity_heading = lane_heading;
    double speed = feature->speed();
    feature->mutable_velocity()->set_x(speed * std::cos(velocity_heading));
    feature->mutable_velocity()->set_y(speed * std::sin(velocity_heading));
    feature->set_velocity_heading(velocity_heading);
  }
}

void Obstacle::UpdateVelocity(const double theta, double* velocity_x,
                              double* velocity_y, double* velocity_heading,
                              double* speed) {
  *speed = std::hypot(*velocity_x, *velocity_y);
  double angle_diff = common::math::NormalizeAngle(*velocity_heading - theta);
  if (std::fabs(angle_diff) <= FLAGS_max_lane_angle_diff) {
    *velocity_heading = theta;
    *velocity_x = *speed * std::cos(*velocity_heading);
    *velocity_y = *speed * std::sin(*velocity_heading);
  }
}

void Obstacle::SetAcceleration(Feature* feature) {
  double acc_x = 0.0;
  double acc_y = 0.0;
  double acc_z = 0.0;
  double acc = 0.0;

  if (feature_history_.size() > 0) {
    double curr_ts = feature->timestamp();
    double prev_ts = feature_history_.front().timestamp();

    const Point3D& curr_velocity = feature->velocity();
    const Point3D& prev_velocity = feature_history_.front().velocity();

    if (curr_ts > prev_ts) {
      /*
       * A damp function is to punish acc calculation for low speed
       * and reward it for high speed
       */
      double damping_x = Damp(curr_velocity.x(), 0.001);
      double damping_y = Damp(curr_velocity.y(), 0.001);
      double damping_z = Damp(curr_velocity.z(), 0.001);

      acc_x = (curr_velocity.x() - prev_velocity.x()) / (curr_ts - prev_ts);
      acc_y = (curr_velocity.y() - prev_velocity.y()) / (curr_ts - prev_ts);
      acc_z = (curr_velocity.z() - prev_velocity.z()) / (curr_ts - prev_ts);

      acc_x *= damping_x;
      acc_y *= damping_y;
      acc_z *= damping_z;

      acc_x =
          common::math::Clamp(acc_x * damping_x, FLAGS_vehicle_min_linear_acc,
                              FLAGS_vehicle_max_linear_acc);
      acc_y =
          common::math::Clamp(acc_y * damping_y, FLAGS_vehicle_min_linear_acc,
                              FLAGS_vehicle_max_linear_acc);
      acc_z =
          common::math::Clamp(acc_z * damping_z, FLAGS_vehicle_min_linear_acc,
                              FLAGS_vehicle_max_linear_acc);

      double heading = feature->velocity_heading();
      acc = acc_x * std::cos(heading) + acc_y * std::sin(heading);
    }
  }

  feature->mutable_acceleration()->set_x(acc_x);
  feature->mutable_acceleration()->set_y(acc_y);
  feature->mutable_acceleration()->set_z(acc_z);
  feature->set_acc(acc);

  ADEBUG << "Obstacle [" << id_ << "] has acceleration [" << std::fixed
         << std::setprecision(6) << acc_x << ", " << std::fixed
         << std::setprecision(6) << acc_y << ", " << std::fixed
         << std::setprecision(6) << acc_z << "]";
  ADEBUG << "Obstacle [" << id_ << "] has acceleration value [" << std::fixed
         << std::setprecision(6) << acc << "].";
}

void Obstacle::SetTheta(const PerceptionObstacle& perception_obstacle,
                        Feature* feature) {
  double theta = 0.0;
  if (perception_obstacle.has_theta()) {
    theta = perception_obstacle.theta();
  }
  feature->set_theta(theta);

  ADEBUG << "Obstacle [" << id_ << "] has theta [" << std::fixed
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

  ADEBUG << "Obstacle [" << id_ << "] has dimension [" << std::fixed
         << std::setprecision(6) << length << ", " << std::fixed
         << std::setprecision(6) << width << ", " << std::fixed
         << std::setprecision(6) << height << "].";
}

void Obstacle::SetIsNearJunction(const PerceptionObstacle& perception_obstacle,
                                 Feature* feature) {
  if (!perception_obstacle.has_position()) {
    return;
  }
  if (!perception_obstacle.position().has_x() ||
      !perception_obstacle.position().has_y()) {
    return;
  }
  double x = perception_obstacle.position().x();
  double y = perception_obstacle.position().y();
  bool is_near_junction =
      PredictionMap::NearJunction({x, y}, FLAGS_junction_search_radius);
  feature->set_is_near_junction(is_near_junction);
}

bool Obstacle::HasJunctionFeatureWithExits() {
  if (history_size() == 0) {
    return false;
  }
  return latest_feature().has_junction_feature() &&
         latest_feature().junction_feature().junction_exit_size() > 0;
}

void Obstacle::InitKFMotionTracker(const Feature& feature) {
  // Set transition matrix F
  // constant acceleration dynamic model
  Eigen::Matrix<double, 6, 6> F;
  F.setIdentity();
  kf_motion_tracker_.SetTransitionMatrix(F);

  // Set observation matrix H
  Eigen::Matrix<double, 2, 6> H;
  H.setIdentity();
  kf_motion_tracker_.SetObservationMatrix(H);

  // Set covariance of transition noise matrix Q
  // make the noise this order:
  // noise(x/y) < noise(vx/vy) < noise(ax/ay)
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
  // make the covariance this order:
  // cov(x/y) < cov(vx/vy) < cov(ax/ay)
  Eigen::Matrix<double, 6, 6> P;
  P.setIdentity();
  P *= FLAGS_p_var;

  // Set initial state
  Eigen::Matrix<double, 6, 1> x;
  x(0, 0) = feature.position().x();
  x(1, 0) = feature.position().y();
  x(2, 0) = feature.velocity().x();
  x(3, 0) = feature.velocity().y();
  x(4, 0) = feature.acceleration().x();
  x(5, 0) = feature.acceleration().y();

  kf_motion_tracker_.SetStateEstimate(x, P);
}

void Obstacle::UpdateKFMotionTracker(const Feature& feature) {
  double delta_ts = 0.0;
  if (feature_history_.size() > 0) {
    delta_ts = feature.timestamp() - feature_history_.front().timestamp();
  }
  if (delta_ts > FLAGS_double_precision) {
    // Set transition matrix and predict
    auto F = kf_motion_tracker_.GetTransitionMatrix();
    F(0, 2) = delta_ts;
    F(0, 4) = 0.5 * delta_ts * delta_ts;
    F(1, 3) = delta_ts;
    F(1, 5) = 0.5 * delta_ts * delta_ts;
    F(2, 4) = delta_ts;
    F(3, 5) = delta_ts;
    kf_motion_tracker_.SetTransitionMatrix(F);
    kf_motion_tracker_.Predict();

    // Set observation and correct
    Eigen::Matrix<double, 2, 1> z;
    z(0, 0) = feature.position().x();
    z(1, 0) = feature.position().y();
    kf_motion_tracker_.Correct(z);
  }
}

void Obstacle::InitKFPedestrianTracker(const Feature& feature) {
  // Set transition matrix F
  Eigen::Matrix<double, 2, 2> F;
  F.setIdentity();
  kf_pedestrian_tracker_.SetTransitionMatrix(F);

  // Set observation matrix H
  Eigen::Matrix<double, 2, 2> H;
  H.setIdentity();
  kf_pedestrian_tracker_.SetObservationMatrix(H);

  // Set control matrix
  Eigen::Matrix<double, 2, 4> B;
  B.setZero();
  kf_pedestrian_tracker_.SetControlMatrix(B);

  // Set covariance of transition noise matrix Q
  Eigen::Matrix<double, 2, 2> Q;
  Q.setIdentity();
  Q *= FLAGS_q_var;
  kf_pedestrian_tracker_.SetTransitionNoise(Q);

  // Set observation noise matrix R
  Eigen::Matrix<double, 2, 2> R;
  R.setIdentity();
  R *= FLAGS_r_var;
  kf_pedestrian_tracker_.SetObservationNoise(R);

  // Set current state covariance matrix P
  Eigen::Matrix<double, 2, 2> P;
  P.setIdentity();
  P *= FLAGS_p_var;

  // Set initial state
  Eigen::Matrix<double, 2, 1> x;
  x.setZero();
  x(0, 0) = feature.position().x();
  x(1, 0) = feature.position().y();

  kf_pedestrian_tracker_.SetStateEstimate(x, P);
}

void Obstacle::UpdateKFPedestrianTracker(const Feature& feature) {
  double delta_ts = 0.0;
  if (!feature_history_.empty()) {
    delta_ts = feature.timestamp() - feature_history_.front().timestamp();
  }
  if (delta_ts > std::numeric_limits<double>::epsilon()) {
    Eigen::Matrix<double, 2, 4> B = kf_pedestrian_tracker_.GetControlMatrix();
    B(0, 0) = delta_ts;
    B(0, 2) = 0.5 * delta_ts * delta_ts;
    B(1, 1) = delta_ts;
    B(1, 3) = 0.5 * delta_ts * delta_ts;
    kf_pedestrian_tracker_.SetControlMatrix(B);

    // Set control vector
    Eigen::Matrix<double, 4, 1> u;
    u(0, 0) = feature.velocity().x();
    u(1, 0) = feature.velocity().y();
    u(2, 0) = 0.0;
    u(3, 0) = 0.0;

    kf_pedestrian_tracker_.Predict(u);

    // Set observation vector
    Eigen::Matrix<double, 2, 1> z;
    z(0, 0) = feature.position().x();
    z(1, 0) = feature.position().y();
    kf_pedestrian_tracker_.Correct(z);
  }
}

void Obstacle::SetCurrentLanes(Feature* feature) {
  Eigen::Vector2d point(feature->position().x(), feature->position().y());
  double heading = feature->velocity_heading();
  int max_num_lane = FLAGS_max_num_current_lane;
  double max_angle_diff = FLAGS_max_lane_angle_diff;
  double lane_search_radius = FLAGS_lane_search_radius;
  if (PredictionMap::InJunction(point, FLAGS_junction_search_radius)) {
    max_num_lane = FLAGS_max_num_current_lane_in_junction;
    max_angle_diff = FLAGS_max_lane_angle_diff_in_junction;
    lane_search_radius = FLAGS_lane_search_radius_in_junction;
  }
  std::vector<std::shared_ptr<const LaneInfo>> current_lanes;
  PredictionMap::OnLane(current_lanes_, point, heading, lane_search_radius,
                        true, max_num_lane, max_angle_diff, &current_lanes);
  current_lanes_ = current_lanes;
  if (current_lanes_.empty()) {
    ADEBUG << "Obstacle [" << id_ << "] has no current lanes.";
    return;
  }
  Lane lane;
  if (feature->has_lane()) {
    lane = feature->lane();
  }
  double min_heading_diff = std::numeric_limits<double>::infinity();
  for (std::shared_ptr<const LaneInfo> current_lane : current_lanes) {
    if (current_lane == nullptr) {
      continue;
    }

    int turn_type = PredictionMap::LaneTurnType(current_lane->id().id());
    std::string lane_id = current_lane->id().id();
    double s = 0.0;
    double l = 0.0;
    PredictionMap::GetProjection(point, current_lane, &s, &l);
    if (s < 0.0) {
      continue;
    }

    common::math::Vec2d vec_point(point[0], point[1]);
    double distance = 0.0;
    common::PointENU nearest_point =
        current_lane->GetNearestPoint(vec_point, &distance);
    double nearest_point_heading =
        PredictionMap::PathHeading(current_lane, nearest_point);
    double angle_diff = common::math::AngleDiff(heading, nearest_point_heading);
    double left = 0.0;
    double right = 0.0;
    current_lane->GetWidth(s, &left, &right);
    LaneFeature* lane_feature = lane.add_current_lane_feature();
    lane_feature->set_lane_turn_type(turn_type);
    lane_feature->set_lane_id(lane_id);
    lane_feature->set_lane_s(s);
    lane_feature->set_lane_l(l);
    lane_feature->set_angle_diff(angle_diff);
    lane_feature->set_lane_heading(nearest_point_heading);
    lane_feature->set_dist_to_left_boundary(left - l);
    lane_feature->set_dist_to_right_boundary(right + l);
    lane_feature->set_lane_type(current_lane->lane().type());
    if (std::fabs(angle_diff) < min_heading_diff) {
      lane.mutable_lane_feature()->CopyFrom(*lane_feature);
      min_heading_diff = std::fabs(angle_diff);
    }
    ObstacleClusters::AddObstacle(id_, lane_id, s, l);
    ADEBUG << "Obstacle [" << id_ << "] has current lanes ["
           << lane_feature->ShortDebugString() << "].";
  }

  if (lane.has_lane_feature()) {
    ADEBUG << "Obstacle [" << id_ << "] has one current lane ["
           << lane.lane_feature().ShortDebugString() << "].";
  }

  feature->mutable_lane()->CopyFrom(lane);
}

void Obstacle::SetNearbyLanes(Feature* feature) {
  Eigen::Vector2d point(feature->position().x(), feature->position().y());
  int max_num_lane = FLAGS_max_num_nearby_lane;
  double lane_search_radius = FLAGS_lane_search_radius;
  if (PredictionMap::InJunction(point, FLAGS_junction_search_radius)) {
    max_num_lane = FLAGS_max_num_nearby_lane_in_junction;
    lane_search_radius = FLAGS_lane_search_radius_in_junction;
  }
  double theta = feature->velocity_heading();
  std::vector<std::shared_ptr<const LaneInfo>> nearby_lanes;
  PredictionMap::NearbyLanesByCurrentLanes(point, theta, lane_search_radius,
                                           current_lanes_, max_num_lane,
                                           &nearby_lanes);
  if (nearby_lanes.empty()) {
    ADEBUG << "Obstacle [" << id_ << "] has no nearby lanes.";
    return;
  }

  for (std::shared_ptr<const LaneInfo> nearby_lane : nearby_lanes) {
    if (nearby_lane == nullptr) {
      continue;
    }

    // Ignore bike and sidewalk lanes for vehicles
    if (type_ == PerceptionObstacle::VEHICLE &&
        nearby_lane->lane().has_type() &&
        (nearby_lane->lane().type() == ::apollo::hdmap::Lane::BIKING ||
         nearby_lane->lane().type() == ::apollo::hdmap::Lane::SIDEWALK)) {
      ADEBUG << "Obstacle [" << id_ << "] ignores disqualified lanes.";
      continue;
    }

    double s = -1.0;
    double l = 0.0;
    PredictionMap::GetProjection(point, nearby_lane, &s, &l);
    if (s < 0.0 || s >= nearby_lane->total_length()) {
      continue;
    }
    int turn_type = PredictionMap::LaneTurnType(nearby_lane->id().id());
    double heading = feature->velocity_heading();
    double angle_diff = 0.0;
    hdmap::MapPathPoint nearest_point;
    if (!PredictionMap::ProjectionFromLane(nearby_lane, s, &nearest_point)) {
      angle_diff = common::math::AngleDiff(nearest_point.heading(), heading);
    }

    double left = 0.0;
    double right = 0.0;
    nearby_lane->GetWidth(s, &left, &right);

    LaneFeature* lane_feature =
        feature->mutable_lane()->add_nearby_lane_feature();

    lane_feature->set_lane_turn_type(turn_type);
    lane_feature->set_lane_id(nearby_lane->id().id());
    lane_feature->set_lane_s(s);
    lane_feature->set_lane_l(l);
    lane_feature->set_angle_diff(angle_diff);
    lane_feature->set_dist_to_left_boundary(left - l);
    lane_feature->set_dist_to_right_boundary(right + l);
    lane_feature->set_lane_type(nearby_lane->lane().type());
    ADEBUG << "Obstacle [" << id_ << "] has nearby lanes ["
           << lane_feature->ShortDebugString() << "]";
  }
}

void Obstacle::BuildLaneGraph() {
  // Sanity checks.
  if (history_size() == 0) {
    AERROR << "No feature found.";
    return;
  }

  Feature* feature = mutable_latest_feature();
  // No need to BuildLaneGraph for those non-moving obstacles.
  if (feature->is_still()) {
    ADEBUG << "Not build lane graph for still obstacle";
    return;
  }
  if (feature->lane().lane_graph().lane_sequence_size() > 0) {
    ADEBUG << "Not build lane graph for an old obstacle";
    return;
  }
  double speed = feature->speed();
  double t_max = FLAGS_prediction_trajectory_time_length;
  auto estimated_move_distance = speed * t_max;

  double road_graph_search_distance = std::fmax(
      estimated_move_distance, FLAGS_min_prediction_trajectory_spatial_length);

  // BuildLaneGraph for current lanes:
  // Go through all the LaneSegments in current_lane,
  // construct up to max_num_current_lane of them.
  int seq_id = 0;
  int curr_lane_count = 0;
  for (auto& lane : feature->lane().current_lane_feature()) {
    std::shared_ptr<const LaneInfo> lane_info =
        PredictionMap::LaneById(lane.lane_id());
    const LaneGraph& lane_graph = ObstacleClusters::GetLaneGraph(
        lane.lane_s(), road_graph_search_distance, true, lane_info);
    if (lane_graph.lane_sequence_size() > 0) {
      ++curr_lane_count;
    }
    for (const auto& lane_seq : lane_graph.lane_sequence()) {
      LaneSequence* lane_seq_ptr =
          feature->mutable_lane()->mutable_lane_graph()->add_lane_sequence();
      lane_seq_ptr->CopyFrom(lane_seq);
      lane_seq_ptr->set_lane_sequence_id(seq_id++);
      lane_seq_ptr->set_lane_s(lane.lane_s());
      lane_seq_ptr->set_lane_l(lane.lane_l());
      lane_seq_ptr->set_vehicle_on_lane(true);
      lane_seq_ptr->set_lane_type(lane.lane_type());
      SetLaneSequenceStopSign(lane_seq_ptr);
      ADEBUG << "Obstacle [" << id_ << "] set a lane sequence ["
             << lane_seq.ShortDebugString() << "].";
    }
    if (curr_lane_count >= FLAGS_max_num_current_lane) {
      break;
    }
  }

  // BuildLaneGraph for neighbor lanes.
  int nearby_lane_count = 0;
  for (auto& lane : feature->lane().nearby_lane_feature()) {
    std::shared_ptr<const LaneInfo> lane_info =
        PredictionMap::LaneById(lane.lane_id());
    const LaneGraph& lane_graph = ObstacleClusters::GetLaneGraph(
        lane.lane_s(), road_graph_search_distance, false, lane_info);
    if (lane_graph.lane_sequence_size() > 0) {
      ++nearby_lane_count;
    }
    for (const auto& lane_seq : lane_graph.lane_sequence()) {
      LaneSequence* lane_seq_ptr =
          feature->mutable_lane()->mutable_lane_graph()->add_lane_sequence();
      lane_seq_ptr->CopyFrom(lane_seq);
      lane_seq_ptr->set_lane_sequence_id(seq_id++);
      lane_seq_ptr->set_lane_s(lane.lane_s());
      lane_seq_ptr->set_lane_l(lane.lane_l());
      lane_seq_ptr->set_vehicle_on_lane(false);
      lane_seq_ptr->set_lane_type(lane.lane_type());
      SetLaneSequenceStopSign(lane_seq_ptr);
      ADEBUG << "Obstacle [" << id_ << "] set a lane sequence ["
             << lane_seq.ShortDebugString() << "].";
    }
    if (nearby_lane_count >= FLAGS_max_num_nearby_lane) {
      break;
    }
  }

  if (feature->has_lane() && feature->lane().has_lane_graph()) {
    SetLanePoints(feature);
    SetLaneSequencePath(feature->mutable_lane()->mutable_lane_graph());
  }
  ADEBUG << "Obstacle [" << id_ << "] set lane graph features.";
}

void Obstacle::SetLaneSequenceStopSign(LaneSequence* lane_sequence_ptr) {
  // Set the nearest stop sign along the lane sequence
  if (lane_sequence_ptr->lane_segment_size() <= 0) {
    return;
  }
  double accumulate_s = 0.0;
  for (const LaneSegment& lane_segment : lane_sequence_ptr->lane_segment()) {
    const StopSign& stop_sign =
        ObstacleClusters::QueryStopSignByLaneId(lane_segment.lane_id());
    if (stop_sign.has_stop_sign_id() &&
        stop_sign.lane_s() + accumulate_s > lane_sequence_ptr->lane_s()) {
      lane_sequence_ptr->mutable_stop_sign()->CopyFrom(stop_sign);
      lane_sequence_ptr->mutable_stop_sign()->set_lane_sequence_s(
          stop_sign.lane_s() + accumulate_s);
      ADEBUG << "Set StopSign for LaneSequence ["
             << lane_sequence_ptr->lane_sequence_id() << "].";
      break;
    }
    accumulate_s += lane_segment.total_length();
  }
}

void Obstacle::GetNeighborLaneSegments(
    std::shared_ptr<const LaneInfo> center_lane_info, bool is_left,
    int recursion_depth, std::list<std::string>* const lane_ids_ordered,
    std::unordered_set<std::string>* const existing_lane_ids) {
  // Exit recursion if reached max num of allowed search depth.
  if (recursion_depth <= 0) {
    return;
  }
  if (is_left) {
    std::vector<std::string> curr_left_lane_ids;
    for (const auto& left_lane_id :
         center_lane_info->lane().left_neighbor_forward_lane_id()) {
      if (left_lane_id.has_id()) {
        const std::string& lane_id = left_lane_id.id();
        // If haven't seen this lane id before.
        if (existing_lane_ids->count(lane_id) == 0) {
          existing_lane_ids->insert(lane_id);
          lane_ids_ordered->push_front(lane_id);
          curr_left_lane_ids.push_back(lane_id);
        }
      }
    }

    for (const std::string& lane_id : curr_left_lane_ids) {
      GetNeighborLaneSegments(PredictionMap::LaneById(lane_id), true,
                              recursion_depth - 1, lane_ids_ordered,
                              existing_lane_ids);
    }
  } else {
    std::vector<std::string> curr_right_lane_ids;
    for (const auto& right_lane_id :
         center_lane_info->lane().right_neighbor_forward_lane_id()) {
      if (right_lane_id.has_id()) {
        const std::string& lane_id = right_lane_id.id();
        // If haven't seen this lane id before.
        if (existing_lane_ids->count(lane_id) == 0) {
          existing_lane_ids->insert(lane_id);
          lane_ids_ordered->push_back(lane_id);
          curr_right_lane_ids.push_back(lane_id);
        }
      }
    }

    for (const std::string& lane_id : curr_right_lane_ids) {
      GetNeighborLaneSegments(PredictionMap::LaneById(lane_id), false,
                              recursion_depth - 1, lane_ids_ordered,
                              existing_lane_ids);
    }
  }
}

void Obstacle::BuildLaneGraphFromLeftToRight() {
  // Sanity checks.
  if (history_size() == 0) {
    AERROR << "No feature found.";
    return;
  }

  // No need to BuildLaneGraph for those non-moving obstacles.
  Feature* feature = mutable_latest_feature();
  if (feature->is_still()) {
    ADEBUG << "Don't build lane graph for non-moving obstacle.";
    return;
  }
  if (feature->lane().lane_graph_ordered().lane_sequence_size() > 0) {
    ADEBUG << "Don't build lane graph for an old obstacle.";
    return;
  }
  // double speed = feature->speed();
  double road_graph_search_distance = 50.0 * 0.95;  // (45mph for 3sec)
  // std::fmax(speed * FLAGS_prediction_trajectory_time_length +
  //               0.5 * FLAGS_vehicle_max_linear_acc *
  //               FLAGS_prediction_trajectory_time_length *
  //               FLAGS_prediction_trajectory_time_length,
  //           FLAGS_min_prediction_trajectory_spatial_length);

  // Treat the most probable lane_segment as the center, put its left
  // and right neighbors into a vector following the left-to-right order.
  if (!feature->has_lane() || !feature->lane().has_lane_feature()) {
    return;
  }
  std::shared_ptr<const LaneInfo> center_lane_info =
      PredictionMap::LaneById(feature->lane().lane_feature().lane_id());
  std::list<std::string> lane_ids_ordered_list;
  std::unordered_set<std::string> existing_lane_ids;
  GetNeighborLaneSegments(center_lane_info, true, 2, &lane_ids_ordered_list,
                          &existing_lane_ids);
  lane_ids_ordered_list.push_back(feature->lane().lane_feature().lane_id());
  existing_lane_ids.insert(feature->lane().lane_feature().lane_id());
  GetNeighborLaneSegments(center_lane_info, false, 2, &lane_ids_ordered_list,
                          &existing_lane_ids);

  const std::vector<std::string> lane_ids_ordered(lane_ids_ordered_list.begin(),
                                                  lane_ids_ordered_list.end());
  // TODO(all): sort the lane_segments from left to right (again)
  //            to double-check and make sure it's well sorted.
  // Build lane_graph for every lane_segment and update it into proto.
  int seq_id = 0;
  for (const std::string& lane_id : lane_ids_ordered) {
    // Construct the local lane_graph based on the current lane_segment.
    bool vehicle_is_on_lane = (lane_id == center_lane_info->lane().id().id());
    std::shared_ptr<const LaneInfo> curr_lane_info =
        PredictionMap::LaneById(lane_id);
    const LaneGraph& local_lane_graph =
        ObstacleClusters::GetLaneGraphWithoutMemorizing(
            feature->lane().lane_feature().lane_s(), road_graph_search_distance,
            true, curr_lane_info);
    // Update it into the Feature proto
    for (const auto& lane_seq : local_lane_graph.lane_sequence()) {
      LaneSequence* lane_seq_ptr = feature->mutable_lane()
                                       ->mutable_lane_graph_ordered()
                                       ->add_lane_sequence();
      lane_seq_ptr->CopyFrom(lane_seq);
      lane_seq_ptr->set_lane_sequence_id(seq_id++);
      lane_seq_ptr->set_lane_s(feature->lane().lane_feature().lane_s());
      lane_seq_ptr->set_lane_l(feature->lane().lane_feature().lane_l());
      lane_seq_ptr->set_vehicle_on_lane(vehicle_is_on_lane);
      ADEBUG << "Obstacle [" << id_ << "] set a lane sequence ["
             << lane_seq.ShortDebugString() << "].";
    }
  }

  // Build lane_points.
  if (feature->lane().has_lane_graph_ordered()) {
    SetLanePoints(feature, 0.5, 100, true,
                  feature->mutable_lane()->mutable_lane_graph_ordered());
    SetLaneSequencePath(feature->mutable_lane()->mutable_lane_graph_ordered());
  }
  ADEBUG << "Obstacle [" << id_ << "] set lane graph features.";
}

// The default SetLanePoints applies to lane_graph with
// FLAGS_target_lane_gap.
void Obstacle::SetLanePoints(Feature* feature) {
  LaneGraph* lane_graph = feature->mutable_lane()->mutable_lane_graph();
  SetLanePoints(feature, FLAGS_target_lane_gap, FLAGS_max_num_lane_point, false,
                lane_graph);
}

// The generic SetLanePoints
void Obstacle::SetLanePoints(const Feature* feature,
                             const double lane_point_spacing,
                             const uint64_t max_num_lane_point,
                             const bool is_bidirection,
                             LaneGraph* const lane_graph) {
  ADEBUG << "Spacing = " << lane_point_spacing;
  // Sanity checks.
  if (feature == nullptr || !feature->has_velocity_heading()) {
    AERROR << "Null feature or no velocity heading.";
    return;
  }
  double heading = feature->velocity_heading();
  double x = feature->position().x();
  double y = feature->position().y();
  Eigen::Vector2d position(x, y);

  // Go through every lane_sequence.
  for (int i = 0; i < lane_graph->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence = lane_graph->mutable_lane_sequence(i);
    if (lane_sequence->lane_segment_size() <= 0) {
      continue;
    }
    // TODO(jiacheng): can refactor the following two parts into one to
    //                 make it more elegant.

    // If building bidirectionally, then build backward lane-points as well.
    if (is_bidirection) {
      int lane_index = 0;
      double lane_seg_s = lane_sequence->lane_segment(lane_index).start_s();
      while (lane_index < lane_sequence->lane_segment_size()) {
        // Go through lane_segment one by one sequentially.
        LaneSegment* lane_segment =
            lane_sequence->mutable_lane_segment(lane_index);

        // End-condition: reached the current ADC's location.
        if (lane_index == lane_sequence->adc_lane_segment_idx() &&
            lane_seg_s > lane_segment->adc_s()) {
          lane_segment->set_adc_lane_point_idx(lane_segment->lane_point_size());
          break;
        }

        if (lane_seg_s > lane_segment->end_s()) {
          // If already exceeds the current lane_segment, then go to the
          // next following one.
          ADEBUG << "Move on to the next lane-segment.";
          lane_seg_s = lane_seg_s - lane_segment->end_s();
          ++lane_index;
        } else {
          // Otherwise, update lane_graph:
          // 1. Sanity checks.
          std::string lane_id = lane_segment->lane_id();
          lane_segment->set_lane_turn_type(
              PredictionMap::LaneTurnType(lane_id));
          ADEBUG << "Currently on " << lane_id;
          auto lane_info = PredictionMap::LaneById(lane_id);
          if (lane_info == nullptr) {
            break;
          }
          // 2. Get the closeset lane_point
          ADEBUG << "Lane-segment s = " << lane_seg_s;
          Eigen::Vector2d lane_point_pos =
              PredictionMap::PositionOnLane(lane_info, lane_seg_s);
          double lane_point_heading =
              PredictionMap::HeadingOnLane(lane_info, lane_seg_s);
          double lane_point_width =
              PredictionMap::LaneTotalWidth(lane_info, lane_seg_s);
          double lane_point_angle_diff =
              common::math::AngleDiff(lane_point_heading, heading);
          // 3. Update it into the lane_graph
          ADEBUG << lane_point_pos[0] << "    " << lane_point_pos[1];
          LanePoint lane_point;
          lane_point.mutable_position()->set_x(lane_point_pos[0]);
          lane_point.mutable_position()->set_y(lane_point_pos[1]);
          lane_point.set_heading(lane_point_heading);
          lane_point.set_width(lane_point_width);
          lane_point.set_angle_diff(lane_point_angle_diff);
          // Update into lane_segment.
          lane_segment->add_lane_point()->CopyFrom(lane_point);
          lane_seg_s += lane_point_spacing;
        }
      }
    }

    // Build lane-point in the forward direction.
    int lane_index = lane_sequence->adc_lane_segment_idx();
    double total_s = 0.0;
    double lane_seg_s = lane_sequence->lane_segment(lane_index).adc_s();
    if (!is_bidirection) {
      lane_index = 0;
      lane_seg_s = lane_sequence->lane_segment(0).start_s();
    }
    std::size_t count_point = 0;
    while (lane_index < lane_sequence->lane_segment_size() &&
           count_point < max_num_lane_point) {
      // Go through lane_segment one by one sequentially.
      LaneSegment* lane_segment =
          lane_sequence->mutable_lane_segment(lane_index);

      if (lane_seg_s > lane_segment->end_s()) {
        // If already exceeds the current lane_segment, then go to the
        // next following one.
        ADEBUG << "Move on to the next lane-segment.";
        lane_seg_s = lane_seg_s - lane_segment->end_s();
        ++lane_index;
      } else {
        // Otherwise, update lane_graph:
        // 1. Sanity checks.
        std::string lane_id = lane_segment->lane_id();
        lane_segment->set_lane_turn_type(PredictionMap::LaneTurnType(lane_id));
        ADEBUG << "Currently on " << lane_id;
        auto lane_info = PredictionMap::LaneById(lane_id);
        if (lane_info == nullptr) {
          break;
        }
        // 2. Get the closeset lane_point
        ADEBUG << "Lane-segment s = " << lane_seg_s;
        Eigen::Vector2d lane_point_pos =
            PredictionMap::PositionOnLane(lane_info, lane_seg_s);
        double lane_point_heading =
            PredictionMap::HeadingOnLane(lane_info, lane_seg_s);
        double lane_point_width =
            PredictionMap::LaneTotalWidth(lane_info, lane_seg_s);
        double lane_point_angle_diff =
            common::math::AngleDiff(lane_point_heading, heading);
        // 3. Update it into the lane_graph
        ADEBUG << lane_point_pos[0] << "    " << lane_point_pos[1];
        LanePoint lane_point;
        // Update direct information.
        lane_point.mutable_position()->set_x(lane_point_pos[0]);
        lane_point.mutable_position()->set_y(lane_point_pos[1]);
        lane_point.set_heading(lane_point_heading);
        lane_point.set_width(lane_point_width);
        lane_point.set_angle_diff(lane_point_angle_diff);
        // Update deducted information.
        double lane_l = feature->lane().lane_feature().lane_l();
        lane_point.set_relative_s(total_s);
        lane_point.set_relative_l(0.0 - lane_l);
        // Update into lane_segment.
        lane_segment->add_lane_point()->CopyFrom(lane_point);
        ++count_point;
        total_s += lane_point_spacing;
        lane_seg_s += lane_point_spacing;
      }
    }
  }
  ADEBUG << "Obstacle [" << id_ << "] has lane segments and points.";
}

void Obstacle::SetLaneSequencePath(LaneGraph* const lane_graph) {
  // Go through every lane_sequence.
  for (int i = 0; i < lane_graph->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence = lane_graph->mutable_lane_sequence(i);
    double lane_segment_s = 0.0;
    // Go through every lane_segment.
    for (int j = 0; j < lane_sequence->lane_segment_size(); ++j) {
      LaneSegment* lane_segment = lane_sequence->mutable_lane_segment(j);
      // Go through every lane_point and set the corresponding path_point.
      for (int k = 0; k < lane_segment->lane_point_size(); ++k) {
        LanePoint* lane_point = lane_segment->mutable_lane_point(k);
        PathPoint path_point;
        path_point.set_s(lane_point->relative_s());
        path_point.set_theta(lane_point->heading());
        lane_sequence->add_path_point()->CopyFrom(path_point);
      }
      lane_segment_s += lane_segment->total_length();
    }
    // Sanity checks.
    int num_path_point = lane_sequence->path_point_size();
    if (num_path_point <= 0) {
      continue;
    }
    // Go through every path_point, calculate kappa, and update it.
    int segment_index = 0;
    int point_index = 0;
    for (int j = 0; j + 1 < num_path_point; ++j) {
      while (segment_index < lane_sequence->lane_segment_size() &&
             point_index >=
                 lane_sequence->lane_segment(segment_index).lane_point_size()) {
        ++segment_index;
        point_index = 0;
      }
      PathPoint* first_point = lane_sequence->mutable_path_point(j);
      PathPoint* second_point = lane_sequence->mutable_path_point(j + 1);
      double delta_theta = apollo::common::math::AngleDiff(
          second_point->theta(), first_point->theta());
      double delta_s = second_point->s() - first_point->s();
      double kappa = std::abs(delta_theta / (delta_s + FLAGS_double_precision));
      lane_sequence->mutable_path_point(j)->set_kappa(kappa);
      if (segment_index < lane_sequence->lane_segment_size() &&
          point_index <
              lane_sequence->lane_segment(segment_index).lane_point_size()) {
        lane_sequence->mutable_lane_segment(segment_index)
            ->mutable_lane_point(point_index)
            ->set_kappa(kappa);
        ++point_index;
      }
    }
    lane_sequence->mutable_path_point(num_path_point - 1)->set_kappa(0.0);
  }
}

void Obstacle::SetNearbyObstacles() {
  // This function runs after all basic features have been set up
  Feature* feature_ptr = mutable_latest_feature();

  LaneGraph* lane_graph = feature_ptr->mutable_lane()->mutable_lane_graph();
  for (int i = 0; i < lane_graph->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence = lane_graph->mutable_lane_sequence(i);
    if (lane_sequence->lane_segment_size() == 0) {
      AERROR << "Empty lane sequence found.";
      continue;
    }
    double obstacle_s = lane_sequence->lane_s();
    double obstacle_l = lane_sequence->lane_l();
    NearbyObstacle forward_obstacle;
    if (ObstacleClusters::ForwardNearbyObstacle(
            *lane_sequence, id_, obstacle_s, obstacle_l, &forward_obstacle)) {
      lane_sequence->add_nearby_obstacle()->CopyFrom(forward_obstacle);
    }
    NearbyObstacle backward_obstacle;
    if (ObstacleClusters::BackwardNearbyObstacle(
            *lane_sequence, id_, obstacle_s, obstacle_l, &backward_obstacle)) {
      lane_sequence->add_nearby_obstacle()->CopyFrom(backward_obstacle);
    }
  }
}

void Obstacle::SetMotionStatus() {
  int history_size = static_cast<int>(feature_history_.size());
  if (history_size == 0) {
    AERROR << "Zero history found";
    return;
  }
  double pos_std = FLAGS_still_obstacle_position_std;
  double speed_threshold = FLAGS_still_obstacle_speed_threshold;
  if (type_ == PerceptionObstacle::PEDESTRIAN ||
      type_ == PerceptionObstacle::BICYCLE) {
    speed_threshold = FLAGS_still_pedestrian_speed_threshold;
    pos_std = FLAGS_still_pedestrian_position_std;
  } else if (type_ == PerceptionObstacle::UNKNOWN ||
             type_ == PerceptionObstacle::UNKNOWN_MOVABLE) {
    speed_threshold = FLAGS_still_unknown_speed_threshold;
    pos_std = FLAGS_still_unknown_position_std;
  }
  double speed = feature_history_.front().speed();

  if (history_size == 1) {
    if (speed < speed_threshold) {
      ADEBUG << "Obstacle [" << id_ << "] has a small speed [" << speed
             << "] and is considered stationary in the first frame.";
      feature_history_.front().set_is_still(true);
    } else {
      feature_history_.front().set_is_still(false);
    }
    return;
  }

  double start_x = 0.0;
  double start_y = 0.0;
  double avg_drift_x = 0.0;
  double avg_drift_y = 0.0;
  int len = std::min(history_size, FLAGS_max_still_obstacle_history_length);
  len = std::max(len, FLAGS_min_still_obstacle_history_length);
  CHECK_GT(len, 1);

  auto feature_riter = feature_history_.rbegin();
  start_x = feature_riter->position().x();
  start_y = feature_riter->position().y();
  ++feature_riter;
  while (feature_riter != feature_history_.rend()) {
    avg_drift_x += (feature_riter->position().x() - start_x) / (len - 1);
    avg_drift_y += (feature_riter->position().y() - start_y) / (len - 1);
    ++feature_riter;
  }

  double delta_ts = feature_history_.front().timestamp() -
                    feature_history_.back().timestamp();
  double speed_sensibility = std::sqrt(2 * history_size) * 4 * pos_std /
                             ((history_size + 1) * delta_ts);
  if (speed < speed_threshold) {
    ADEBUG << "Obstacle [" << id_ << "] has a small speed [" << speed
           << "] and is considered stationary.";
    feature_history_.front().set_is_still(true);
  } else if (speed_sensibility < speed_threshold) {
    ADEBUG << "Obstacle [" << id_ << "]"
           << "] considered moving [sensibility = " << speed_sensibility << "]";
    feature_history_.front().set_is_still(false);
  } else {
    double distance = std::hypot(avg_drift_x, avg_drift_y);
    double distance_std = std::sqrt(2.0 / len) * pos_std;
    if (distance > 2.0 * distance_std) {
      ADEBUG << "Obstacle [" << id_ << "] is moving.";
      feature_history_.front().set_is_still(false);
    } else {
      ADEBUG << "Obstacle [" << id_ << "] is stationary.";
      feature_history_.front().set_is_still(true);
    }
  }
}

void Obstacle::SetMotionStatusBySpeed() {
  auto history_size = feature_history_.size();
  if (history_size < 2) {
    ADEBUG << "Obstacle [" << id_ << "] has no history and "
           << "is considered moving.";
    if (history_size > 0) {
      feature_history_.front().set_is_still(false);
    }
    return;
  }

  double speed_threshold = FLAGS_still_obstacle_speed_threshold;
  double speed = feature_history_.front().speed();

  if (FLAGS_use_navigation_mode) {
    if (speed < speed_threshold) {
      feature_history_.front().set_is_still(true);
    } else {
      feature_history_.front().set_is_still(false);
    }
  }
}

void Obstacle::InsertFeatureToHistory(const Feature& feature) {
  feature_history_.emplace_front(feature);
  ADEBUG << "Obstacle [" << id_ << "] inserted a frame into the history.";
}

std::unique_ptr<Obstacle> Obstacle::Create(
    const PerceptionObstacle& perception_obstacle, const double timestamp,
    const int prediction_id) {
  std::unique_ptr<Obstacle> ptr_obstacle(new Obstacle());
  if (!ptr_obstacle->Insert(perception_obstacle, timestamp, prediction_id)) {
    return nullptr;
  }
  return ptr_obstacle;
}

std::unique_ptr<Obstacle> Obstacle::Create(const Feature& feature) {
  std::unique_ptr<Obstacle> ptr_obstacle(new Obstacle());
  ptr_obstacle->InsertFeatureToHistory(feature);
  return ptr_obstacle;
}

bool Obstacle::ReceivedNewerMessage(const double timestamp) const {
  if (feature_history_.empty()) {
    return false;
  }
  auto last_timestamp_received = feature_history_.front().timestamp();
  return timestamp <= last_timestamp_received;
}

void Obstacle::DiscardOutdatedHistory() {
  auto num_of_frames = feature_history_.size();
  const double latest_ts = feature_history_.front().timestamp();
  while (latest_ts - feature_history_.back().timestamp() >=
         FLAGS_max_history_time) {
    feature_history_.pop_back();
  }
  auto num_of_discarded_frames = num_of_frames - feature_history_.size();
  if (num_of_discarded_frames > 0) {
    ADEBUG << "Obstacle [" << id_ << "] discards " << num_of_discarded_frames
           << " historical features";
  }
}

void Obstacle::SetRNNStates(const std::vector<Eigen::MatrixXf>& rnn_states) {
  rnn_states_ = rnn_states;
}

void Obstacle::GetRNNStates(std::vector<Eigen::MatrixXf>* rnn_states) {
  rnn_states->clear();
  rnn_states->insert(rnn_states->end(), rnn_states_.begin(), rnn_states_.end());
}

void Obstacle::InitRNNStates() {
  if (network::RnnModel::Instance()->IsOk()) {
    network::RnnModel::Instance()->ResetState();
    network::RnnModel::Instance()->State(&rnn_states_);
    rnn_enabled_ = true;
    ADEBUG << "Success to initialize rnn model.";
  } else {
    AWARN << "Fail to initialize rnn model.";
    rnn_enabled_ = false;
  }
}

bool Obstacle::RNNEnabled() const { return rnn_enabled_; }

void Obstacle::SetCaution() {
  CHECK_GT(feature_history_.size(), 0);
  Feature* feature = mutable_latest_feature();
  feature->mutable_priority()->set_priority(ObstaclePriority::CAUTION);
}

void Obstacle::SetEvaluatorType(
    const ObstacleConf::EvaluatorType& evaluator_type) {
  obstacle_conf_.set_evaluator_type(evaluator_type);
}

void Obstacle::SetPredictorType(
    const ObstacleConf::PredictorType& predictor_type) {
  obstacle_conf_.set_predictor_type(predictor_type);
}

}  // namespace prediction
}  // namespace apollo
