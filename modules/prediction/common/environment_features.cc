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
#include "modules/prediction/common/environment_features.h"

#include "cyber/common/log.h"

namespace apollo {
namespace prediction {

void EnvironmentFeatures::set_ego_position(const double x, const double y) {
  ego_position_.set_x(x);
  ego_position_.set_y(y);
  ego_position_.set_z(0.0);
}

const apollo::common::Point3D& EnvironmentFeatures::get_ego_position() const {
  return ego_position_;
}

void EnvironmentFeatures::set_ego_speed(const double ego_speed) {
  ego_speed_ = ego_speed;
}

double EnvironmentFeatures::get_ego_speed() const { return ego_speed_; }

void EnvironmentFeatures::set_ego_heading(const double ego_heading) {
  ego_heading_ = ego_heading;
}

double EnvironmentFeatures::get_ego_heading() const { return ego_heading_; }

void EnvironmentFeatures::set_ego_acceleration(const double ego_acceleration) {
  ego_acceleration_ = ego_acceleration;
}

double EnvironmentFeatures::get_ego_acceleration() const {
  return ego_acceleration_;
}

bool EnvironmentFeatures::has_ego_lane() const { return has_ego_lane_; }

void EnvironmentFeatures::reset_ego_lane() {
  has_ego_lane_ = false;
  ego_lane_id_ = "";
  ego_lane_s_ = -1.0;
}

void EnvironmentFeatures::SetEgoLane(const std::string& lane_id,
                                     const double lane_s) {
  has_ego_lane_ = true;
  ego_lane_id_ = lane_id;
  ego_lane_s_ = lane_s;
}

std::pair<std::string, double> EnvironmentFeatures::GetEgoLane() const {
  ACHECK(has_ego_lane_);
  return {ego_lane_id_, ego_lane_s_};
}

bool EnvironmentFeatures::has_left_neighbor_lane() const {
  return has_left_neighbor_lane_;
}

void EnvironmentFeatures::reset_left_neighbor_lane() {
  has_left_neighbor_lane_ = false;
}

void EnvironmentFeatures::SetLeftNeighborLane(const std::string& lane_id,
                                              const double lane_s) {
  has_left_neighbor_lane_ = true;
  left_neighbor_lane_id_ = lane_id;
  left_neighbor_lane_s_ = lane_s;
}

std::pair<std::string, double> EnvironmentFeatures::GetLeftNeighborLane()
    const {
  ACHECK(has_left_neighbor_lane_);
  return {left_neighbor_lane_id_, left_neighbor_lane_s_};
}

bool EnvironmentFeatures::has_right_neighbor_lane() const {
  return has_right_neighbor_lane_;
}

void EnvironmentFeatures::reset_right_neighbor_lane() {
  has_right_neighbor_lane_ = false;
}

void EnvironmentFeatures::SetRightNeighborLane(const std::string& lane_id,
                                               const double lane_s) {
  has_right_neighbor_lane_ = true;
  right_neighbor_lane_id_ = lane_id;
  right_neighbor_lane_s_ = lane_s;
}

std::pair<std::string, double> EnvironmentFeatures::GetRightNeighborLane()
    const {
  ACHECK(has_right_neighbor_lane_);
  return {right_neighbor_lane_id_, right_neighbor_lane_s_};
}

bool EnvironmentFeatures::has_front_junction() const {
  return has_front_junction_;
}

void EnvironmentFeatures::reset_front_junction() {
  has_front_junction_ = false;
}

void EnvironmentFeatures::SetFrontJunction(const std::string& junction_id,
                                           const double dist) {
  has_front_junction_ = true;
  front_junction_id_ = junction_id;
  dist_to_front_junction_ = dist;
}

std::pair<std::string, double> EnvironmentFeatures::GetFrontJunction() const {
  ACHECK(has_front_junction_);
  return {front_junction_id_, dist_to_front_junction_};
}

void EnvironmentFeatures::AddObstacleId(const int obstacle_id) {
  obstacle_ids_.push_back(obstacle_id);
}

const std::vector<int>& EnvironmentFeatures::get_obstacle_ids() const {
  return obstacle_ids_;
}

const std::unordered_set<std::string>&
EnvironmentFeatures::nonneglectable_reverse_lanes() const {
  return nonneglectable_reverse_lanes_;
}

void EnvironmentFeatures::AddNonneglectableReverseLanes(
    const std::string& lane_id) {
  nonneglectable_reverse_lanes_.insert(lane_id);
}

bool EnvironmentFeatures::RemoveNonneglectableReverseLanes(
    const std::string& lane_id) {
  if (nonneglectable_reverse_lanes_.find(lane_id) ==
      nonneglectable_reverse_lanes_.end()) {
    return false;
  }
  nonneglectable_reverse_lanes_.erase(lane_id);
  return true;
}

}  // namespace prediction
}  // namespace apollo
