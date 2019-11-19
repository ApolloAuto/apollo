/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/submodules/submodule_output.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

void SubmoduleOutput::InsertObstacle(const Obstacle& obstacle) {
  curr_frame_obstacles_.push_back(obstacle);
}

void SubmoduleOutput::InsertEgoVehicle(const Obstacle& ego_vehicle) {
  ego_vehicle_ = ego_vehicle;
}

void SubmoduleOutput::InsertPerceptionObstacle(
    const apollo::perception::PerceptionObstacle& perception_obstacle) {
  curr_frame_perception_obstacles_.push_back(perception_obstacle);
}

void SubmoduleOutput::set_curr_frame_movable_obstacle_ids(
    const std::vector<int>& curr_frame_movable_obstacle_ids) {
  curr_frame_movable_obstacle_ids_ = curr_frame_movable_obstacle_ids;
}

void SubmoduleOutput::set_curr_frame_unmovable_obstacle_ids(
    const std::vector<int>& curr_frame_unmovable_obstacle_ids) {
  curr_frame_unmovable_obstacle_ids_ = curr_frame_unmovable_obstacle_ids;
}

void SubmoduleOutput::set_curr_frame_considered_obstacle_ids(
    const std::vector<int>& curr_frame_considered_obstacle_ids) {
  curr_frame_considered_obstacle_ids_ = curr_frame_considered_obstacle_ids;
}

void SubmoduleOutput::set_perception_header(
    const apollo::common::Header& perception_header) {
  perception_header_ = perception_header;
}

void SubmoduleOutput::set_perception_error_code(
    const apollo::common::ErrorCode& perception_error_code) {
  perception_error_code_ = perception_error_code;
}

void SubmoduleOutput::set_frame_start_time(const double frame_start_time) {
  frame_start_time_ = frame_start_time;
}

const std::vector<Obstacle>& SubmoduleOutput::curr_frame_obstacles() const {
  return curr_frame_obstacles_;
}

const Obstacle& SubmoduleOutput::GetEgoVehicle() const {
  return ego_vehicle_;
}

const std::vector<apollo::perception::PerceptionObstacle>&
SubmoduleOutput::curr_frame_perception_obstacles() const {
  return curr_frame_perception_obstacles_;
}

std::vector<int> SubmoduleOutput::curr_frame_movable_obstacle_ids() const {
  return curr_frame_movable_obstacle_ids_;
}

std::vector<int> SubmoduleOutput::curr_frame_unmovable_obstacle_ids() const {
  return curr_frame_unmovable_obstacle_ids_;
}

std::vector<int> SubmoduleOutput::curr_frame_considered_obstacle_ids() const {
  return curr_frame_considered_obstacle_ids_;
}

apollo::common::Header SubmoduleOutput::perception_header() const {
  return perception_header_;
}

apollo::common::ErrorCode SubmoduleOutput::perception_error_code() const {
  return perception_error_code_;
}

double SubmoduleOutput::frame_start_time() const { return frame_start_time_; }

}  // namespace prediction
}  // namespace apollo
