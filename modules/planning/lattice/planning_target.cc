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

#include "modules/planning/lattice/planning_target.h"

namespace apollo {
namespace planning {

PlanningTarget::Task PlanningTarget::task() const {
  return task_;
}

std::string PlanningTarget::task_name() const {
  switch (task_) {
    case Task::CRUISE:
        return "cruise";
    case Task::FOLLOW:
        return "follow";
    case Task::STOP:
        return "stop";
  }
  return "undefined";
}

void PlanningTarget::set_task(const PlanningTarget::Task& task) {
  task_ = task;
}

void PlanningTarget::set_cruise_target(const double cruise_speed) {
  cruise_target_ = cruise_speed;
}

void PlanningTarget::set_stop_target(const double stop_position) {
  stop_target_ = stop_position;
}

void PlanningTarget::set_follow_target(
    const double follow_target_position,
    const double follow_target_velocity,
    const double follow_target_time) {
  follow_target_position_ = follow_target_position;
  follow_target_velocity_ = follow_target_velocity;
  follow_target_time_ = follow_target_time;
}

double PlanningTarget::cruise_target() const {
  return cruise_target_;
}

double PlanningTarget::stop_target() const {
  return stop_target_;
}

std::array<double, 3> PlanningTarget::follow_target() const {
  return {follow_target_position_,
          follow_target_velocity_,
          follow_target_time_};
}

} // namespace planning
} // namespace adu
