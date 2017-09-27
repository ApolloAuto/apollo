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

/**
 * @file planning_objective.h
 *
 */

#ifndef MODULES_PLANNING_LATTICE_PLANNING_TARGET_H_
#define MODULES_PLANNING_LATTICE_PLANNING_TARGET_H_

#include <array>
#include <utility>
#include <string>

namespace apollo {
namespace planning {

// follow, follow which target
// stop, stop at which point (yielding will also be considered as stop?)
// cruise, target speed
class PlanningTarget {
 public:
  enum class Task {
    FOLLOW,
    STOP,
    CRUISE
  };

  PlanningTarget() = default;

  virtual ~PlanningTarget() = default;

  PlanningTarget::Task task() const;

  std::string task_name() const;

  void set_task(const PlanningTarget::Task& task);

  void set_cruise_target(const double cruise_speed);

  void set_stop_target(const double stop_position);

  void set_follow_target(const double follow_target_position,
                         const double follow_target_velocity,
                         const double follow_target_time);

  double cruise_target() const;

  double stop_target() const;

  std::array<double, 3> follow_target() const;

 private:
  Task task_ = Task::CRUISE;

  double cruise_target_ = 0.0;

  double stop_target_ = 0.0;

  double follow_target_position_ = 0.0;

  double follow_target_time_ = 0.0;

  double follow_target_velocity_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_PLANNING_TARGET_H_
