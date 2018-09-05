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

#include "modules/planning/planner/em/em_planner.h"

namespace apollo {
namespace planning {

using common::Status;
using common::TrajectoryPoint;

Status EMPlanner::Init(const PlanningConfig& config) {
  config_ = config;
  scenario_manager_.Init();
  return Status::OK();
}

Status EMPlanner::Plan(const TrajectoryPoint& planning_start_point,
                       Frame* frame) {
  scenario_manager_.Update();
  scenario_ = scenario_manager_.mutable_scenario();
  scenario_->Init(config_);  // init will be skipped if it was called before
  return scenario_->Process(planning_start_point, frame);
}

}  // namespace planning
}  // namespace apollo
