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
 * @file planning_data.cc
 **/

#include "modules/planning/common/planning_data.h"

namespace apollo {
namespace planning {

const ReferenceLine& PlanningData::reference_line() const {
  return *_reference_line.get();
}

const DecisionData& PlanningData::decision_data() const {
  return *_decision_data.get();
}

const PublishableTrajectory& PlanningData::computed_trajectory() const {
  return _computed_trajectory;
}

const TrajectoryPoint& PlanningData::init_planning_point() const {
  return _init_planning_point;
}

DecisionData* PlanningData::mutable_decision_data() const {
  return _decision_data.get();
}

PublishableTrajectory* PlanningData::mutable_computed_trajectory() {
  return &_computed_trajectory;
};

void PlanningData::set_init_planning_point(
    const TrajectoryPoint& init_planning_point) {
  _init_planning_point = init_planning_point;
}

void PlanningData::set_reference_line(
    std::unique_ptr<ReferenceLine>& reference_line) {
  _reference_line = std::move(reference_line);
}
void PlanningData::set_decision_data(
    std::unique_ptr<DecisionData>& decision_data) {
  _decision_data = std::move(decision_data);
}

void PlanningData::set_computed_trajectory(
    PublishableTrajectory publishable_trajectory) {
  _computed_trajectory = std::move(publishable_trajectory);
}

}  // namespace planning
}  // namespace apollo
