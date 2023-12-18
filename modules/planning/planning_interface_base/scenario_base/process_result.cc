/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file process_result.cc
 */

#include "modules/planning/planning_interface_base/scenario_base/process_result.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

StageResult::StageResult(const StageStatusType& stage_status,
                         const common::Status& task_status)
    : task_status_(task_status), stage_status_(stage_status) {}

const common::Status& StageResult::GetTaskStatus() const {
  return task_status_;
}

const StageStatusType& StageResult::GetStageStatus() const {
  return stage_status_;
}

const StageResult& StageResult::SetTaskStatus(const common::Status& status) {
  task_status_ = status;
  return *this;
}

const StageResult& StageResult::SetStageStatus(
    const StageStatusType& stage_status) {
  stage_status_ = stage_status;
  return *this;
}

const StageResult& StageResult::SetStageStatus(
    const StageStatusType& stage_status, const std::string& message) {
  stage_status_ = stage_status;
  task_status_ = common::Status(task_status_.code(), message);
  return *this;
}

bool StageResult::HasError() const {
  return stage_status_ == StageStatusType::ERROR;
}

bool StageResult::IsTaskError() const { return !task_status_.ok(); }

ScenarioResult::ScenarioResult(const ScenarioStatusType& scenario_status,
                               const StageStatusType& stage_status,
                               const common::Status& task_status)
    : scenario_status_(scenario_status),
      stage_result_(stage_status, task_status) {}

const common::Status& ScenarioResult::GetTaskStatus() const {
  return stage_result_.GetTaskStatus();
}

const StageStatusType& ScenarioResult::GetStageStatus() const {
  return stage_result_.GetStageStatus();
}

const ScenarioStatusType& ScenarioResult::GetScenarioStatus() const {
  return scenario_status_;
}

const ScenarioResult& ScenarioResult::SetStageResult(
    const StageResult& stage_result) {
  stage_result_ = stage_result;
  return *this;
}

const ScenarioResult& ScenarioResult::SetStageResult(
    const StageStatusType& stage_status, const std::string& message) {
  stage_result_ = StageResult();
  stage_result_.SetStageStatus(stage_status, message);
  return *this;
}

const ScenarioResult& ScenarioResult::SetScenarioStatus(
    const ScenarioStatusType& scenario_status) {
  scenario_status_ = scenario_status;
  return *this;
}

}  // namespace planning
}  // namespace apollo
