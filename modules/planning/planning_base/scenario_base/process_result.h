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
 * @file process_result.h
 */

#pragma once

#include <string>

#include "modules/common/status/status.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

enum class StageStatusType {
  ERROR = 1,
  READY = 2,
  RUNNING = 3,
  FINISHED = 4,
};

enum class ScenarioStatusType {
  STATUS_UNKNOWN = 0,
  STATUS_PROCESSING = 1,
  STATUS_DONE = 2,
};

/**
 * @class ProcessResult
 *
 * @brief This class contains the execution result of scenario, stage and task.
 */
class StageResult {
 public:
  StageResult(const StageStatusType& stage_status = StageStatusType::READY,
              const common::Status& task_status = common::Status::OK());
  /**
   * @brief Get the task execution result.
   *
   * @return The execution result of task.
   */
  const common::Status& GetTaskStatus() const;
  /**
   * @brief Get the stage status.
   *
   * @return The stage status.
   */
  const StageStatusType& GetStageStatus() const;
  /**
   * @brief Set the task execution result.
   *
   * @param status The execution result of task.
   */
  const StageResult& SetTaskStatus(const common::Status& status);
  /**
   * @brief Set the stage status.
   *
   * @param stage_status The stage status.
   */
  const StageResult& SetStageStatus(const StageStatusType& stage_status);
  /**
   * @brief Set the stage status.
   *
   * @param stage_status The stage status.
   * @param message Stage message.
   */
  const StageResult& SetStageStatus(const StageStatusType& stage_status,
                                    const std::string& message);
  /**
   * @brief Check if StageResult contains error.
   *
   * @return True if there is error contained.
   */
  bool HasError() const;
  bool IsTaskError() const;

 private:
  common::Status task_status_;
  StageStatusType stage_status_;
};

/**
 * @class ProcessResult
 *
 * @brief This class contains the execution result of stage and task.
 */
class ScenarioResult {
 public:
  ScenarioResult(const ScenarioStatusType& scenario_status =
                     ScenarioStatusType::STATUS_UNKNOWN,
                 const StageStatusType& stage_status = StageStatusType::READY,
                 const common::Status& task_status = common::Status::OK());
  /**
   * @brief Get the task execution result.
   *
   * @return The execution result of task.
   */
  const common::Status& GetTaskStatus() const;
  /**
   * @brief Get the stage status.
   *
   * @return The stage status.
   */
  const StageStatusType& GetStageStatus() const;
  /**
   * @brief Get the scenario status.
   *
   * @return The scenario status.
   */
  const ScenarioStatusType& GetScenarioStatus() const;
  /**
   * @brief Set the stage status.
   *
   * @param stage_result The stage result.
   */
  const ScenarioResult& SetStageResult(const StageResult& stage_result);
  /**
   * @brief Set the stage status.
   * @param stage_status The stage status.
   * @param message Stage message.
   *
   * @param stage_result The stage result.
   */
  const ScenarioResult& SetStageResult(const StageStatusType& stage_status,
                                       const std::string& message);
  /**
   * @brief Set the scenario status.
   *
   * @param scenario_status The scenario status.
   */
  const ScenarioResult& SetScenarioStatus(
      const ScenarioStatusType& scenario_status);

 private:
  ScenarioStatusType scenario_status_;
  StageResult stage_result_;
};

}  // namespace planning
}  // namespace apollo
