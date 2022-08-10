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

/**
 * @file
 **/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {
namespace scenario {

class Stage {
 public:
  enum StageStatus {
    ERROR = 1,
    READY = 2,
    RUNNING = 3,
    FINISHED = 4,
  };

  Stage(const ScenarioConfig::StageConfig& config,
        const std::shared_ptr<DependencyInjector>& injector);

  virtual ~Stage() = default;

  const ScenarioConfig::StageConfig& config() const { return config_; }

  StageType stage_type() const { return config_.stage_type(); }

  /**
   * @brief Each stage does its business logic inside Process function.
   * If the stage want to transit to a different stage after finish,
   * it should set the type of 'next_stage_'.
   */
  virtual StageStatus Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  /**
   * @brief The sequence of tasks inside the stage. These tasks usually will be
   * executed in order.
   */
  const std::vector<Task*>& TaskList() const { return task_list_; }

  const std::string& Name() const;

  template <typename T>
  T* GetContextAs() {
    return static_cast<T*>(context_);
  }

  void SetContext(void* context) { context_ = context; }

  Task* FindTask(TaskConfig::TaskType task_type) const;

  StageType NextStage() const { return next_stage_; }

 protected:
  bool ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  bool ExecuteTaskOnReferenceLineForOnlineLearning(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  bool ExecuteTaskOnOpenSpace(Frame* frame);

  virtual Stage::StageStatus FinishScenario();

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                       const std::string& name, const double time_diff_ms);

 protected:
  std::map<TaskConfig::TaskType, std::unique_ptr<Task>> tasks_;
  std::vector<Task*> task_list_;
  ScenarioConfig::StageConfig config_;
  StageType next_stage_;
  void* context_ = nullptr;
  std::string name_;
  std::shared_ptr<DependencyInjector> injector_;
};

#define DECLARE_STAGE(NAME, CONTEXT)                          \
  class NAME : public Stage {                                 \
   public:                                                    \
    explicit NAME(const ScenarioConfig::StageConfig& config)  \
        : Stage(config) {}                                    \
    Stage::StageStatus Process(                               \
        const common::TrajectoryPoint& planning_init_point,   \
        Frame* frame) override;                               \
    CONTEXT* GetContext() { return GetContextAs<CONTEXT>(); } \
  }

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
