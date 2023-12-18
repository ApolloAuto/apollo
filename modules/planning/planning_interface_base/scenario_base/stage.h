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

#include "modules/planning/planning_interface_base/scenario_base/proto/scenario_pipeline.pb.h"

#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_interface_base/scenario_base/process_result.h"

namespace apollo {
namespace common {
class TrajectoryPoint;
}  // namespace common
}  // namespace apollo

namespace apollo {
namespace planning {

class Task;
class Frame;
class ReferenceLineInfo;

class Stage {
 public:
  Stage();

  virtual bool Init(const StagePipeline& config,
                    const std::shared_ptr<DependencyInjector>& injector,
                    const std::string& config_dir, void* context);

  virtual ~Stage() = default;

  /**
   * @brief Each stage does its business logic inside Process function.
   * If the stage want to transit to a different stage after finish,
   * it should set the type of 'next_stage_'.
   */
  virtual StageResult Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  const std::string& Name() const;

  template <typename T>
  T* GetContextAs() const {
    return static_cast<T*>(context_);
  }

  const std::string& NextStage() const { return next_stage_; }

 protected:
  StageResult ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  StageResult ExecuteTaskOnReferenceLineForOnlineLearning(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  StageResult ExecuteTaskOnOpenSpace(Frame* frame);

  virtual StageResult FinishScenario();

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                       const std::string& name, const double time_diff_ms);

  std::vector<std::shared_ptr<Task>> task_list_;
  std::shared_ptr<Task> fallback_task_;
  std::string next_stage_;
  void* context_;
  std::shared_ptr<DependencyInjector> injector_;
  StagePipeline pipeline_config_;

 private:
  std::string name_;
};

}  // namespace planning
}  // namespace apollo
