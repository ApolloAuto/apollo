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

#include <memory>
#include <string>
#include <unordered_map>

#include "modules/planning/planning_interface_base/scenario_base/proto/scenario_pipeline.pb.h"

#include "cyber/common/file.h"
#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_interface_base/scenario_base/process_result.h"

namespace apollo {
namespace common {
class TrajectoryPoint;
}  // namespace common
}  // namespace apollo

namespace apollo {
namespace planning {

class Frame;

struct ScenarioContext {
 public:
  ScenarioContext() {}
};

class Stage;
class DependencyInjector;

class Scenario {
 public:
  Scenario();

  virtual ~Scenario() = default;

  virtual bool Init(std::shared_ptr<DependencyInjector> injector,
                    const std::string& name);

  /**
   * @brief Get the scenario context.
   */
  virtual ScenarioContext* GetContext() = 0;

  /**
   * Each scenario should define its own transfer condition, i.e., when it
   * should allow to transfer from other scenario to itself.
   */
  virtual bool IsTransferable(const Scenario* other_scenario,
                              const Frame& frame) {
    return false;
  }

  virtual ScenarioResult Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame);

  virtual bool Exit(Frame* frame) { return true; }

  virtual bool Enter(Frame* frame) { return true; }

  /**
   * Each scenario should define its own stages object's creation
   * scenario will call stage's Stage::Process function following a configured
   * order, The return value of Stage::Process function determines the
   * transition from one stage to another.
   */
  std::shared_ptr<Stage> CreateStage(const StagePipeline& stage_pipeline);

  const ScenarioStatusType& GetStatus() const {
    return scenario_result_.GetScenarioStatus();
  }

  const std::string GetStage() const;

  const std::string& GetMsg() const { return msg_; }

  const std::string& Name() const { return name_; }

  /**
   * @brief Reset the scenario, used before entering the scenario.
   */
  void Reset();

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  ScenarioResult scenario_result_;
  std::shared_ptr<Stage> current_stage_;
  std::unordered_map<std::string, const StagePipeline*> stage_pipeline_map_;
  std::string msg_;  // debug msg
  std::shared_ptr<DependencyInjector> injector_;

  std::string config_path_;
  std::string config_dir_;
  std::string name_;
  ScenarioPipeline scenario_pipeline_config_;
};

template <typename T>
bool Scenario::LoadConfig(T* config) {
  return apollo::cyber::common::LoadConfig<T>(config_path_, config);
}

}  // namespace planning
}  // namespace apollo
