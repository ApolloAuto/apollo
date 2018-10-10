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

#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {

ScenarioConfig::ScenarioType Scenario::scenario_type() const {
  return scenario_type_;
}

bool Scenario::InitTasks(const ScenarioConfig& config,
                         const std::string& stage_name,
                         std::vector<std::unique_ptr<Task>>* tasks) {
  CHECK_GT(config.stage_size(), 0);

  ScenarioConfig::Stage stage;
  bool stage_found = false;
  for (int i = 0; i < config.stage_size(); ++i) {
    if (config.stage(i).stage_name() == stage_name) {
      stage = config.stage(i);
      stage_found = true;
    }
  }
  if (!stage_found) {
    return false;
  }

  // get all scenario_task_configs
  std::vector<ScenarioConfig::ScenarioTaskConfig> task_configs;
  for (int i = 0; i < config.scenario_task_config_size(); ++i) {
    task_configs.push_back(config.scenario_task_config(i));
  }

  // init task in the same order as defined on conf
  for (int i = 0; i < stage.task_size(); ++i) {
    TaskType task = stage.task(i);
    auto task_config_it = std::find_if(
        task_configs.begin(), task_configs.end(),
        [task](ScenarioConfig::ScenarioTaskConfig& task_config) {
          return task_config.task_type() == task;
        });
    if (task_config_it != task_configs.end()) {
      tasks->emplace_back(task_factory_.CreateObject(task));
      AINFO << "Created task:" << TaskType_Name(task);
      if (!tasks->back()->Init(*task_config_it)) {
        AERROR << "Init task[" << TaskType_Name(task) << "] failed.";
        return false;
      }
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
