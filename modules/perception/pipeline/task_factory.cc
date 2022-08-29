/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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


#include "modules/perception/pipeline/task_factory.h"


namespace apollo {
namespace perception {
namespace pipeline {


apollo::common::util::Factory<
    TaskType, Task,
    Task *(*)(const TaskConfig& config),
    std::unordered_map<
        TaskType,
        Task *(*)(const TaskConfig& config),
        std::hash<int>>>
    TaskFactory::task_factory_;

void TaskFactory::Init() {
  task_factory_.Register(
      TaskType::GLOBAL_ROT_SCALE_TRANS,
      [](const TaskConfig& task_config) -> Task* {
        return new GlobalRotScaleTrans(task_config);
      });
  // Todo(zero): need to add more type
  // need to deal with PipelineConfig& config

}

std::unique_ptr<Task> TaskFactory::CreateTask(const TaskConfig& task_config) {
  return task_factory_.CreateObject(task_config.task_type(), task_config);
}

} // namespace pipeline
} // namespace perception
} // namespace apollo
