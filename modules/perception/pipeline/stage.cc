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

#include "modules/perception/pipeline/stage.h"

#include "modules/perception/pipeline/task_factory.h"


namespace apollo {
namespace perception {
namespace pipeline {


bool Stage::Initialize(const StageConfig& stage_config) {
  Clear();

  for (const auto& task_config : stage_config.task_config()) {
    task_config_map_[task_config.task_type()] = &task_config;
  }

  for (int i = 0; i < stage_config.task_config_size(); ++i) {
    auto task_type = stage_config.task_type(i);
    if (!common::util::ContainsKey(task_config_map_, task_type)) {
      AERROR << "Task type : " << TaskType_Name(task_type)
             << " has no config";
      return false;
    }

    Task* task_ptr = TaskFactory::CreateTask(stage_config);

    if (task_ptr == nullptr) {
      AERROR << "Create task type : " << TaskType_Name(task_type)
             << " failed!";
    } else {
      task_ptrs_.push_back(task_ptr);
    }
  }

  return true;
}

bool Stage::InnerProcess(DataFrame* data_frame) {
  for (const auto& task_ptr : task_ptrs_) {
    if (task_ptr->IsEnabled()) {
      task_ptr->Process(data_frame);
    }
  }
}

void Stage::Clear() {
  task_config_map_.clear();
  task_ptrs_.clear();
}

Stage::~Stage() {
  for (Task* task_ptr : task_ptrs_) {
    delete task_ptr;
    task_ptr == nullptr;
  }
}


} // namespace pipeline
} // namespace perception
} // namespace apollo
