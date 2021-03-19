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

#include "cyber/mainboard/module_controller.h"

#include <utility>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/component/component_base.h"

namespace apollo {
namespace cyber {
namespace mainboard {

void ModuleController::Clear() {
  for (auto& component : component_list_) {
    component->Shutdown();
  }
  component_list_.clear();  // keep alive
  class_loader_manager_.UnloadAllLibrary();
}

bool ModuleController::LoadAll() {
  const std::string work_root = common::WorkRoot();
  const std::string current_path = common::GetCurrentPath();
  const std::string dag_root_path = common::GetAbsolutePath(work_root, "dag");
  std::vector<std::string> paths;
  for (auto& dag_conf : args_.GetDAGConfList()) {
    std::string module_path = "";
    if (dag_conf == common::GetFileName(dag_conf)) {
      // case dag conf argument var is a filename
      module_path = common::GetAbsolutePath(dag_root_path, dag_conf);
    } else if (dag_conf[0] == '/') {
      // case dag conf argument var is an absolute path
      module_path = dag_conf;
    } else {
      // case dag conf argument var is a relative path
      module_path = common::GetAbsolutePath(current_path, dag_conf);
      if (!common::PathExists(module_path)) {
        module_path = common::GetAbsolutePath(work_root, dag_conf);
      }
    }
    total_component_nums += GetComponentNum(module_path);
    paths.emplace_back(std::move(module_path));
  }
  if (has_timer_component) {
    total_component_nums += scheduler::Instance()->TaskPoolSize();
  }
  common::GlobalData::Instance()->SetComponentNums(total_component_nums);
  for (auto module_path : paths) {
    AINFO << "Start initialize dag: " << module_path;
    if (!LoadModule(module_path)) {
      AERROR << "Failed to load module: " << module_path;
      return false;
    }
  }
  return true;
}

bool ModuleController::LoadModule(const DagConfig& dag_config) {
  const std::string work_root = common::WorkRoot();

  for (auto module_config : dag_config.module_config()) {
    std::string load_path;
    if (module_config.module_library().front() == '/') {
      load_path = module_config.module_library();
    } else {
      load_path =
          common::GetAbsolutePath(work_root, module_config.module_library());
    }

    if (!common::PathExists(load_path)) {
      AERROR << "Path does not exist: " << load_path;
      return false;
    }

    class_loader_manager_.LoadLibrary(load_path);

    for (auto& component : module_config.components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }

    for (auto& component : module_config.timer_components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }
  }
  return true;
}

bool ModuleController::LoadModule(const std::string& path) {
  DagConfig dag_config;
  if (!common::GetProtoFromFile(path, &dag_config)) {
    AERROR << "Get proto failed, file: " << path;
    return false;
  }
  return LoadModule(dag_config);
}

int ModuleController::GetComponentNum(const std::string& path) {
  DagConfig dag_config;
  int component_nums = 0;
  if (common::GetProtoFromFile(path, &dag_config)) {
    for (auto module_config : dag_config.module_config()) {
      component_nums += module_config.components_size();
      if (module_config.timer_components_size() > 0) {
        has_timer_component = true;
      }
    }
  }
  return component_nums;
}

}  // namespace mainboard
}  // namespace cyber
}  // namespace apollo
