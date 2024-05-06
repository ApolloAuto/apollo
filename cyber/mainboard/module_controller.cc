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
#include "cyber/plugin_manager/plugin_manager.h"

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
  for (auto& plugin_description : args_.GetPluginDescriptionList()) {
    apollo::cyber::plugin_manager::PluginManager::Instance()->LoadPlugin(
        plugin_description);
  }
  if (!args_.GetDisablePluginsAutoLoad()) {
    apollo::cyber::plugin_manager::PluginManager::Instance()
        ->LoadInstalledPlugins();
  }
  for (auto& dag_conf : args_.GetDAGConfList()) {
    std::string module_path = dag_conf;
    if (!common::GetFilePathWithEnv(dag_conf, "APOLLO_DAG_PATH",
                                    &module_path)) {
      AERROR << "no dag conf [" << dag_conf << "] found!";
      return false;
    }
    AINFO << "mainboard: use dag conf " << module_path;
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
  for (auto module_config : dag_config.module_config()) {
    std::string load_path;
    if (!common::GetFilePathWithEnv(module_config.module_library(),
                                    "APOLLO_LIB_PATH", &load_path)) {
      AERROR << "no module library [" << module_config.module_library()
             << "] found!";
      return false;
    }
    AINFO << "mainboard: use module library " << load_path;

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
