/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_interface_base/task_base/task.h"

#include "cyber/class_loader/class_loader_manager.h"
#include "cyber/plugin_manager/plugin_manager.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;

Task::Task()
    : frame_(nullptr),
      reference_line_info_(nullptr),
      injector_(nullptr),
      config_path_(""),
      default_config_path_(""),
      name_("") {}

bool Task::Init(const std::string& config_dir, const std::string& name,
                const std::shared_ptr<DependencyInjector>& injector) {
  injector_ = injector;
  name_ = name;
  config_path_ =
      config_dir + "/" + ConfigUtil::TransformToPathName(name) + ".pb.txt";

  // Get the name of this class.
  int status;
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
  // Generate the default task config path from PluginManager.
  default_config_path_ =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginConfPath<Task>(class_name, "conf/default_conf.pb.txt");
  return true;
}

const std::string& Task::Name() const { return name_; }

Status Task::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  return Status::OK();
}

Status Task::Execute(Frame* frame) {
  frame_ = frame;
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
