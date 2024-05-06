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

#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

#include "cyber/class_loader/class_loader_manager.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

TrafficRule::TrafficRule() : injector_(nullptr), config_path_(""), name_("") {}

bool TrafficRule::Init(const std::string& name,
                       const std::shared_ptr<DependencyInjector>& injector) {
  injector_ = injector;
  name_ = name;

  // Get the name of this class.
  int status;
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
  // Generate the traffic rule config path from PluginManager.
  config_path_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                     ->GetPluginConfPath<TrafficRule>(
                         class_name, "conf/default_conf.pb.txt");
  return true;
}

}  //  namespace planning
}  //  namespace apollo
