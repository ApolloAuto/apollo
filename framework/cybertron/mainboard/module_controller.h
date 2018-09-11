
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
#ifndef CYBERTRON_MAINBOARD_MODULE_CONTROLLER_H_
#define CYBERTRON_MAINBOARD_MODULE_CONTROLLER_H_

#include <memory>
#include <string>
#include <vector>

#include "cybertron/class_loader/class_loader_manager.h"
#include "cybertron/component/component.h"
#include "cybertron/mainboard/module_argument.h"
#include "cybertron/proto/dag_config.pb.h"

using apollo::cybertron::proto::DagConfig;

namespace apollo {
namespace cybertron {
namespace mainboard {
class ModuleController {
 public:
  explicit ModuleController(const ModuleArgument& args);
  virtual ~ModuleController();

  bool Init();
  bool LoadAll();
  void Clear();

 private:
  // void LoadModules();
  // void UnloadModules();
  bool LoadModule(const std::string& path);
  bool LoadModule(const DagConfig& dag_config);

  ModuleArgument args_;

  class_loader::ClassLoaderManager class_loader_manager_;
  std::vector<std::shared_ptr<ComponentBase>> component_list_;
};

}  // namespace mainboard
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_MAINBOARD_MODULE_CONTROLLER_H_
