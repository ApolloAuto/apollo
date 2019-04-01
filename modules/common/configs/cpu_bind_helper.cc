/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/configs/cpu_bind_helper.h"

#include <sched.h>
#include <unistd.h>

#include <fstream>
#include <utility>

#include "modules/common/configs/config_gflags.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace common {

CpuBindHelper::CpuBindHelper() {
}

void CpuBindHelper::Init(const std::string &config_file) {
  init_ = true;
  YAML::Node root_node = YAML::LoadFile(config_file);
  if (root_node["cpubind"]) {
    const auto& rule_node = root_node["cpubind"];
    std::size_t rule_size = rule_node.size();
    for (std::size_t i = 0; i < rule_size; ++i) {
      const auto& module_node = rule_node[i];
      const std::string module_name = module_node["module"].as<std::string>();
      const std::size_t core_size = module_node["cores"].size();
      std::vector<int> core_list(core_size);
      for (std::size_t j = 0; j < core_size; ++j) {
        core_list[j] = module_node["cores"][j].as<int>();
      }

      bindrule_map_.emplace(module_name, std::move(core_list));
    }
  }
}

void CpuBindHelper::BindCpu(const std::string& module_name) {
  if (!init_) {
    Init(FLAGS_cpubind_config_path);
  }

  if (bindrule_map_.find(module_name) != bindrule_map_.end()) {
    cpu_set_t mask;
    CPU_ZERO(&mask);

    const std::vector<int>& core_list = bindrule_map_.at(module_name);
    for (const auto& elem : core_list) {
      CPU_SET(elem, &mask);
    }

    sched_setaffinity(0, sizeof(mask), &mask);
  }
}

}  // namespace common
}  // namespace apollo


