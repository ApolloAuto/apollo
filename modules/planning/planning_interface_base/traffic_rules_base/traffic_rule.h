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

#pragma once

#include <memory>
#include <string>

#include <cxxabi.h>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/util/config_util.h"

namespace apollo {
namespace planning {

class TrafficRule {
 public:
  TrafficRule();

  virtual ~TrafficRule() = default;

  virtual bool Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector);

  virtual common::Status ApplyRule(
      Frame* const frame, ReferenceLineInfo* const reference_line_info) = 0;

  virtual void Reset() = 0;

  std::string Getname() { return name_; }

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  std::shared_ptr<DependencyInjector> injector_;

  std::string config_path_;
  std::string name_;
};

template <typename T>
bool TrafficRule::LoadConfig(T* config) {
  CHECK_NOTNULL(config);
  if (!apollo::cyber::common::LoadConfig<T>(config_path_, config)) {
    AERROR << "Failed to load default config file" << config_path_;
    return false;
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
