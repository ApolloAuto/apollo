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

#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/util/config_util.h"

namespace apollo {
namespace planning {

class Task {
 public:
  Task();

  virtual ~Task() = default;

  const std::string& Name() const;

  virtual bool Init(const std::string& config_dir, const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector);

  virtual common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

  virtual common::Status Execute(Frame* frame);

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  Frame* frame_;
  ReferenceLineInfo* reference_line_info_;
  std::shared_ptr<DependencyInjector> injector_;

  std::string config_path_;
  std::string default_config_path_;
  std::string name_;
};

template <typename T>
bool Task::LoadConfig(T* config) {
  return ConfigUtil::LoadMergedConfig(default_config_path_, config_path_,
                                      config);
}

}  // namespace planning
}  // namespace apollo
