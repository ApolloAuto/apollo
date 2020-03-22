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

#include <string>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {

class Task {
 public:
  explicit Task(const TaskConfig& config);

  virtual ~Task() = default;

  const std::string& Name() const;

  const TaskConfig& Config() const { return config_; }

  virtual common::Status Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

  virtual common::Status Execute(Frame* frame);

 protected:
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;

  TaskConfig config_;
  std::string name_;
};

}  // namespace planning
}  // namespace apollo
