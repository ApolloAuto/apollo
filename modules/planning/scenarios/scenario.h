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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/toolkits/task.h"

namespace apollo {
namespace planning {

class Scenario {
 public:
  enum ScenarioStatus {
    STATUS_UNKNOWN = 0,
    STATUS_INITED = 1,
    STATUS_PROCESSING = 2,
    STATUS_DONE = 3,
  };

  Scenario() = default;

  explicit Scenario(const ScenarioConfig::ScenarioType& scenario_type)
      : scenario_type_(scenario_type) {}

  virtual ~Scenario() = default;

  virtual ScenarioConfig::ScenarioType scenario_type() const;

  virtual bool Init() = 0;

  virtual common::Status Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  virtual bool IsTransferable(const Scenario& current_scenario,
                              const common::TrajectoryPoint& ego_point,
                              const Frame& frame) const = 0;

  const ScenarioStatus& GetStatus() const { return status_; }

 protected:
  bool InitTasks(const ScenarioConfig& config,
                 const std::string& stage_name,
                 std::vector<std::unique_ptr<Task>>* tasks);

 protected:
  bool is_init_ = false;
  apollo::common::util::Factory<TaskType, Task> task_factory_;
  ScenarioStatus status_ = STATUS_UNKNOWN;
  const ScenarioConfig::ScenarioType scenario_type_;
};

}  // namespace planning
}  // namespace apollo
