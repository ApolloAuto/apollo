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

/**
 * @file trajectory_optimizer.h
 **/

#pragma once

#include <memory>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class TrajectoryOptimizer : public Task {
 public:
  explicit TrajectoryOptimizer(const TaskConfig &config);
  TrajectoryOptimizer(const TaskConfig &config,
                      const std::shared_ptr<DependencyInjector> &injector);
  virtual ~TrajectoryOptimizer() = default;

  apollo::common::Status Execute(Frame *frame) override;

 protected:
  virtual apollo::common::Status Process() = 0;
};

}  // namespace planning
}  // namespace apollo
