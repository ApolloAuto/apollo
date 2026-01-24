/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include <memory>

#include "modules/planning/tasks/open_space_trajectory_optimizer_park/proto/open_space_trajectory_optimizer_park.pb.h"
#include "modules/planning/tasks/open_space_trajectory_optimizer_park/optimizer.h"
#include "modules/planning/planning_interface_base/task_base/task.h"
#include "modules/planning/planning_interface_base/task_base/common/trajectory_optimizer.h"

#include "cyber/common/log.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

class OpenSpaceTrajectoryOptimizerPark : public TrajectoryOptimizer {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;
  ~OpenSpaceTrajectoryOptimizerPark();

 private:
  void Stop();

  apollo::common::Status Process() override;

  void Optimize();

  void GenerateTrajectoryThread();

  void LoadResult(
      DiscretizedTrajectory* const trajectory_data);

  void ReuseLastFrameDebug(const Frame* last_frame);

 private:
  std::unique_ptr<Optimizer> trajectory_optimizer_;
  OpenSpaceTrajectoryOptimizerParkConfig config_;
  bool thread_init_flag_ = false;
  bool replan_flag_ = false;
  std::future<void> task_future_;
  bool trajectory_update_ = false;
  std::mutex data_mutex_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::OpenSpaceTrajectoryOptimizerPark, Task)

}  // namespace planning
}  // namespace apollo
