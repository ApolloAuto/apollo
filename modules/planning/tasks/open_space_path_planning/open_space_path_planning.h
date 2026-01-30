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
#include <vector>
#include "modules/planning/tasks/open_space_path_planning/proto/open_space_path_planning.pb.h"

#include "cyber/common/log.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/planning_open_space/utils/open_space_trajectory_optimizer_util.h"
#include "modules/planning/planning_interface_base/task_base/common/trajectory_optimizer.h"

namespace apollo {
namespace planning {

struct OpenSpacePathPlanningThreadData {
  std::vector<double> start_pose;
  std::vector<double> end_pose;
  std::vector<double> XYbounds;
  double rotate_angle;
  apollo::common::math::Vec2d translate_origin;
  Eigen::MatrixXi obstacles_edges_num;
  Eigen::MatrixXd obstacles_A;
  Eigen::MatrixXd obstacles_b;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
    std::vector<std::vector<common::math::Vec2d>> soft_boundary_vertices_vec;
  bool reeds_sheep_last_straight;
};

class OpenSpacePathPlanning : public TrajectoryOptimizer {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;
  ~OpenSpacePathPlanning();

 private:
  void Stop();
  apollo::common::Status Process() override;
  void PathPlanning();
  void GeneratePathThread();
  void LoadResult(DiscretizedTrajectory* const trajectory_data);
  OpenSpacePathPlanningConfig config_;
  OpenSpacePathPlanningThreadData thread_data_;
  std::future<void> task_future_;
  std::atomic<bool> thread_init_flag_{false};
  std::atomic<bool> data_ready_{false};
  std::atomic<bool> path_update_{false};
  std::mutex data_mutex_;
  std::unique_ptr<HybridAStar> path_finder_;
  HybridAStartResult result_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::OpenSpacePathPlanning, Task)

}  // namespace planning
}  // namespace apollo
