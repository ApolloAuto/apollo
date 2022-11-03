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
 * @file
 **/

#pragma once

#include <memory>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_optimizer.h"
#include "modules/planning/tasks/optimizers/trajectory_optimizer.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

struct OpenSpaceTrajectoryThreadData {
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  std::vector<double> end_pose;
  std::vector<double> XYbounds;
  double rotate_angle;
  apollo::common::math::Vec2d translate_origin;
  Eigen::MatrixXi obstacles_edges_num;
  Eigen::MatrixXd obstacles_A;
  Eigen::MatrixXd obstacles_b;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
};

class OpenSpaceTrajectoryProvider : public TrajectoryOptimizer {
 public:
  OpenSpaceTrajectoryProvider(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

  ~OpenSpaceTrajectoryProvider();

  void Stop();

  void Restart();

 private:
  apollo::common::Status Process() override;

  void GenerateTrajectoryThread();

  bool IsVehicleNearDestination(const common::VehicleState& vehicle_state,
                                const std::vector<double>& end_pose,
                                double rotate_angle,
                                const common::math::Vec2d& translate_origin);

  bool IsVehicleStopDueToFallBack(const bool is_on_fallback,
                                  const common::VehicleState& vehicle_state);

  void GenerateStopTrajectory(DiscretizedTrajectory* const trajectory_data);

  void LoadResult(DiscretizedTrajectory* const trajectory_data);

  void ReuseLastFrameResult(const Frame* last_frame,
                            DiscretizedTrajectory* const trajectory_data);

  void ReuseLastFrameDebug(const Frame* last_frame);

 private:
  bool thread_init_flag_ = false;

  std::unique_ptr<OpenSpaceTrajectoryOptimizer>
      open_space_trajectory_optimizer_;

  size_t optimizer_thread_counter = 0;

  OpenSpaceTrajectoryThreadData thread_data_;
  std::future<void> task_future_;
  std::atomic<bool> is_generation_thread_stop_{false};
  std::atomic<bool> trajectory_updated_{false};
  std::atomic<bool> data_ready_{false};
  std::atomic<bool> trajectory_error_{false};
  std::atomic<bool> trajectory_skipped_{false};
  std::mutex open_space_mutex_;
};

}  // namespace planning
}  // namespace apollo
