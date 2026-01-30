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

#include <memory>
#include <vector>

#include "Eigen/Eigen"

#ifdef ALIVE
#undef ALIVE
#endif

#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/planning_base/common/open_space_info.h"
#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"
#include "modules/planning/planning_open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/planning_open_space/trajectory_smoother/distance_approach_problem.h"
#include "modules/planning/planning_open_space/trajectory_smoother/dual_variable_warm_start_problem.h"
#include "modules/planning/planning_open_space/trajectory_smoother/iterative_anchoring_smoother.h"
#include "modules/planning/planning_open_space/utils/open_space_trajectory_optimizer_util.h"
#include "modules/planning/tasks/open_space_trajectory_optimizer_park/proto/open_space_trajectory_optimizer_park.pb.h"

namespace apollo {
namespace planning {
class Optimizer {
 public:
  Optimizer(
      const OpenSpaceTrajectoryOptimizerParkConfig& config);

  virtual ~Optimizer() = default;

  common::Status Plan(const OpenSpaceInfo open_space_info);

  void GetOptimizedTrajectory(DiscretizedTrajectory& optimized_trajectory) {
    optimized_trajectory.clear();
    optimized_trajectory = optimized_trajectory_;
  }

  void UpdateDebugInfo(
    planning_internal::OpenSpaceDebug* open_space_debug);

 private:
  void LoadTrajectoryInEigen(
      DiscretizedTrajectory& result,
      Eigen::MatrixXd& xWS, Eigen::MatrixXd& uWS);

  void PartitionTrajectory(
      const DiscretizedTrajectory& raw_trajectory,
      std::vector<DiscretizedTrajectory>* partitioned_trajectories);

  void CombineTrajectories(
    const std::vector<Eigen::MatrixXd>& xWS_vec,
    const std::vector<Eigen::MatrixXd>& uWS_vec,
    const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& l_warm_up_vec,
    const std::vector<Eigen::MatrixXd>& n_warm_up_vec,
    const std::vector<Eigen::MatrixXd>& dual_l_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& dual_n_result_ds_vec,
    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds);

  bool GenerateDistanceApproachTraj(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const std::vector<double>& XYbounds,
    const Eigen::MatrixXi& obstacles_edges_num,
    const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    const Eigen::MatrixXd& last_time_u, const double init_v,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* l_warm_up,
    Eigen::MatrixXd* n_warm_up, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds);

  bool GenerateDecoupledTraj(
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc);

  void UseWarmStartAsResult(
    const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWS,
    const Eigen::MatrixXd& l_warm_up, const Eigen::MatrixXd& n_warm_up,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds, Eigen::MatrixXd* dual_l_result_ds,
    Eigen::MatrixXd* dual_n_result_ds);

  void LoadTrajectory(
    const Eigen::MatrixXd& state_result, const Eigen::MatrixXd& control_result,
    const Eigen::MatrixXd& time_result);

  void LoadResult(
    const DiscretizedTrajectory& discretized_trajectory,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc);

  void RecordDebugInfo(
    const common::math::Vec2d& translate_origin, const double rotate_angle,
    const std::vector<double>& end_pose, const Eigen::MatrixXd& xWS,
    const Eigen::MatrixXd& uWS, const Eigen::MatrixXd& l_warm_up,
    const Eigen::MatrixXd& n_warm_up, const Eigen::MatrixXd& dual_l_result_ds,
    const Eigen::MatrixXd& dual_n_result_ds,
    const Eigen::MatrixXd& state_result_ds,
    const Eigen::MatrixXd& control_result_ds,
    const Eigen::MatrixXd& time_result_ds,
    const std::vector<double> XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>&
        obstacles_vertices_vec);

 private:
  OpenSpaceTrajectoryOptimizerParkConfig config_;
  DiscretizedTrajectory optimized_trajectory_;
  std::unique_ptr<DistanceApproachProblem> distance_approach_;
  std::unique_ptr<DualVariableWarmStartProblem> dual_variable_warm_start_;
  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;
  apollo::planning_internal::OpenSpaceDebug open_space_debug_;
};
}  // namespace planning
}  // namespace apollo
