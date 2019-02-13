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
#include <vector>

#include "Eigen/Eigen"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/open_space/trajectory_smoother/distance_approach_problem.h"
#include "modules/planning/open_space/trajectory_smoother/dual_variable_warm_start_problem.h"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

/*
Initially inspired by "Optimization-Based Collision Avoidance" from Xiaojing
Zhang, Alexander Linigerb and Francesco Borrellia
*/

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class OpenSpaceTrajectoryGenerator
 * @brief OpenSpaceTrajectoryGenerator is a derived class of Planner.
 *        It reads a recorded trajectory from a trajectory file and
 *        outputs proper segment of the trajectory according to vehicle
 * position.
 */
class OpenSpaceTrajectoryGenerator {
 public:
  /**
   * @brief Constructor
   */
  OpenSpaceTrajectoryGenerator() = default;

  /**
   * @brief Destructor
   */
  virtual ~OpenSpaceTrajectoryGenerator() = default;

  apollo::common::Status Init(
      const PlannerOpenSpaceConfig& planner_open_space_config);

  /**
   * @brief plan for open space trajectory generators.
   */
  apollo::common::Status Plan(
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      const apollo::common::VehicleState& vehicle_state,
      const std::vector<double>& XYbounds, const double& rotate_angle,
      const apollo::common::math::Vec2d& translate_origin,
      const std::vector<double>& end_pose,
      const Eigen::MatrixXi& obstacles_edges_num,
      const Eigen::MatrixXd& obstacles_A, const Eigen::MatrixXd& obstacles_b,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec);

  bool IsCollisionFreeTrajectory(const ADCTrajectory& adc_trajectory);

  void BuildPredictedEnvironment(const std::vector<const Obstacle*>& obstacles);

  void UpdateTrajectory(apollo::common::Trajectory* trajectory_to_end);

  void UpdateDebugInfo(
      ::apollo::planning_internal::OpenSpaceDebug* open_space_debug);

  void GetStitchingTrajectory(
      std::vector<common::TrajectoryPoint>* stitching_trajectory);

  void LoadTrajectory(const Eigen::MatrixXd& state_result_ds,
                      const Eigen::MatrixXd& control_result_ds,
                      const Eigen::MatrixXd& time_result_ds);

  void Stop();

  void RecordDebugInfo(const Eigen::MatrixXd& xWS, const Eigen::MatrixXd& uWs,
                       const Eigen::MatrixXd& l_warm_up,
                       const Eigen::MatrixXd& n_warm_up,
                       const Eigen::MatrixXd& dual_l_result_ds,
                       const Eigen::MatrixXd& dual_n_result_ds,
                       const Eigen::MatrixXd& state_result_ds,
                       const Eigen::MatrixXd& control_result_ds,
                       const Eigen::MatrixXd& time_result_ds,
                       const std::vector<double>& XYbounds,
                       const std::vector<std::vector<common::math::Vec2d>>&
                           obstacles_vertices_vec);

 private:
  bool IsInitPointNearDestination(
      const common::TrajectoryPoint& planning_init_point,
      const std::vector<double>& end_pose, const double& rotate_angle,
      const Vec2d& translate_origin);

  std::unique_ptr<::apollo::planning::HybridAStar> warm_start_;
  std::unique_ptr<::apollo::planning::DistanceApproachProblem>
      distance_approach_;
  std::unique_ptr<::apollo::planning::DualVariableWarmStartProblem>
      dual_variable_warm_start_;
  common::PathPoint init_state_;
  const common::VehicleParam& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  common::TrajectoryPoint planning_init_point_;
  std::vector<common::TrajectoryPoint> stitching_trajectory_;
  double init_x_ = 0.0;
  double init_y_ = 0.0;
  double init_phi_ = 0.0;
  double init_v_ = 0.0;
  double init_steer_ = 0.0;
  double init_a_ = 0.0;
  size_t horizon_ = 0;
  double ts_ = 0.0;
  Eigen::MatrixXd ego_;
  std::vector<double> XYbounds_;
  apollo::common::Trajectory trajectory_to_end_;
  apollo::planning_internal::OpenSpaceDebug open_space_debug_;
};

}  // namespace planning
}  // namespace apollo
