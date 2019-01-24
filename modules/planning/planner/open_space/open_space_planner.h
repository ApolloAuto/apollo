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

#include "Eigen/Eigen"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/open_space/constraints_formulation/open_space_ROI.h"
#include "modules/planning/open_space/open_space_trajectory_generator.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

/*
Initially inspired by "Optimization-Based Collision Avoidance"
(arXiv:1711.03449) from Xiaojing Zhang , Alexander Liniger and Francesco
Borrelli
*/

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

struct OpenSpaceThreadData {
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  apollo::common::VehicleState vehicle_state;
  double rotate_angle;
  apollo::common::math::Vec2d translate_origin;
  std::vector<double> end_pose;
  Eigen::MatrixXi obstacles_edges_num;
  Eigen::MatrixXd obstacles_A;
  Eigen::MatrixXd obstacles_b;
  std::vector<double> XYbounds;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
};

/**
 * @class OpenSpacePlanner
 * @brief OpenSpacePlanner is a derived class of Planner.
 *        It reads a recorded trajectory from a trajectory file and
 *        outputs proper segment of the trajectory according to vehicle
 * position.
 */

class OpenSpacePlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  OpenSpacePlanner() = default;

  /**
   * @brief Destructor
   */
  virtual ~OpenSpacePlanner() = default;

  std::string Name() override { return "OPEN_SPACE"; }

  apollo::common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief override function Plan in parent class Planner.
   */
  apollo::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point,
      Frame* frame) override {
    return Status::OK();
  }

  apollo::common::Status Plan(
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      Frame* frame);

  void GenerateTrajectoryThread();

  void Stop() override;

 private:
  void LoadTrajectoryToFrame(Frame* frame);
  bool IsVehicleNearDestination(const common::VehicleState& vehicle_state,
                                const std::vector<double>& end_pose,
                                const double& rotate_angle,
                                const Vec2d& translate_origin);

 private:
  PlannerOpenSpaceConfig planner_open_space_config_;
  DistanceApproachConfig distance_approach_config_;
  std::unique_ptr<OpenSpaceTrajectoryGenerator>
      open_space_trajectory_generator_;
  std::unique_ptr<OpenSpaceROI> open_space_roi_generator_;

  std::vector<common::TrajectoryPoint> stitching_trajectory_;
  apollo::common::VehicleState vehicle_state_;
  double rotate_angle_;
  apollo::common::math::Vec2d translate_origin_;
  std::vector<double> end_pose_;
  size_t obstacles_num_;
  Eigen::MatrixXi obstacles_edges_num_;
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec_;
  std::vector<double> XYbounds_;

  planning_internal::OpenSpaceDebug open_space_debug_;
  apollo::common::Trajectory trajectory_to_end_;
  ADCTrajectory trajectory_to_end_pb_;

  OpenSpaceThreadData thread_data_;
  std::future<void> task_future_;
  std::atomic<bool> is_stop_{false};
  std::atomic<bool> trajectory_updated_{false};
  std::mutex open_space_mutex_;
};

}  // namespace planning
}  // namespace apollo
