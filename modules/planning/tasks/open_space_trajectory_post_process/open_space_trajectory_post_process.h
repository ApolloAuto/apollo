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
#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"

#include "modules/planning/tasks/open_space_trajectory_post_process/proto/open_space_trajectory_post_process.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"
#include "modules/planning/planning_interface_base/task_base/common/trajectory_optimizer.h"

namespace apollo {
namespace planning {
class OpenSpaceTrajectoryPostProcess : public TrajectoryOptimizer {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

  ~OpenSpaceTrajectoryPostProcess() = default;

 private:
  common::Status Process() override;

  void InterpolateTrajectory(
      const DiscretizedTrajectory& stitched_trajectory_result,
      DiscretizedTrajectory* interpolated_trajectory);

  void UpdateVehicleInfo();

  bool EncodeTrajectory(const DiscretizedTrajectory& trajectory,
                        std::string* const encoding);

  bool CheckTrajTraversed(const std::string& trajectory_encoding_to_check);

  void UpdateTrajHistory(const std::string& chosen_trajectory_encoding);

  void PartitionTrajectory(const DiscretizedTrajectory& trajectory,
                           std::vector<TrajGearPair>* partitioned_trajectories);

  void LoadTrajectoryPoint(const common::TrajectoryPoint& trajectory_point,
                           const bool is_trajectory_last_point,
                           const canbus::Chassis::GearPosition& gear,
                           common::math::Vec2d* last_pos_vec,
                           double* distance_s,
                           DiscretizedTrajectory* current_trajectory);

  bool CheckReachTrajectoryEnd(const DiscretizedTrajectory& trajectory,
                               const canbus::Chassis::GearPosition& gear,
                               const size_t trajectories_size,
                               size_t* current_trajectory_point_index);

  bool UseFailSafeSearch(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const std::vector<std::string>& trajectories_encodings,
      size_t* current_trajectory_index, size_t* current_trajectory_point_index);

  bool InsertGearShiftTrajectory(
      const bool flag_change_to_next, const size_t current_trajectory_index,
      const std::vector<TrajGearPair>& partitioned_trajectories,
      TrajGearPair* gear_switch_idle_time_trajectory);

  void GenerateGearShiftTrajectory(
      const canbus::Chassis::GearPosition& gear_position, double init_kappa,
      TrajGearPair* gear_switch_idle_time_trajectory);
  void GenerateStopTrajectory(
      const canbus::Chassis::GearPosition& gear_position,
      TrajGearPair* stop_trajectory);
  void AdjustRelativeTimeAndS(
      const std::vector<TrajGearPair>& partitioned_trajectories,
      const size_t current_trajectory_index,
      const size_t closest_trajectory_point_index,
      TrajGearPair* current_partitioned_trajectory);

  bool CheckArrivePoint(
      const canbus::Chassis::GearPosition& gear,
      const apollo::common::PathPoint& path_end_point,
      const double& lateral_offset_to_midpoint,
      const double& longitudinal_offset_to_midpoint,
      const double& heading_offset_to_midpoint);

 private:
  OpenSpaceTrajectoryPostProcessConfig config_;
  double heading_search_range_ = 0.0;
  double heading_track_range_ = 0.0;
  double distance_search_range_ = 0.0;
  double heading_offset_to_midpoint_ = 0.0;
  double lateral_offset_to_midpoint_ = 0.0;
  double longitudinal_offset_to_midpoint_ = 0.0;
  double vehicle_box_iou_threshold_to_midpoint_ = 0.0;
  double scale_destination_ = 0.0;
  int stop_check_window_ = 0;
  int stop_check_count_ = 0;

  common::VehicleParam vehicle_param_;
  double ego_length_ = 0.0;
  double ego_width_ = 0.0;
  double shift_distance_ = 0.0;
  double wheel_base_ = 0.0;

  double ego_theta_ = 0.0;
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  double ego_v_ = 0.0;
  apollo::canbus::Chassis_GearPosition ego_gear_;
  common::math::Box2d ego_box_;
  double vehicle_moving_direction_ = 0.0;
  int last_index_ = -1;
  size_t current_trajectory_index_ = 0;
  bool fail_search_fallback_ = false;
  double last_time_ = 0;
  struct pair_comp_ {
    bool operator()(
        const std::pair<std::pair<size_t, size_t>, double>& left,
        const std::pair<std::pair<size_t, size_t>, double>& right) const {
      return left.second <= right.second;
    }
  };
  struct comp_ {
    bool operator()(const std::pair<size_t, double>& left,
                    const std::pair<size_t, double>& right) {
      return left.second <= right.second;
    }
  };
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::OpenSpaceTrajectoryPostProcess, Task)

}  // namespace planning
}  // namespace apollo
