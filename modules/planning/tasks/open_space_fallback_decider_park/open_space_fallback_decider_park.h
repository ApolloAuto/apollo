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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/planning/tasks/open_space_fallback_decider_park/proto/open_space_fallback_decider_park.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"
#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planning_interface_base/task_base/common/decider.h"
#include "modules/planning/planning_open_space/utils/open_space_fallback_util.h"

namespace apollo {
namespace planning {
class OpenSpaceFallbackDeciderPark : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(Frame* frame) override;

  // bool IsCollisionFreeTrajectory(const ADCTrajectory& trajectory_pb);

  bool IsCollisionFreeTrajectory(
      const TrajGearPair& trajectory_pb,
      const std::vector<std::vector<common::math::Box2d>>&
          predicted_bounding_rectangles,
      const std::vector<Obstacle*>& static_obstacles, int* current_idx,
      int* first_collision_idx, bool& is_collision_with_static_obstacle,
      bool& is_collision_with_dynamic_obstacle);

  bool QuardraticFormulaLowerSolution(const double a, const double b,
                                      const double c, double* sol);

  bool IsCollisionFreeEgoBox();

  void CalculateFallbackTrajectory(
      const TrajGearPair& ChosenPartitionedTrajectory,
      const double& deceleration, const double& relative_time_interval,
      TrajGearPair* traj_gear_pair);

  void CalculateFallbackTrajectoryWithTrajectoryPose(
    const TrajGearPair& ChosenPartitionedTrajectory, const apollo::common::TrajectoryPoint start_point, const apollo::common::TrajectoryPoint end_point, const double& stop_distance,
    const double& relative_time_interval, TrajGearPair* traj_gear_pair);

 private:
  OpenSpaceFallBackDeciderParkConfig config_;
  double open_space_fallback_collision_distance_ = 0;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::OpenSpaceFallbackDeciderPark,
                                     Task)

}  // namespace planning
}  // namespace apollo
