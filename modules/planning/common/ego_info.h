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
 * @file ego_info.h
 **/

#pragma once

#include <vector>

#include "cyber/common/macros.h"

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class EgoInfo {
 public:
  ~EgoInfo() = default;

  bool Update(const common::TrajectoryPoint& start_point,
              const common::VehicleState& vehicle_state);

  void Clear();

  common::TrajectoryPoint start_point() const { return start_point_; }

  common::VehicleState vehicle_state() const { return vehicle_state_; }

  double front_clear_distance() const { return front_clear_distance_; }

  common::math::Box2d ego_box() const { return ego_box_; }

  void CalculateFrontObstacleClearDistance(
      const std::vector<const Obstacle*>& obstacles);

 private:
  FRIEND_TEST(EgoInfoTest, EgoInfoSimpleTest);

  void set_vehicle_state(const common::VehicleState& vehicle_state) {
    vehicle_state_ = vehicle_state;
  }

  void set_start_point(const common::TrajectoryPoint& start_point) {
    start_point_ = start_point;
    const auto& param = ego_vehicle_config_.vehicle_param();
    start_point_.set_a(
        std::fmax(std::fmin(start_point_.a(), param.max_acceleration()),
                  param.max_deceleration()));
  }

  void CalculateEgoBox(const common::VehicleState& vehicle_state);

  // stitched point (at stitching mode)
  // or real vehicle point (at non-stitching mode)
  common::TrajectoryPoint start_point_;

  // ego vehicle state
  common::VehicleState vehicle_state_;

  double front_clear_distance_ = FLAGS_default_front_clear_distance;

  common::VehicleConfig ego_vehicle_config_;

  common::math::Box2d ego_box_;

  DECLARE_SINGLETON(EgoInfo)
};

}  // namespace planning
}  // namespace apollo
