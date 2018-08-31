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

#ifndef MODULES_PLANNING_COMMON_FRAME_OPEN_SPACE_H_
#define MODULES_PLANNING_COMMON_FRAME_OPEN_SPACE_H_

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/lag_prediction.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

/**
 * @class FrameOpenSpace
 *
 * @brief FrameOpenSpace holds all data for one planning cycle.
 */

class FrameOpenSpace {
 public:
  explicit FrameOpenSpace(uint32_t sequence_num,
                          const common::TrajectoryPoint &planning_start_point,
                          const double start_time,
                          const common::VehicleState &vehicle_state);

  common::Status Init();

  const common::TrajectoryPoint &PlanningStartPoint() const;

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  Obstacle *Find(const std::string &id);

  const std::vector<const Obstacle *> obstacles() const;

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }

  const ADCTrajectory &trajectory() const { return trajectory_; }

  const bool is_near_destination() const { return is_near_destination_; }

 private:
  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such
   * obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;

  void AddObstacle(const Obstacle &obstacle);

 private:
  uint32_t sequence_num_ = 0;
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  const double start_time_;
  common::VehicleState vehicle_state_;
  bool is_near_destination_ = false;
  prediction::PredictionObstacles prediction_;
  ThreadSafeIndexedObstacles obstacles_;
  ChangeLaneDecider change_lane_decider_;
  ADCTrajectory trajectory_;  // last published trajectory
  std::unique_ptr<LagPrediction> lag_predictor_;
  apollo::common::monitor::MonitorLogger monitor_logger_;
};

class FrameOpenSpaceHistory : public IndexedQueue<uint32_t, FrameOpenSpace> {
 private:
  DECLARE_SINGLETON(FrameOpenSpaceHistory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_OPEN_SPACE_H_
