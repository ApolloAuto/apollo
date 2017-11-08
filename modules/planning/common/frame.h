/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file frame.h
 **/

#ifndef MODULES_PLANNING_COMMON_FRAME_H_
#define MODULES_PLANNING_COMMON_FRAME_H_

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

class Frame {
 public:
  explicit Frame(uint32_t sequence_num,
                 const common::TrajectoryPoint &planning_start_point,
                 const common::VehicleState &vehicle_state);

  void SetPrediction(const prediction::PredictionObstacles &prediction);
  const common::TrajectoryPoint &PlanningStartPoint() const;
  common::Status Init();

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  std::list<ReferenceLineInfo> &reference_line_info();

  void AddObstacle(const Obstacle &obstacle);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();
  const ReferenceLineInfo *DriveReferenceLinfInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const Obstacle *AddStaticVirtualObstacle(const std::string &id,
                                           const common::math::Box2d &box);

  bool Rerouting();

 private:
  /**
   * @brief create obstacles from prediction input.
   * @param prediction the received prediction result.
   */
  void CreatePredictionObstacles(
      const prediction::PredictionObstacles &prediction);

  bool InitReferenceLineInfo();

  void AlignPredictionTime(const double trajectory_header_time);

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if such
   * obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;

  const Obstacle *CreateDestinationObstacle();

 private:
  uint32_t sequence_num_ = 0;
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;

  /**
   * the reference line info that the vehicle finally choose to drive on.
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  prediction::PredictionObstacles prediction_;

  ThreadSafeIndexedObstacles obstacles_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
