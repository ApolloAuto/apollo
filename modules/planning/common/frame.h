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
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/qp_spline_reference_line_smoother_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/status/status.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

class Frame {
 public:
  explicit Frame(const uint32_t sequence_num);

  // functions called out of optimizers
  void SetPrediction(const prediction::PredictionObstacles &prediction);
  void SetPlanningStartPoint(const common::TrajectoryPoint &start_point);
  void SetVehicleInitPose(const localization::Pose &pose);
  const common::TrajectoryPoint &PlanningStartPoint() const;
  common::Status Init(const PlanningConfig &config,
                      const double current_time_stamp);

  static void SetMap(const hdmap::HDMap *pnc_map);

  uint32_t SequenceNum() const;

  void UpdateRoutingResponse(const routing::RoutingResponse &routing);

  const routing::RoutingResponse &routing_response() const;

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

  static bool Rerouting();

 private:
  /**
   * @brief This is the function that can create one reference lines
   * from routing result.
   * @param position: the position near routing ( distance < 20m in current
   * config).
   * @param reference_line return the reference line
   * @param segments : return the connected lanes corresponding to each
   * reference line.
   * @return true if at least one reference line is successfully created.
   */
  bool CreateReferenceLineFromRouting(
      const common::PointENU &position,
      std::list<ReferenceLine> *reference_lines,
      std::list<hdmap::RouteSegments> *segments);

  /**
   * @brief create obstacles from prediction input.
   * @param prediction the received prediction result.
   */
  void CreatePredictionObstacles(
      const prediction::PredictionObstacles &prediction);

  bool InitReferenceLineInfo();

  void AlignPredictionTime(const double trajectory_header_time);

  /**
   * Check if there is collision with obstacles
   */
  bool CheckCollision();

  const Obstacle *CreateDestinationObstacle();

 private:
  common::TrajectoryPoint planning_start_point_;

  std::list<ReferenceLineInfo> reference_line_info_;

  /**
   * the reference line info that the vehicle finally choose to drive on.
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  prediction::PredictionObstacles prediction_;

  ThreadSafeIndexedObstacles obstacles_;

  uint32_t sequence_num_ = 0;
  localization::Pose init_pose_;
  static std::unique_ptr<hdmap::PncMap> pnc_map_;
  QpSplineReferenceLineSmootherConfig smoother_config_;

  std::string collision_obstacle_id_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
