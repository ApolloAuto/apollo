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
#include <memory>
#include <string>

#include "modules/common/proto/geometry.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/planning_data.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace planning {

using Obstacles = IndexedList<std::string, Obstacle>;
using PathObstacles = IndexedList<std::string, PathObstacle>;

class Frame {
 public:
  Frame(const uint32_t sequence_num);

  void SetRoutingResponse(const routing::RoutingResponse &routing);
  void SetPrediction(const prediction::PredictionObstacles &prediction);
  void SetPlanningStartPoint(const common::TrajectoryPoint &start_point);
  void SetVehicleInitPose(const localization::Pose &pose);
  const common::TrajectoryPoint &PlanningStartPoint() const;
  bool Init();

  bool AddDecision(const std::string &tag, const std::string &object_id,
                   const ObjectDecisionType &decision);

  static const hdmap::PncMap *PncMap();
  static void SetMap(hdmap::PncMap *pnc_map);

  const ReferenceLine &reference_line() const;

  uint32_t sequence_num() const;
  const PlanningData &planning_data() const;

  PlanningData *mutable_planning_data();
  std::string DebugString() const;

  const Obstacles &GetObstacles() const;
  Obstacles *MutableObstacles();

  void set_computed_trajectory(const PublishableTrajectory &trajectory);
  const PublishableTrajectory &computed_trajectory() const;

 private:
  bool CreateReferenceLineFromRouting();
  bool SmoothReferenceLine();
  void CreateObstacles(const prediction::PredictionObstacles &prediction);

 private:
  common::TrajectoryPoint planning_start_point_;

  routing::RoutingResponse routing_result_;
  prediction::PredictionObstacles prediction_;
  Obstacles obstacles_;
  PathObstacles path_obstacles_;
  uint32_t sequence_num_ = 0;
  hdmap::Path hdmap_path_;
  localization::Pose init_pose_;
  PublishableTrajectory _computed_trajectory;
  std::unique_ptr<ReferenceLine> reference_line_ = nullptr;
  PlanningData _planning_data;
  static const hdmap::PncMap *pnc_map_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
