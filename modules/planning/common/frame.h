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
#include "modules/map/proto/routing.pb.h"
#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/planning_data.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace planning {

using ObstacleTable = IndexedList<uint32_t, Obstacle>;

class Frame {
 public:
  Frame(const uint32_t sequence_num);

  void SetRoutingResult(const hdmap::RoutingResult &routing);
  void SetPrediction(const prediction::PredictionObstacles &prediction);
  void SetPlanningStartPoint(const TrajectoryPoint &start_point);
  void SetVehicleInitPose(const localization::Pose &pose);
  const TrajectoryPoint &PlanningStartPoint() const;
  bool Init();

  static const hdmap::PncMap *PncMap();
  static void SetMap(hdmap::PncMap *pnc_map);

  uint32_t sequence_num() const;
  const PlanningData &planning_data() const;

  PlanningData *mutable_planning_data();
  std::string DebugString() const;

  const ObstacleTable &GetObstacleTable() const;
  ObstacleTable *MutableObstacleTable();

  void set_computed_trajectory(const PublishableTrajectory &trajectory);
  const PublishableTrajectory &computed_trajectory() const;

 private:
  bool CreateReferenceLineFromRouting();
  bool SmoothReferenceLine();
  void SetDecisionDataFromPrediction(
      const prediction::PredictionObstacles &prediction_obstacles);

 private:
  TrajectoryPoint planning_start_point_;

  hdmap::RoutingResult routing_result_;
  prediction::PredictionObstacles prediction_;
  ObstacleTable obstacle_table_;
  uint32_t sequence_num_ = 0;
  hdmap::Path hdmap_path_;
  localization::Pose init_pose_;
  PublishableTrajectory _computed_trajectory;
  PlanningData _planning_data;
  static const hdmap::PncMap *pnc_map_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
