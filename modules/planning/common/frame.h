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
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/reference_line_smoother_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class Frame {
 public:
  explicit Frame(const uint32_t sequence_num);

  void SetRoutingResponse(const routing::RoutingResponse &routing);
  void SetPrediction(const prediction::PredictionObstacles &prediction);
  void SetPlanningStartPoint(const common::TrajectoryPoint &start_point);
  void SetVehicleInitPose(const localization::Pose &pose);
  const common::TrajectoryPoint &PlanningStartPoint() const;
  bool Init(const PlanningConfig &config);

  static const hdmap::PncMap *PncMap();
  static void SetMap(hdmap::PncMap *pnc_map);

  uint32_t sequence_num() const;

  std::string DebugString() const;

  const IndexedObstacles &GetObstacles() const;

  const ADCTrajectory &GetADCTrajectory() const;
  ADCTrajectory *MutableADCTrajectory();

  void SetComputedTrajectory(const PublishableTrajectory &trajectory);
  const PublishableTrajectory &computed_trajectory() const;
  const localization::Pose &VehicleInitPose() const;

  const routing::RoutingResponse &routing_response() const;

  void RecordInputDebug();

  void AlignPredictionTime(const double trajectory_header_time);

  std::vector<ReferenceLineInfo> &reference_line_info();
  const std::vector<ReferenceLineInfo> &reference_line_info() const;

  bool AddObstacle(std::unique_ptr<Obstacle> obstacle);

 private:
  /**
   * @brief This is the function that can create one reference lines
   * from routing result.
   * In current implementation, only one reference line will be returned.
   * But this is in sufficient when multiple driving options exist.
   *
   * TODO create multiple reference_lines from this function.
   */
  bool CreateReferenceLineFromRouting(
      const common::PointENU &position, const routing::RoutingResponse &routing,
      std::vector<ReferenceLine> *reference_lines);

  /**
   * @brief create obstacles from prediction input.
   * @param prediction the received prediction result.
   */
  void CreatePredictionObstacles(
      const prediction::PredictionObstacles &prediction);

  bool InitReferenceLineInfo(const std::vector<ReferenceLine> &reference_lines);

 private:
  common::TrajectoryPoint planning_start_point_;

  std::vector<ReferenceLineInfo> reference_line_info_;

  routing::RoutingResponse routing_response_;
  prediction::PredictionObstacles prediction_;
  IndexedObstacles obstacles_;
  uint32_t sequence_num_ = 0;
  localization::Pose init_pose_;
  static const hdmap::PncMap *pnc_map_;
  ReferenceLineSmootherConfig smoother_config_;

  ADCTrajectory trajectory_pb_;  // planning output pb
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
