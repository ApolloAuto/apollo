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
#include <tuple>
#include <vector>

#include "cyber/common/macros.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decider_config.pb.h"

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/math/finite_element_qp/fem_1d_expanded_jerk_qp_problem.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class SidePassPathDecider : public Decider {
 public:
  explicit SidePassPathDecider(const TaskConfig& config);

  enum class SidePassDirection {
    LEFT = 0,
    RIGHT = 1,
  };

 private:
  void InitSolver();
  common::Status Process(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) override;

  bool DecideSidePassDirection(const std::vector<bool>& can_side_pass,
                               size_t left_length, size_t right_length);

  bool GetLaneInfoFromPoint(
    double point_x, double point_y, double point_z, double point_theta,
    hdmap::LaneInfoConstPtr* const lane);

  bool GeneratePath(Frame* const frame,
                    ReferenceLineInfo* const reference_line_info);

  std::vector<std::tuple<double, double, double>> GetPathBoundaries(
      const common::TrajectoryPoint& planning_start_point,
      const SLBoundary& adc_sl_boundary, const ReferenceLine& reference_line,
      const IndexedList<std::string, Obstacle>& indexed_obstacles,
      bool* fail_to_find_boundary);

  bool TrimGeneratedPath(
      std::vector<common::FrenetFramePoint>* ptr_frenet_frame_path);

  const Obstacle* GetNearestObstacle(
      const SLBoundary& adc_sl_boundary, const ReferenceLine& reference_line,
      const IndexedList<std::string, Obstacle>& indexed_obstacles);

  void RecordDebugInfo(ReferenceLineInfo* const reference_line_info);

 private:
  common::TrajectoryPoint adc_planning_start_point_;
  common::FrenetFramePoint adc_frenet_frame_point_;
  std::unique_ptr<Fem1dQpProblem> fem_qp_;
  SidePassDirection decided_direction_ = SidePassDirection::LEFT;
  double delta_s_ = 0.0;
  double total_path_length_ = 0.0;

  hdmap::Lane curr_lane_;
  const Obstacle* nearest_obstacle_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
