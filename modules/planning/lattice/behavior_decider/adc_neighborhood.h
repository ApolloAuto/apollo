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

#ifndef MODULES_PLANNING_LATTICE_ADC_NEIGHBORHOOD_H_
#define MODULES_PLANNING_LATTICE_ADC_NEIGHBORHOOD_H_

#include <array>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/common/obstacle.h"

namespace apollo {
namespace planning {

class ADCNeighborhood {
 public:
  ADCNeighborhood(
      const Frame* frame,
      const apollo::common::TrajectoryPoint& planning_init_point,
      const ReferenceLine& reference_line);

  bool ForwardNearestObstacle(
      std::array<double, 3>* forward_nearest_obstacle_state,
      double* enter_time);

  bool BackwardNearestObstacle(
      std::array<double, 3>* backward_nearest_obstacle_state,
      double* enter_time);

  void GetCriticalConditions(
      std::vector<CriticalCondition>* critical_conditions);

  bool GetCriticalCondition(
      const std::string& obstacle_id,
      CriticalCondition* critical_condition);

  bool IsInNeighborhood(const Obstacle* obstacle) const;

  bool IsForward(const Obstacle* obstacle) const;

  bool IsBackward(const Obstacle* obstacle) const;

 private:
  void InitNeighborhood(
      const Frame* frame,
      const apollo::common::TrajectoryPoint& planning_init_point,
      const ReferenceLine& reference_line);

  void SetupObstacles(
      const Frame* frame,
      const ReferenceLine& reference_line);

  void SetupADC(
      const Frame* frame,
      const apollo::common::TrajectoryPoint& planning_init_point,
      const ReferenceLine& reference_line);

  double SpeedOnReferenceLine(
      const std::vector<apollo::common::PathPoint>& discretized_ref_points,
      const Obstacle* obstacle,
      const SLBoundary& sl_boundary);

  void SetCriticalPoint(
      const double t, const double s, const double v,
      CriticalPoint* critical_point);

 private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  // array of [t, start_s, end_s, s_dot, s_dotdot]
  std::vector<std::array<double, 5>> forward_neighborhood_;
  // array of [t, start_s, end_s, s_dot, s_dotdot]
  std::vector<std::array<double, 5>> backward_neighborhood_;
  // obstacle_id -> critical conditions
  std::unordered_map<std::string, CriticalCondition> critical_conditions_;

  std::unordered_set<std::string> forward_obstacle_id_set_;
  std::unordered_set<std::string> backward_obstacle_id_set_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_ADC_NEIGHBORHOOD_H_
