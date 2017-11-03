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

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/common/obstacle.h"

namespace apollo {
namespace planning {

class ADCNeighborhood {
 public:
  ADCNeighborhood(Frame* frame,
      const apollo::common::TrajectoryPoint& planning_init_point,
      const ReferenceLine& reference_line);

  bool ForwardNearestObstacle(
      std::array<double, 3>* forward_nearest_obstacle_state,
      double* enter_time);

  bool BackwardNearestObstacle(
      std::array<double, 3>* backward_nearest_obstacle_state,
      double* enter_time);

  bool IsInNeighborhood(const Obstacle& obstacle);

 private:
  void InitNeighborhood(Frame* frame,
      const apollo::common::TrajectoryPoint& planning_init_point,
      const ReferenceLine& reference_line);

  void SetupObstacles(Frame* frame,
      const ReferenceLine& reference_line);

  void SetupADC(Frame* frame,
      const apollo::common::TrajectoryPoint& planning_init_point,
      const ReferenceLine& reference_line);

 private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  // array of [t, start_s, end_s, s_dot, s_dotdot]
  std::vector<std::array<double, 5>> neighborhood_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_ADC_NEIGHBORHOOD_H_
