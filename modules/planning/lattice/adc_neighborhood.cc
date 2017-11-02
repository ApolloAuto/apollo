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

#include "modules/planning/lattice/adc_neighborhood.h"

#include <utility>
#include <vector>
#include <cmath>

#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/lattice/lattice_params.h"
#include "modules/planning/lattice/reference_line_matcher.h"
#include "modules/planning/lattice/lattice_util.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::perception::PerceptionObstacle;

ADCNeighborhood::ADCNeighborhood(Frame* frame,
    const ReferenceLine& reference_line) {
  reference_line_ = reference_line;
  InitNeighborhood(frame);
}

void ADCNeighborhood::InitNeighborhood(Frame* frame) {
  const std::vector<const Obstacle*>& obstacles = frame->obstacles();
  std::vector<PathPoint> discretized_ref_points =
      ToDiscretizedReferenceLine(reference_line_.reference_points());
  for (const Obstacle* obstacle : obstacles) {
    double relative_time = 0.0;
    while (relative_time < planned_trajectory_time) {
      if (obstacle->Trajectory().trajectory_point_size() == 0) {
        continue;
      }
      common::TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      common::math::Box2d box = obstacle->GetBoundingBox(point);
      bool overlap = reference_line_.HasOverlap(box);
      if (overlap) {
        std::array<double, 4> obstacle_state;
        obstacle_state[0] = relative_time;
        SLBoundary sl_boundary;
        reference_line_.GetSLBoundary(box, &sl_boundary);
        obstacle_state[1] = sl_boundary.start_s();
        PathPoint obstacle_point_on_ref_line =
            ReferenceLineMatcher::match_to_reference_line(
                discretized_ref_points, obstacle_state[1]);
        const PerceptionObstacle& perception_obstacle = obstacle->Perception();
        auto velocity = perception_obstacle.velocity();
        double speed = std::hypot(velocity.x(), velocity.y());
        double obstacle_theta = std::atan2(velocity.y(), velocity.x());
        double diff_theta =
            obstacle_theta - obstacle_point_on_ref_line.theta();
        obstacle_state[2] = speed * std::cos(diff_theta);
        neighborhood_.push_back(std::move(obstacle_state));
      }
      relative_time += trajectory_time_resolution;
    }
  }
}

}  // namespace planning
}  // namespace apollo
