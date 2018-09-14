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

/*
 * hybrid_a_star.cc
 */

#include "modules/planning/planner/open_space/hybrid_a_star.h"

namespace apollo {
namespace planning {

HybridAStar::HybridAStar(double sx, double sy, double sphi, double ex,
                         double ey, double ephi,
                         std::vector<const Obstacle*> obstacles,
                         double xy_grid_resolution,
                         double phi_grid_resolution) {
  start_node_.reset(new Node3d(sx, sy, sphi, sx / xy_grid_resolution,
                             sy / xy_grid_resolution,
                             sphi / phi_grid_resolution));
  end_node_.reset(new Node3d(ex, ey, ephi, ex / xy_grid_resolution,
                           ey / xy_grid_resolution,
                           ephi / phi_grid_resolution));
  obstacles_ = obstacles;
  xy_grid_resolution_ = xy_grid_resolution;
  phi_grid_resolution_ = phi_grid_resolution;
}

Status HybridAStar::Plan() {}

}  // namespace planning
}  // namespace apollo