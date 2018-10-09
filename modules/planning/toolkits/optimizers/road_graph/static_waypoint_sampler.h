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

#include <limits>
#include <list>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/dp_poly_path_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/toolkits/optimizers/road_graph/trajectory_cost.h"
#include "modules/planning/toolkits/optimizers/road_graph/waypoint_sampler.h"

namespace apollo {
namespace planning {

/**
 * This class only samples waypoints at fixed positions on the route. This
 * method can increase the path stability.
 */
class StaticWaypointSampler : public WaypointSampler {
 public:
  explicit StaticWaypointSampler(const WaypointSamplerConfig &config)
      : WaypointSampler(config) {}

  virtual bool SamplePathWaypoints(
      const common::TrajectoryPoint &init_point,
      std::vector<std::vector<common::SLPoint>> *const points);
};

}  // namespace planning
}  // namespace apollo
