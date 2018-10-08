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
 * @file side_pass_waypoint_sampler.h
 **/

#pragma once


#include <vector>

#include "modules/planning/toolkits/optimizers/road_graph/waypoint_sampler.h"

namespace apollo {
namespace planning {

class SidePassWaypointSampler final : public WaypointSampler {
 public:
  explicit SidePassWaypointSampler(const WaypointSamplerConfig &config)
      : WaypointSampler(config) {}
  virtual ~SidePassWaypointSampler() = default;

  bool SamplePathWaypoints(
      const common::TrajectoryPoint &init_point,
      std::vector<std::vector<common::SLPoint>> *const points) override;

  bool HasSidepass();
};

}  // namespace planning
}  // namespace apollo


