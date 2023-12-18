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

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/proto/dp_poly_path_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/path_decision.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"
#include "modules/planning/planning_base/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/planning_base/reference_line/reference_point.h"
#include "modules/planning/planning_interface_base/task_base/optimizers/road_graph/trajectory_cost.h"

namespace apollo {
namespace planning {

class WaypointSampler {
 public:
  explicit WaypointSampler(const WaypointSamplerConfig &config)
      : config_(config) {}
  virtual ~WaypointSampler() = default;

  virtual void Init(const ReferenceLineInfo *reference_line_info,
                    const common::SLPoint &init_sl_point_,
                    const common::FrenetFramePoint &init_frenet_frame_point);

  virtual void SetDebugLogger(apollo::planning_internal::Debug *debug) {
    planning_debug_ = debug;
  }

  virtual bool SamplePathWaypoints(
      const common::TrajectoryPoint &init_point,
      std::vector<std::vector<common::SLPoint>> *const points);

 protected:
  const WaypointSamplerConfig &config_;
  const ReferenceLineInfo *reference_line_info_ = nullptr;
  common::SLPoint init_sl_point_;
  common::FrenetFramePoint init_frenet_frame_point_;
  apollo::planning_internal::Debug *planning_debug_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
