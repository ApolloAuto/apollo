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

#include "cyber/common/macros.h"

#include "modules/map/proto/map_lane.pb.h"

#include "modules/map/pnc_map/route_segments.h"
#include "modules/planning/common/path/discretized_path.h"

namespace apollo {
namespace planning {

class DistanceEstimator {
 public:
  DistanceEstimator() = default;
  DistanceEstimator(const common::TrajectoryPoint& ego_point,
                    const hdmap::RouteSegments& route_segments);

  virtual ~DistanceEstimator() = default;

  double GetSDistanceToFirstOverlapLane(const DiscretizedPath& path);

 private:
  bool UpdateCurrentLane();

  double GetOverlapPointS(const DiscretizedPath& path);

 private:
  common::TrajectoryPoint ego_point_;
  hdmap::RouteSegments route_segments_;
  hdmap::Lane curr_lane_;
};

}  // namespace planning
}  // namespace apollo
