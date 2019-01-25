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

#include "modules/planning/common/distance_estimator.h"

#include <algorithm>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace apollo {
namespace planning {

using common::math::Vec2d;
using common::TrajectoryPoint;
using hdmap::RouteSegments;
using hdmap::HDMapUtil;

DistanceEstimator::DistanceEstimator(const TrajectoryPoint& ego_point,
                                     const RouteSegments& route_segments)
    : ego_point_(ego_point), route_segments_(route_segments) {}

double DistanceEstimator::GetSDistanceToFirstOverlapLane(
    const DiscretizedPath& path) {
  return 0.0;
}

bool DistanceEstimator::UpdateCurrentLane() {
  hdmap::LaneInfoConstPtr lane;
  double s = 0.0;
  double l = 0.0;
  if (HDMapUtil::BaseMapPtr()->GetNearestLaneWithHeading(
          common::util::MakePointENU(ego_point_.path_point().x(),
                                     ego_point_.path_point().y(),
                                     ego_point_.path_point().z()),
          1.0, ego_point_.path_point().theta(), M_PI / 3.0, &lane, &s,
          &l) != 0) {
    AERROR << "Failed to find nearest lane from map at position: "
           << ego_point_.DebugString()
           << ", heading:" << ego_point_.path_point().theta();
    return false;
  }
  curr_lane_ = lane->lane();
  ADEBUG << curr_lane_.DebugString();

  return true;
}

double DistanceEstimator::GetOverlapPointS(const DiscretizedPath& path) {
  double dist = 0.0;
  if (curr_lane_.successor_id_size() == 0) {
    return dist;
  }
  const auto& next_lane_info =
      HDMapUtil::BaseMapPtr()->GetLaneById(curr_lane_.successor_id(0));

  for (const auto& path_point : path) {
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
    const std::vector<Vec2d>& corners = vehicle_box.GetAllCorners();

    bool is_overlap = false;
    for (const auto id : next_lane_info->lane().predecessor_id()) {
      if (is_overlap) {
        break;
      }
      if (id.id() == curr_lane_.id().id()) {
        continue;
      }
      const auto& lane_info = HDMapUtil::BaseMapPtr()->GetLaneById(id);
      for (const auto& corner : corners) {
        if (lane_info->IsOnLane(corner)) {
          is_overlap = true;
          break;
        }
      }
    }
    if (!is_overlap) {
      dist = std::max(dist, path_point.s());
    } else {
      break;
    }
  }
  return dist;
}

}  // namespace planning
}  // namespace apollo
