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

/**
 * @file:
 **/

#include "modules/map/pnc_map/route_segments.h"

#include <algorithm>

#include "modules/common/util/util.h"

namespace apollo {
namespace hdmap {
namespace {

// Minimum error in lane segmentation.
const double kSegmentationEpsilon = 0.2;
}

const std::string &RouteSegments::Id() const { return id_; }

void RouteSegments::SetId(const std::string &id) { id_ = id; }

void RouteSegments::SetCanExit(bool can_exit) { can_exit_ = can_exit; }

bool RouteSegments::CanExit() const { return can_exit_; }

bool RouteSegments::WithinLaneSegment(const LaneSegment &lane_segment,
                                      const LaneWaypoint &waypoint) {
  return waypoint.lane &&
         lane_segment.lane->id().id() == waypoint.lane->id().id() &&
         lane_segment.start_s - kSegmentationEpsilon <= waypoint.s &&
         lane_segment.end_s + kSegmentationEpsilon >= waypoint.s;
}

bool RouteSegments::WithinLaneSegment(const routing::LaneSegment &lane_segment,
                                      const LaneWaypoint &waypoint) {
  return waypoint.lane && lane_segment.id() == waypoint.lane->id().id() &&
         lane_segment.start_s() - kSegmentationEpsilon <= waypoint.s &&
         lane_segment.end_s() + kSegmentationEpsilon >= waypoint.s;
}

const LaneWaypoint &RouteSegments::RouteEndWaypoint() const {
  return route_end_waypoint_;
}

void RouteSegments::SetRouteEndWaypoint(const LaneWaypoint &waypoint) {
  route_end_waypoint_ = waypoint;
}

LaneWaypoint RouteSegments::LastWaypoint() const {
  if (size() > 0) {
    return LaneWaypoint(back().lane, back().end_s);
  }
  return LaneWaypoint();
}

bool RouteSegments::GetProjection(const common::PointENU &point_enu, double *s,
                                  double *l, LaneWaypoint *waypoint) const {
  const auto point = common::math::Vec2d{point_enu.x(), point_enu.y()};
  *l = std::numeric_limits<double>::infinity();
  double accumulate_s = 0.0;
  bool has_projection = false;
  for (auto iter = begin(); iter != end();
       accumulate_s += (iter->end_s - iter->start_s), ++iter) {
    double lane_s = 0.0;
    double lane_l = 0.0;
    if (!iter->lane->GetProjection(point, &lane_s, &lane_l)) {
      AERROR << "Failed to get projection from point " << point.DebugString()
             << " on lane " << iter->lane->id().id();
      return false;
    }
    if (lane_s < iter->start_s - kSegmentationEpsilon ||
        lane_s > iter->end_s + kSegmentationEpsilon) {
      continue;
    }
    if (std::fabs(lane_l) < std::fabs(*l)) {
      has_projection = true;
      lane_s = std::max(iter->start_s, lane_s);
      lane_s = std::min(iter->end_s, lane_s);
      *l = lane_l;
      *s = lane_s - iter->start_s + accumulate_s;
      waypoint->lane = iter->lane;
      waypoint->s = lane_s;
    }
  }
  return has_projection;
}

void RouteSegments::SetPreviousAction(routing::ChangeLaneType action) {
  previous_action_ = action;
}

routing::ChangeLaneType RouteSegments::PreviousAction() const {
  return previous_action_;
}

void RouteSegments::SetNextAction(routing::ChangeLaneType action) {
  next_action_ = action;
}

routing::ChangeLaneType RouteSegments::NextAction() const {
  return next_action_;
}

bool RouteSegments::IsWaypointOnSegment(const LaneWaypoint &waypoint) const {
  for (auto iter = begin(); iter != end(); ++iter) {
    if (WithinLaneSegment(*iter, waypoint)) {
      return true;
    }
  }
  return false;
}

bool RouteSegments::CanDriveFrom(const LaneWaypoint &waypoint) const {
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);

  // 0 if waypoint is on segment, ok
  if (IsWaypointOnSegment(waypoint)) {
    return true;
  }

  // 1. should have valid projection.
  LaneWaypoint segment_waypoint;
  double route_s = 0.0;
  double lane_l = 0.0;
  bool has_projection =
      GetProjection(point, &route_s, &lane_l, &segment_waypoint);
  if (!has_projection) {
    ADEBUG << "No projection from waypoint: " << waypoint.DebugString();
    return false;
  }
  // 2. heading should be the same.
  double waypoint_heading = waypoint.lane->Heading(waypoint.s);
  double segment_heading = segment_waypoint.lane->Heading(segment_waypoint.s);
  double heading_diff =
      common::math::AngleDiff(waypoint_heading, segment_heading);
  if (std::fabs(heading_diff) > M_PI / 2) {
    ADEBUG << "Angle diff too large";
    return false;
  }

  // 3. segment waypoint should be waypoint's neighbor
  // assume waypoint is at left side
  const auto *neighbor_ids =
      &(segment_waypoint.lane->lane().left_neighbor_forward_lane_id());
  if (lane_l < 0) {  // waypoint is at right side
    neighbor_ids =
        &(segment_waypoint.lane->lane().right_neighbor_forward_lane_id());
  }
  bool is_neighbor = false;
  for (const auto &id : *neighbor_ids) {
    if (id.id() == waypoint.lane->id().id()) {
      is_neighbor = true;
      break;
    }
  }
  if (!is_neighbor) {
    ADEBUG << "waypoint is not neighbor of current segment";
    return false;
  }

  // 4. the waypoint and the projected lane should not be separated apart.
  double waypoint_left_width = 0.0;
  double waypoint_right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &waypoint_left_width,
                          &waypoint_right_width);
  double segment_left_width = 0.0;
  double segment_right_width = 0.0;
  segment_waypoint.lane->GetWidth(segment_waypoint.s, &segment_left_width,
                                  &segment_right_width);
  auto segment_projected_point =
      segment_waypoint.lane->GetSmoothPoint(segment_waypoint.s);
  double dist = common::util::DistanceXY(point, segment_projected_point);
  const double kLaneSeparationDistance = 0.2;
  if (lane_l < 0) {  // waypoint at right side
    if (dist >
        waypoint_left_width + segment_right_width + kLaneSeparationDistance) {
      ADEBUG << "waypoint is too far to reach";
      return false;
    }
  } else {  // waypoint at left side
    if (dist >
        waypoint_right_width + segment_left_width + kLaneSeparationDistance) {
      ADEBUG << "waypoint is too far to reach";
      return false;
    }
  }

  return true;
}

}  // namespace hdmap
}  // namespace apollo
