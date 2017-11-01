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
 * @file: pnc_map.cc
 **/

#include "modules/map/pnc_map/pnc_map.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "google/protobuf/text_format.h"

#include "modules/map/proto/map_id.pb.h"

#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/routing/common/routing_gflags.h"


DEFINE_bool(reckless_change_lane, false, "always allow change lane");

namespace apollo {
namespace hdmap {

using apollo::routing::RoutingResponse;

namespace {

// Minimum error in lane segmentation.
const double kSegmentationEpsilon = 0.2;

// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

void RemoveDuplicates(std::vector<common::math::Vec2d> *points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    }
  }
  points->resize(count);
}

bool WithinLaneSegment(const LaneSegment &lane_segment,
                       const LaneWaypoint &waypoint) {
  return waypoint.lane &&
         lane_segment.lane->id().id() == waypoint.lane->id().id() &&
         lane_segment.start_s - kSegmentationEpsilon <= waypoint.s &&
         lane_segment.end_s + kSegmentationEpsilon >= waypoint.s;
}

bool WithinLaneSegment(const routing::LaneSegment &lane_segment,
                       const LaneWaypoint &waypoint) {
  return waypoint.lane && lane_segment.id() == waypoint.lane->id().id() &&
         lane_segment.start_s() - kSegmentationEpsilon <= waypoint.s &&
         lane_segment.end_s() + kSegmentationEpsilon >= waypoint.s;
}

void RemoveDuplicates(std::vector<MapPathPoint> *points) {
  CHECK_NOTNULL(points);
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

}  // namespace

const std::string &RouteSegments::Id() const { return id_; }

void RouteSegments::SetId(const std::string &id) { id_ = id; }

void RouteSegments::SetCanExit(bool can_exit) { can_exit_ = can_exit; }

bool RouteSegments::CanExit() const { return can_exit_; }

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

PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}

const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

bool PncMap::UpdatePosition(const common::PointENU &point) {
  if (!GetNearestPointFromRouting(point, &current_waypoint_)) {
    AERROR << "Failed to get waypoint from routing";
    return false;
  }
  auto current_route_index = GetWaypointIndex(current_waypoint_);
  if (current_route_index.size() != 3 || current_route_index[0] < 0) {
    AERROR << "Failed to get routing index from waypoint";
    return false;
  }

  // only update passage_start_point_ when route passage changes
  if (route_index_.size() != 3 || route_index_[0] != current_route_index[0] ||
      route_index_[1] != current_route_index[1]) {  //  different passage
    passage_start_point_ = point;
    min_l_to_lane_center_ = std::numeric_limits<double>::max();
  }
  current_point_ = point;
  route_index_ = current_route_index;
  double s = 0.0;
  double l = 0.0;
  if (!current_waypoint_.lane->GetProjection({point.x(), point.y()}, &s, &l)) {
    AERROR << "Failed to project poin " << point.ShortDebugString()
           << " onto lane " << current_waypoint_.lane->id().id();
    return false;
  }
  min_l_to_lane_center_ = std::min(min_l_to_lane_center_, std::fabs(l));
  return true;
}

bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  if (routing_.has_header() && routing.has_header() &&
      routing_.header().sequence_num() == routing.header().sequence_num() &&
      (std::fabs(routing_.header().timestamp_sec() -
                 routing.header().timestamp_sec()) < 0.1)) {
    ADEBUG << "Same routing, skip update routing";
    return false;
  }
  if (!ValidateRouting(routing)) {
    AERROR << "Invalid routing";
    return false;
  }
  routing_lane_ids_.clear();
  for (const auto &road : routing.road()) {
    for (const auto &passage : road.passage()) {
      for (const auto &lane : passage.segment()) {
        routing_lane_ids_.insert(lane.id());
      }
    }
  }
  current_waypoint_.lane = nullptr;
  route_index_.clear();
  current_point_.Clear();
  passage_start_point_.Clear();
  routing_ = routing;
  return true;
}

const routing::RoutingResponse &PncMap::routing_response() const {
  return routing_;
}

bool PncMap::ValidateRouting(const RoutingResponse &routing) {
  const int num_road = routing.road_size();
  if (num_road == 0) {
    AERROR << "Route is empty.";
    return false;
  }
  return true;
}

std::vector<int> PncMap::GetWaypointIndex(const LaneWaypoint &waypoint) const {
  for (int road_index = routing_.road_size() - 1; road_index >= 0;
       --road_index) {
    const auto &road_segment = routing_.road(road_index);
    for (int passage_index = road_segment.passage_size() - 1;
         passage_index >= 0; --passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = passage.segment_size() - 1; lane_index >= 0;
           --lane_index) {
        if (WithinLaneSegment(passage.segment(lane_index), waypoint)) {
          return {road_index, passage_index, lane_index};
        }
      }
    }
  }
  return {-1, -1, -1};
}

bool PncMap::PassageToSegments(routing::Passage passage,
                               RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane : " << lane.id();
      return false;
    }
    segments->emplace_back(lane_ptr, lane.start_s(), lane.end_s());
  }
  return !segments->empty();
}

bool RouteSegments::IsOnSegment() const { return is_on_segment_; }

void RouteSegments::SetIsOnSegment(bool on_segment) {
  is_on_segment_ = on_segment;
}

std::vector<int> PncMap::GetNeighborPassages(const routing::RoadSegment &road,
                                             int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<int> result;
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  if (source_passage.can_exit()) {  // no need to change lane
    return result;
  }
  RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "failed to convert passage to segments";
    return result;
  }
  std::unordered_set<std::string> neighbor_lanes;
  if (source_passage.change_lane_type() == routing::LEFT) {
    for (const auto &segment : source_segments) {
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(left_id.id());
      }
    }
  } else if (source_passage.change_lane_type() == routing::RIGHT) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }

  for (int i = 0; i < road.passage_size(); ++i) {
    if (i == start_passage) {
      continue;
    }
    const auto &target_passage = road.passage(i);
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }
  return result;
}

bool PncMap::GetRouteSegments(
    const double backward_length, const double forward_length,
    std::vector<RouteSegments> *const route_segments) const {
  // vehicle has to be this close to lane center before considering change lane
  constexpr double kMaxDistanceToLaneCenter = 0.5;
  if (!current_waypoint_.lane || route_index_.size() != 3 ||
      route_index_[0] < 0) {
    AERROR << "Invalid position, use UpdatePosition() function first";
    return false;
  }
  const int road_index = route_index_[0];
  const int passage_index = route_index_[1];
  const auto &road = routing_.road(road_index);
  const double dist_on_passage =
      common::util::DistanceXY(current_point_, passage_start_point_);
  const bool allow_change_lane =
      FLAGS_reckless_change_lane || (
      (min_l_to_lane_center_ < kMaxDistanceToLaneCenter) &&
      (dist_on_passage > FLAGS_min_length_for_lane_change));
  // raw filter to find all neighboring passages
  auto drive_passages = GetNeighborPassages(road, passage_index);
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    RouteSegments segments;
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    auto nearest_point = current_point_;
    if (index == passage_index) {
      nearest_point =
          current_waypoint_.lane->GetSmoothPoint(current_waypoint_.s);
    }
    double s = 0.0;
    double l = 0.0;
    LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &s, &l, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    if (index != passage_index) {
      if (!allow_change_lane) {
        continue;
      }
      if (!segments.CanDriveFrom(current_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }
    route_segments->emplace_back();
    TruncateLaneSegments(segments, s - backward_length, s + forward_length,
                         &route_segments->back());
    const auto last_waypoint = segments.LastWaypoint();
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    route_segments->back().SetCanExit(passage.can_exit());
    route_segments->back().SetNextAction(passage.change_lane_type());
    std::string route_segment_id =
        std::to_string(road_index) + "_" + std::to_string(index);
    route_segments->back().SetId(route_segment_id);
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
    }
  }
  return !route_segments->empty();
}

bool PncMap::GetNearestPointFromRouting(const common::PointENU &point,
                                        LaneWaypoint *waypoint) const {
  const double kMaxDistance = 20.0;  // meters.
  waypoint->lane = nullptr;
  std::vector<LaneInfoConstPtr> lanes;
  const int status = hdmap_->GetLanes(point, kMaxDistance, &lanes);
  if (status < 0) {
    AERROR << "failed to get lane from point " << point.DebugString();
    return false;
  }
  if (lanes.empty()) {
    AERROR << "No valid lane found within " << kMaxDistance << " meters.";
    return false;
  }
  // get nearest_wayponints for current position
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &lane : lanes) {
    if (routing_lane_ids_.count(lane->id().id()) == 0) {
      continue;
    }
    double distance = 0.0;
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    if (distance < min_distance) {
      min_distance = distance;
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "Failed to get projection for map_point "
               << map_point.DebugString();
        return false;
      }
      waypoint->lane = lane;
      waypoint->s = s;
    }
  }
  if (waypoint->lane == nullptr) {
    AERROR << "failed to find nearest point";
  }
  return waypoint->lane != nullptr;
}

LaneInfoConstPtr PncMap::GetRouteSuccessor(LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id_size() == 0) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (routing_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

LaneInfoConstPtr PncMap::GetRoutePredecessor(LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id_size() == 0) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    if (routing_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

bool PncMap::TruncateLaneSegments(
    const RouteSegments &segments, double start_s, double end_s,
    RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  const double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    double extend_s = -start_s;
    std::vector<LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        lane = GetRoutePredecessor(lane);
        if (lane == nullptr) {
          break;
        }
        s = lane->total_length();
      } else {
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length;
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                       adjusted_end_s);
    }
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s) {
    const auto &last_segment = segments.back();
    auto last_lane = last_segment.lane;
    double last_s = last_segment.end_s;
    while (router_s < end_s - kRouteEpsilon) {
      if (last_lane == nullptr) {
        break;
      }
      if (last_s >= last_lane->total_length() - kRouteEpsilon) {
        last_lane = GetRouteSuccessor(last_lane);
        last_s = 0.0;
      } else {
        const double length =
            std::min(end_s - router_s, last_lane->total_length() - last_s);
        truncated_segments->emplace_back(last_lane, last_s, last_s + length);
        router_s += length;
        last_s += length;
      }
    }
  }
  return true;
}

void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() +
                segment.unit_direction() * (start_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

bool PncMap::CreatePathFromLaneSegments(const RouteSegments &segments,
                                        Path *const path) {
  std::vector<MapPathPoint> points;
  for (const auto &segment : segments) {
    AppendLaneToPoints(segment.lane, segment.start_s, segment.end_s, &points);
  }
  RemoveDuplicates(&points);

  if (points.size() < 2) {
    AWARN << "Cannot create path from " << points.size()
          << " points. Expecting more than 2.";
    return false;
  }

  *path = Path(points, segments, kTrajectoryApproximationMaxError);
  return true;
}

}  // namespace hdmap
}  // namespace apollo
