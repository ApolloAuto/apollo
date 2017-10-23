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
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"

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

bool RouteSegments::GetInnerProjection(const common::PointENU &point_enu,
                                       double *s, double *l) const {
  const auto point = common::math::Vec2d{point_enu.x(), point_enu.y()};
  *l = std::numeric_limits<double>::infinity();
  double accumulate_s = 0.0;
  bool has_projection = false;
  for (auto iter = begin(); iter != end();
       accumulate_s += (iter->end_s - iter->start_s), ++iter) {
    double lane_s = 0.0;
    double lane_l = 0.0;
    if (!iter->lane->GetProjection(point, &lane_s, &lane_l)) {
      return false;
    }
    if (lane_s < iter->start_s - kSegmentationEpsilon ||
        lane_s > iter->end_s + kSegmentationEpsilon) {
      continue;
    }
    has_projection = true;
    if (std::fabs(lane_l) < *l) {
      *l = std::fabs(lane_l);
      *s = lane_s - iter->start_s + accumulate_s;
    }
  }
  return has_projection;
}

PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}

const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  if (routing_.has_header() && routing.has_header() &&
      routing_.header().sequence_num() == routing.header().sequence_num() &&
      (std::fabs(routing_.header().timestamp_sec() ==
                 routing.header().timestamp_sec()) < 0.1)) {
    AINFO << "Same prouting, skip update routing";
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
  routing_ = routing;
  last_waypoint_.reset(nullptr);
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
  return true;
}

std::vector<std::pair<int, routing::ChangeLaneType>> PncMap::GetDrivePassages(
    const routing::RoadSegment &road, int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<std::pair<int, routing::ChangeLaneType>> result;
  result.emplace_back(start_passage, routing::FORWARD);
  const auto &source_passage = road.passage(start_passage);
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  if (source_passage.can_exit()) {  // no need to change lane
    return result;
  }
  RouteSegments source_segments;
  DCHECK(PassageToSegments(source_passage, &source_segments))
      << "failed to convert passage to segments";
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
    std::unordered_set<std::string> target_lane_set;
    target_lane_set.reserve(target_passage.segment_size());
    for (const auto &segment : target_passage.segment()) {
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i, source_passage.change_lane_type());
        break;
      }
    }
  }
  return result;
}

bool PncMap::GetRouteSegments(
    const common::PointENU &point, const double backward_length,
    const double forward_length,
    std::vector<RouteSegments> *const route_segments) const {
  LaneWaypoint waypoint;
  if (!GetNearestPointFromRouting(point, &waypoint)) {
    AERROR << "Failed to get waypoint from routing";
    return false;
  }
  auto index = GetWaypointIndex(waypoint);
  if (index.size() != 3 || index[0] < 0) {
    AERROR << "Failed to get routing index from waypoint";
    return false;
  }
  int road_index = index[0];
  int passage_index = index[1];
  const auto &road = routing_.road(road_index);
  auto drive_passages = GetDrivePassages(road, passage_index);
  for (const auto &drive_passage : drive_passages) {
    auto index = drive_passage.first;
    auto change_lane_type = drive_passage.second;
    const auto &passage = road.passage(drive_passage.first);
    RouteSegments segments;
    DCHECK(PassageToSegments(passage, &segments))
        << "Failed to convert passage to lane segments.";
    double s = 0.0;
    double l = 0.0;
    bool has_projection = segments.GetInnerProjection(point, &s, &l);
    if (!has_projection) {
      ADEBUG << "Passage(" << index << ") in road(" << road_index
             << ") is not in range for lane change";
      continue;
    }
    route_segments->emplace_back();
    TruncateLaneSegments(segments, s - backward_length, s + forward_length,
                         &route_segments->back());
    route_segments->back().SetChangeLaneType(change_lane_type);
  }
  return true;
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
