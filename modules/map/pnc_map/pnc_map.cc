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

#include <fstream>
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

namespace {

// Minimum error in lane segmentation.
const double kSegmentationEpsilon = 0.2;

// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

void remove_duplicates(std::vector<common::math::Vec2d> *points) {
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

void remove_duplicates(std::vector<hdmap::MapPathPoint> *points) {
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
}

PncMap::PncMap(const std::string &map_file) {
  CHECK(!hdmap_.load_map_from_file(map_file)) << "Failed to load map file:"
                                              << map_file;
  AINFO << "map loaded, Map file: " << map_file;
}

bool PncMap::validate_routing(const ::apollo::routing::RoutingResponse &routing) const {
  const int num_routes = routing.route_size();
  if (num_routes == 0) {
    AERROR << "Route is empty.";
    return false;
  }
/*
  const double kLargeEps = 1e-3;
  for (int i = 0; i < num_routes; ++i) {
    const auto &segment = routing.route(i);
    const auto lane = hdmap_.get_lane_by_id(hdmap::MakeMapId(segment.id()));
    if (lane == nullptr) {
      AERROR << "Can not find lane with id = " + segment.id() + ".";
      return false;
    }
    if (segment.start_s() > segment.end_s() + kLargeEps) {
      AERROR << "Segment " << i << " is empty.";
      return false;
    }
    bool start_point_in_lane_change = false;
    bool end_point_in_lane_change = false;
    for (const auto &info : routing.lane_change_info()) {
      if (i > info.start_route_index() && i <= info.end_route_index()) {
        start_point_in_lane_change = true;
      }
      if (i >= info.start_route_index() && i < info.end_route_index()) {
        end_point_in_lane_change = true;
      }
    }
    if (!start_point_in_lane_change &&
        ((i > 0 && std::abs(segment.start_s()) > kLargeEps) ||
         (i == 0 && segment.start_s() < -kLargeEps))) {
      AERROR << "Segment " << i << " (lane " << segment.id()
             << ") should start at s = 0.";
      return false;
    }
    if (!end_point_in_lane_change &&
        ((i < num_routes - 1 &&
          std::abs(segment.end_s() - lane->total_length()) > kLargeEps) ||
         (i == num_routes - 1 &&
          segment.end_s() > lane->total_length() + kLargeEps))) {
      AERROR << "Segment " << i << " (lane " << segment.id()
             << ") should end at s = " << lane->total_length() << ".";
      return false;
    }
  }
  for (int i = 0; i < num_routes - 1; ++i) {
    bool is_lane_change = false;
    for (const auto &info : routing.lane_change_info()) {
      if (i >= info.start_route_index() && i < info.end_route_index()) {
        is_lane_change = true;
        break;
      }
    }
    if (is_lane_change) {
      // TODO: check lane change information.
      continue;
    }
    const auto &segment = routing.route(i);
    const auto lane = hdmap_.get_lane_by_id(hdmap::MakeMapId(segment.id()));
    const std::string next_id = routing.route(i + 1).id();
    bool is_successor = false;
    for (const auto &other_lane_id : lane->lane().successor_id()) {
      if (other_lane_id.id() == next_id) {
        is_successor = true;
        break;
      }
    }
    if (!is_successor) {
      AERROR << "Lane " + segment.id() + " can not connect to Lane " + next_id +
                    ".";
      return false;
    }
  }
*/
  return true;
}

bool PncMap::CreatePathFromRouting(const ::apollo::routing::RoutingResponse &routing,
                                   const common::PointENU &point,
                                   const double backward_length,
                                   const double forward_length,
                                   hdmap::Path *path) const {
  if (path == nullptr) {
    AERROR << "the provided Path is null";
    return false;
  }
  if (backward_length < 0.0 || forward_length < 0.0 ||
      backward_length + forward_length <= 0.0) {
    AERROR << "backward_length[" << backward_length << "] or forward_length["
           << forward_length << "] are invalid";
    return false;
  }
  if (!validate_routing(routing)) {
    AERROR << "The provided routing result is invalid";
    return false;
  }
  const double kMaxDistance = 20.0;  // meters.
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  const int status = hdmap_.get_lanes(point, kMaxDistance, &lanes);
  if (status < 0) {
    AERROR << "failed to get lane from point " << point.DebugString();
    return false;
  }
  std::unordered_set<std::string> routing_lane_ids;
  for (int i = 0; i < routing.route(0).road_info().passage_region(0).segment_size(); ++i ){
    const auto& lane_segment = routing.route(0).road_info().passage_region(0).segment(i);
    routing_lane_ids.insert(lane_segment.id());
  }
  double min_distance = std::numeric_limits<double>::infinity();
  hdmap::LaneWaypoint nearest_waypoint;
  for (const auto lane : lanes) {
    if (!routing_lane_ids.count(lane->id().id())) {
      continue;
    }
    double distance = 0.0;
    common::PointENU map_point =
        lane->get_nearest_point({point.x(), point.y()}, &distance);
    if (distance < min_distance) {
      min_distance = distance;
      double s = 0.0;
      double l = 0.0;
      if (!lane->get_projection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "Failed to get projection for map_point "
               << map_point.DebugString();
        return false;
      }
      nearest_waypoint = hdmap::LaneWaypoint(lane, s);
    }
  }
  if (nearest_waypoint.lane == nullptr) {
    AERROR << "Failed to find point on routing. Point:" << point.DebugString();
    return false;
  }
  double min_overlap_distance = std::numeric_limits<double>::infinity();
  double proj_s = 0.0;
  double accumulate_s = 0.0;
  for (int i = 0; i < routing.route(0).road_info().passage_region(0).segment_size(); ++i){
    const auto& lane_segment = routing.route(0).road_info().passage_region(0).segment(i);
    const double length = lane_segment.end_s() - lane_segment.start_s();
    if (lane_segment.id() == nearest_waypoint.lane->id().id()) {
      double overlap_distance = 0.0;
      if (nearest_waypoint.s < lane_segment.start_s()) {
        overlap_distance = lane_segment.start_s() - nearest_waypoint.s;
      } else if (nearest_waypoint.s > lane_segment.end_s()) {
        overlap_distance = nearest_waypoint.s - lane_segment.end_s();
      }
      if (overlap_distance < min_overlap_distance) {
        min_overlap_distance = overlap_distance;
        proj_s = accumulate_s +
                 std::max(0.0, std::min(length, nearest_waypoint.s -
                                                    lane_segment.start_s()));
      }
    }
    accumulate_s += length;
  }
  if (min_overlap_distance < std::numeric_limits<double>::infinity()) {
    return CreatePathFromRouting(routing, proj_s - backward_length,
                                 proj_s + forward_length, path);
  } else {
    AERROR << "cannot find map lane from routing segments.";
    return false;
  }
}

bool PncMap::CreatePathFromRouting(const ::apollo::routing::RoutingResponse &routing,
                                   hdmap::Path *path) const {
  if (!validate_routing(routing)) {
    AERROR << "routing is invalid";
    return false;
  }
  double length = 0.0;
  for (int i = 0; i < routing.route(0).road_info().passage_region(0).segment_size(); ++i){
    const auto& lane_segment = routing.route(0).road_info().passage_region(0).segment(i);
    CHECK_LE(lane_segment.start_s(),
             lane_segment.end_s() + common::math::kMathEpsilon);
    length += lane_segment.end_s() - lane_segment.start_s();
  }
  double expected_length = routing.measurement().distance();
  if (std::abs(length - expected_length) > 1e-2) {
    // TODO: report error if the length is mis-matched.
    LOG(WARNING) << "Actual length of routing result is " << length
                 << " which is different from " << expected_length;
  }
  return CreatePathFromRouting(routing, 0.0, length, path);
}

bool PncMap::CreatePathFromRouting(const ::apollo::routing::RoutingResponse &routing,
                                   double start_s, double end_s,
                                   hdmap::Path *path) const {
  if (path == nullptr) {
    AERROR << "input path is null";
    return false;
  }
  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  if (!validate_routing(routing)) {
    AERROR << " routing is invalid";
    return false;
  }
  const double kRouteEpsilon = 1e-3;
  std::vector<hdmap::MapPathPoint> points;
  std::vector<hdmap::LaneSegment> lane_segments;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) {
    const auto &first_segment = routing.route(0).road_info().passage_region(0).segment(0);
    auto lane = hdmap_.get_lane_by_id(hdmap::MakeMapId(first_segment.id()));
    if (!lane) {
      AERROR << "failed to get lane from id " << first_segment.id();
      return false;
    }
    double s = first_segment.start_s();
    double extend_s = -start_s;
    std::vector<hdmap::LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        if (lane->lane().predecessor_id_size() == 0) {
          break;
        }
        const auto &next_lane_id = lane->lane().predecessor_id(0);
        lane = hdmap_.get_lane_by_id(next_lane_id);
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
    std::reverse(extended_lane_segments.begin(), extended_lane_segments.end());
    for (const hdmap::LaneSegment &segment : extended_lane_segments) {
      append_lane_to_points(segment.lane, segment.start_s, segment.end_s,
                            &points, &lane_segments);
    }
  }
  double router_s = 0;
  for (int i = 0; i < routing.route(0).road_info().passage_region(0).segment_size(); ++i ){
    const auto& lane_segment = routing.route(0).road_info().passage_region(0).segment(i);
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s(), lane_segment.start_s());
    const double adjusted_end_s = std::min(
        end_s - router_s + lane_segment.start_s(), lane_segment.end_s());
    if (adjusted_start_s < adjusted_end_s) {
      const auto lane_info =
          hdmap_.get_lane_by_id(hdmap::MakeMapId(lane_segment.id()));
      if (lane_info == nullptr) {
        AERROR << "Failed to find lane " << lane_segment.id() << " in map";
        return false;
      }
      append_lane_to_points(lane_info, adjusted_start_s, adjusted_end_s,
                            &points, &lane_segments);
    }
    router_s += (lane_segment.end_s() - lane_segment.start_s());
    if (router_s > end_s) {
      break;
    }
  }
  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s && routing.route_size() > 0) {
    const auto &last_segment = routing.route(routing.route_size() - 1).road_info().passage_region(0).segment(routing.route(routing.route_size() - 1).road_info().passage_region(0).segment_size() - 1);
    std::string last_lane_id = last_segment.id();
    double last_s = last_segment.end_s();
    while (router_s < end_s - kRouteEpsilon) {
      const auto lane = hdmap_.get_lane_by_id(hdmap::MakeMapId(last_lane_id));
      if (lane == nullptr) {
        break;
      }
      if (last_s >= lane->total_length() - kRouteEpsilon) {
        if (lane->lane().successor_id_size() == 0) {
          break;
        }
        last_lane_id = lane->lane().successor_id(0).id();
        last_s = 0.0;
      } else {
        const double length =
            std::min(end_s - router_s, lane->total_length() - last_s);
        append_lane_to_points(lane, last_s, last_s + length, &points,
                              &lane_segments);
        router_s += length;
        last_s += length;
      }
    }
  }
  remove_duplicates(&points);
  *path = hdmap::Path(points, lane_segments, kTrajectoryApproximationMaxError);
  return true;
}

void PncMap::append_lane_to_points(
    hdmap::LaneInfoConstPtr lane, const double start_s, const double end_s,
    std::vector<hdmap::MapPathPoint> *const points,
    std::vector<hdmap::LaneSegment> *const lane_segments) const {
  if (points == nullptr || lane_segments == nullptr || start_s >= end_s) {
    return;
  }
  lane_segments->emplace_back(lane, start_s, end_s);
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           hdmap::LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() +
                segment.unit_direction() * (start_s - accumulate_s),
            lane->headings()[i], hdmap::LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], hdmap::LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

const hdmap::HDMap *PncMap::HDMap() const { return &hdmap_; }

}  // namespace hdmap
}  // namespace apollo
