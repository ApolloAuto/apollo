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
 * @file lane_follow_map.cc
 **/

#include "modules/planning/pnc_map/lane_follow_map/lane_follow_map.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

#include "modules/common_msgs/map_msgs/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace

LaneFollowMap::LaneFollowMap() : hdmap_(hdmap::HDMapUtil::BaseMapPtr()) {}

bool LaneFollowMap::CanProcess(const planning::PlanningCommand &command) const {
  return command.has_lane_follow_command();
}

hdmap::LaneWaypoint LaneFollowMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return hdmap::LaneWaypoint(lane, waypoint.s());
}

hdmap::LaneSegment LaneFollowMap::ToLaneSegment(
    const routing::LaneSegment &segment) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return hdmap::LaneSegment(lane, segment.start_s(), segment.end_s());
}

void LaneFollowMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  // Search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  // Search forwards
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> LaneFollowMap::FutureRouteWaypoints() const {
  const auto &waypoints =
      last_command_.lane_follow_command().routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

void LaneFollowMap::GetEndLaneWayPoint(
    std::shared_ptr<routing::LaneWaypoint> &end_point) const {
  if (!last_command_.has_lane_follow_command() ||
      !last_command_.lane_follow_command().has_routing_request()) {
    end_point = nullptr;
    return;
  }
  const auto &routing_request =
      last_command_.lane_follow_command().routing_request();
  if (routing_request.waypoint().size() < 1) {
    end_point = nullptr;
    return;
  }
  end_point = std::make_shared<routing::LaneWaypoint>();
  end_point->CopyFrom(*(routing_request.waypoint().rbegin()));
}

hdmap::LaneInfoConstPtr LaneFollowMap::GetLaneById(const hdmap::Id &id) const {
  if (nullptr == hdmap_) {
    return nullptr;
  }
  return hdmap_->GetLaneById(id);
}

bool LaneFollowMap::IsValid(const planning::PlanningCommand &command) const {
  if (!CanProcess(command)) {
    return false;
  }
  const auto &routing = command.lane_follow_command();
  const int num_road = routing.road_size();
  if (num_road == 0) {
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

void LaneFollowMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear();
  range_start_ = std::max(0, adc_index - 1);
  range_end_ = range_start_;
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      break;
    }
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}

bool LaneFollowMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  if (!IsValid(last_command_)) {
    AERROR << "The routing is invalid when updating vehicle state.";
    route_segments_lane_ids_.clear();
    return false;
  }
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    // Position is reset, but not replan.
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  adc_state_ = vehicle_state;
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    AERROR << "Failed to get waypoint from routing with point: " << "("
           << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    route_segments_lane_ids_.clear();
    return false;
  }
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }
  ADEBUG << "adc_waypoint_" << adc_waypoint_.DebugString() << "route_index"
         << route_index;
  // Track how many routing request waypoints the adc have passed.
  UpdateNextRoutingWaypointIndex(route_index);
  adc_route_index_ = route_index;
  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

bool LaneFollowMap::UpdatePlanningCommand(
    const planning::PlanningCommand &command) {
  if (!CanProcess(command)) {
    AERROR << "Command cannot be processed by LaneFollowMap!";
    return false;
  }
  if (!PncMapBase::UpdatePlanningCommand(command)) {
    return false;
  }
  const auto &routing = command.lane_follow_command();
  range_lane_ids_.clear();
  route_indices_.clear();
  all_lane_ids_.clear();
  route_segments_lane_ids_.clear();
  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment =
            ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};

        if (road_index == routing.road_size() - 1 &&
            lane_index == passage.segment_size() - 1 && passage.can_exit()) {
          dest_lane_segment_ = passage.segment(lane_index);
        }
      }
    }
  }

  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1;
  next_routing_waypoint_index_ = 0;
  UpdateRoutingRange(adc_route_index_);

  routing_waypoint_index_.clear();
  const auto &request_waypoints = routing.routing_request().waypoint();
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }
  int i = 0;
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    while (i < request_waypoints.size() &&
           hdmap::RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                                   request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          hdmap::LaneWaypoint(route_indices_[j].segment.lane,
                              request_waypoints.Get(i).s()),
          j);
      ++i;
    }
  }
  adc_waypoint_ = hdmap::LaneWaypoint();
  stop_for_destination_ = false;
  return true;
}

int LaneFollowMap::SearchForwardWaypointIndex(
    int start, const hdmap::LaneWaypoint &waypoint) const {
  int i = std::max(start, 0);
  while (i < static_cast<int>(route_indices_.size()) &&
         !hdmap::RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                  waypoint)) {
    ++i;
  }
  return i;
}

int LaneFollowMap::SearchBackwardWaypointIndex(
    int start, const hdmap::LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !hdmap::RouteSegments::WithinLaneSegment(
                       route_indices_[i].segment, waypoint)) {
    --i;
  }
  return i;
}

int LaneFollowMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int LaneFollowMap::GetWaypointIndex(const hdmap::LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

bool LaneFollowMap::PassageToSegments(routing::Passage passage,
                                      hdmap::RouteSegments *segments) const {
  CHECK_NOTNULL(segments);
  segments->clear();
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

std::vector<int> LaneFollowMap::GetNeighborPassages(
    const routing::RoadSegment &road, int start_passage) const {
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  std::vector<int> result;
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  if (source_passage.can_exit()) {  // No need to change lane
    return result;
  }
  hdmap::RouteSegments source_segments;
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
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
bool LaneFollowMap::GetRouteSegments(
    const VehicleState &vehicle_state,
    std::list<hdmap::RouteSegments> *const route_segments) {
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}

bool LaneFollowMap::GetRouteSegments(
    const VehicleState &vehicle_state, const double backward_length,
    const double forward_length,
    std::list<hdmap::RouteSegments> *const route_segments) {
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map.";
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }
  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0];
  const int passage_index = route_index[1];
  const auto &road = last_command_.lane_follow_command().road(road_index);
  // Raw filter to find all neighboring passages
  auto drive_passages = GetNeighborPassages(road, passage_index);
  for (const int index : drive_passages) {
    const auto &passage = road.passage(index);
    hdmap::RouteSegments segments;
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);
    common::SLPoint sl;
    hdmap::LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    route_segments->back().SetCanExit(passage.can_exit());
    route_segments->back().SetNextAction(passage.change_lane_type());
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }
  UpdateRouteSegmentsLaneIds(route_segments);
  return !route_segments->empty();
}

bool LaneFollowMap::GetNearestPointFromRouting(
    const common::VehicleState &state, hdmap::LaneWaypoint *waypoint) const {
  waypoint->lane = nullptr;
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  const auto point = PointFactory::ToPointENU(state);
  std::vector<hdmap::LaneInfoConstPtr> valid_lanes;
  for (auto lane_id : all_lane_ids_) {
    hdmap::Id id = hdmap::MakeMapId(lane_id);
    auto lane = hdmap_->GetLaneById(id);
    if (nullptr != lane) {
      valid_lanes.emplace_back(lane);
    }
  }

  // Get nearest_waypoints for current position
  std::vector<hdmap::LaneWaypoint> valid_way_points;
  for (const auto &lane : valid_lanes) {
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      ADEBUG << "not in range" << lane->id().id();
      continue;
    }
    if (route_segments_lane_ids_.size() > 0 &&
        route_segments_lane_ids_.count(lane->id().id()) == 0) {
      ADEBUG << "not in last frame route_segments: " << lane->id().id();
      continue;
    }
    double s = 0.0;
    double l = 0.0;
    {
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        continue;
      }
      ADEBUG << lane->id().id() << "," << s << "," << l;
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
      double lane_heading = lane->Heading(s);
      if (std::fabs(common::math::AngleDiff(lane_heading, state.heading())) >
          M_PI_2 * 1.5) {
        continue;
      }
    }

    valid_way_points.emplace_back();
    auto &last = valid_way_points.back();
    last.lane = lane;
    last.s = s;
    last.l = l;
    ADEBUG << "distance:" << std::fabs(l);
  }
  if (valid_way_points.empty()) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
    return false;
  }

  // find closest lane that satisfy vehicle heading
  int closest_index = -1;
  double distance = std::numeric_limits<double>::max();
  double lane_heading = 0.0;
  double vehicle_heading = state.heading();
  for (size_t i = 0; i < valid_way_points.size(); i++) {
    lane_heading = valid_way_points[i].lane->Heading(valid_way_points[i].s);
    if (std::abs(common::math::AngleDiff(lane_heading, vehicle_heading)) >
        M_PI_2 * 1.5) {
      continue;
    }
    if (std::fabs(valid_way_points[i].l) < distance) {
      distance = std::fabs(valid_way_points[i].l);
      closest_index = i;
    }
  }
  if (closest_index == -1) {
    AERROR << "Can not find nearest waypoint. vehicle heading:"
           << vehicle_heading << "lane heading:" << lane_heading;
    return false;
  }
  waypoint->lane = valid_way_points[closest_index].lane;
  waypoint->s = valid_way_points[closest_index].s;
  waypoint->l = valid_way_points[closest_index].l;
  return true;
}

hdmap::LaneInfoConstPtr LaneFollowMap::GetRouteSuccessor(
    hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

hdmap::LaneInfoConstPtr LaneFollowMap::GetRoutePredecessor(
    hdmap::LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (const auto &route_index : route_indices_) {
    auto &lane = route_index.segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

bool LaneFollowMap::ExtendSegments(const hdmap::RouteSegments &segments,
                                   const common::PointENU &point,
                                   double look_backward, double look_forward,
                                   hdmap::RouteSegments *extended_segments) {
  common::SLPoint sl;
  hdmap::LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

bool LaneFollowMap::ExtendSegments(
    const hdmap::RouteSegments &segments, double start_s, double end_s,
    hdmap::RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    double extend_s = -start_s;
    std::vector<hdmap::LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        lane = GetRoutePredecessor(lane);
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length;
        unique_lanes.insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
  bool found_loop = false;
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() ==
              lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        found_loop = true;
        break;
      }
    }
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
  if (found_loop) {
    return true;
  }

  // adjust end_s to match reference_line_endpoint_extend_length
  double last_lane_segment_length =
      segments.back().end_s - segments.back().start_s;
  auto last_lane = segments.back().lane;
  bool last_lane_can_exit =
      last_lane->id().id() == dest_lane_segment_.id() &&
      (adc_waypoint_.lane->id().id() != dest_lane_segment_.id() ||
       (adc_waypoint_.lane->id().id() == dest_lane_segment_.id() &&
        adc_waypoint_.s < dest_lane_segment_.end_s() + 1.0));
  if (last_lane_can_exit) {
    end_s =
        std::max(router_s - kRouteEpsilon,
                 router_s - last_lane_segment_length +
                     dest_lane_segment_.end_s() - dest_lane_segment_.start_s() +
                     FLAGS_reference_line_endpoint_extend_length);
  }

  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }

  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }

    bool last_lane_can_exit =
        last_lane->id().id() == dest_lane_segment_.id() &&
        (adc_waypoint_.lane->id().id() != dest_lane_segment_.id() ||
         (adc_waypoint_.lane->id().id() == dest_lane_segment_.id() &&
          adc_waypoint_.s < dest_lane_segment_.end_s() + 1.0));

    // adjust end_s to match reference_line_endpoint_extend_length
    if (last_lane_can_exit) {
      end_s = std::max(router_s + 1.0,
                       router_s + dest_lane_segment_.end_s() -
                           dest_lane_segment_.start_s() +
                           FLAGS_reference_line_endpoint_extend_length);
    }

    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
    AINFO << last_lane->id().id() << ", add length: " << length << ", router_s: " << router_s << ", end_s: " << end_s;
  }
  return true;
}

void LaneFollowMap::AppendLaneToPoints(
    hdmap::LaneInfoConstPtr lane, const double start_s, const double end_s,
    std::vector<hdmap::MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
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
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i],
                             hdmap::LaneWaypoint(lane, start_s));
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

void LaneFollowMap::UpdateRouteSegmentsLaneIds(
    const std::list<hdmap::RouteSegments> *route_segments) {
  route_segments_lane_ids_.clear();
  for (auto &route_seg : *route_segments) {
    for (auto &lane_seg : route_seg) {
      if (nullptr == lane_seg.lane) {
        continue;
      }
      route_segments_lane_ids_.insert(lane_seg.lane->id().id());
    }
  }
}

apollo::hdmap::LaneWaypoint LaneFollowMap::GetAdcWaypoint() const {
  return adc_waypoint_;
}

double LaneFollowMap::GetDistanceToDestination() const {
  if (adc_route_index_ < 0 || adc_route_index_ >= route_indices_.size()) {
    AERROR << "adc_route_index error, can not get distance to destination, "
              "return 0.";
    return 0.0;
  }
  const auto &routing = last_command_.lane_follow_command();
  int adc_road_index = route_indices_[adc_route_index_].index[0];
  int adc_passage_index = route_indices_[adc_route_index_].index[1];
  int adc_lane_index = route_indices_[adc_route_index_].index[2];

  bool get_adc_exit_waypoint =
      routing.road(adc_road_index).passage(adc_passage_index).can_exit();
  int start_passage_index = adc_passage_index;
  int start_lane_index = adc_lane_index;
  double start_lane_s = adc_waypoint_.s;

  double dis_to_destination = 0.0;

  for (int road_index = adc_road_index; road_index < routing.road_size();
       ++road_index) {
    const auto &road_segment = routing.road(road_index);
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      const auto &passage = road_segment.passage(passage_index);
      if (!passage.can_exit()) {
        continue;
      }
      // check this passsage is can_exit
      if (!get_adc_exit_waypoint) {
        get_adc_exit_waypoint = true;
        // find adc waypoint in passage which can exit
        for (int index = 0; index < passage.segment_size(); ++index) {
          auto lane = hdmap_->GetLaneById(
              hdmap::MakeMapId(passage.segment(index).id()));
          double s = 0.0;
          double l = 0.0;
          if (!lane->GetProjection({adc_state_.x(), adc_state_.y()}, &s, &l)) {
            continue;
          }
          static constexpr double kEpsilon = 0.5;
          if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
            continue;
          }
          start_passage_index = passage_index;
          start_lane_index = index;
          start_lane_s = s;
        }
      }

      // add distance by lane in can_exit passage
      if (get_adc_exit_waypoint) {
        if (road_index == adc_road_index &&
            passage_index == start_passage_index) {
          for (int lane_index = start_lane_index;
               lane_index < passage.segment_size(); ++lane_index) {
            if (lane_index == start_lane_index) {
              dis_to_destination +=
                  passage.segment(lane_index).end_s() - start_lane_s;
            } else {
              dis_to_destination += passage.segment(lane_index).end_s() -
                                    passage.segment(lane_index).start_s();
            }
          }
        } else {
          for (int lane_index = 0; lane_index < passage.segment_size();
               ++lane_index) {
            dis_to_destination += passage.segment(lane_index).end_s() -
                                  passage.segment(lane_index).start_s();
          }
        }
        break;
      }
    }
  }
  return dis_to_destination;
}

}  // namespace planning
}  // namespace apollo
