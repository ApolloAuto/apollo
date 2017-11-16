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
 * @file
 *
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include <algorithm>
#include <limits>
#include <utility>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/routing/common/routing_gflags.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::RouteSegments;
using apollo::hdmap::LaneWaypoint;

ReferenceLineProvider::ReferenceLineProvider() {}

ReferenceLineProvider::~ReferenceLineProvider() {
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

void ReferenceLineProvider::Init(
    const hdmap::HDMap *hdmap_,
    const QpSplineReferenceLineSmootherConfig &smoother_config) {
  pnc_map_.reset(new hdmap::PncMap(hdmap_));
  smoother_config_ = smoother_config;
  segment_history_.clear();
  std::vector<double> init_t_knots;
  spline_solver_.reset(new Spline2dSolver(init_t_knots, 1));
  if (FLAGS_enable_spiral_reference_line) {
    smoother_.reset(
        new SpiralReferenceLineSmoother(FLAGS_spiral_smoother_max_deviation));
  } else {
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_,
                                                      spline_solver_.get()));
  }
  is_initialized_ = true;
}

bool ReferenceLineProvider::IsAllowChangeLane(
    const common::math::Vec2d &point,
    const std::list<RouteSegments> &route_segments) {
  if (FLAGS_reckless_change_lane) {
    ADEBUG << "enabled reckless change lane enabled";
    return true;
  }
  if (route_segments.size() <= 1) {
    return false;
  }
  auto forward_segment = route_segments.begin();
  while (forward_segment != route_segments.end() &&
         !forward_segment->IsOnSegment()) {
    ++forward_segment;
  }
  if (forward_segment == route_segments.end()) {
    return true;
  }
  common::SLPoint sl;
  LaneWaypoint waypoint;
  if (!forward_segment->GetProjection(point, &sl, &waypoint)) {
    AERROR << "Failed to project to forward segment from point: "
           << point.DebugString();
    return false;
  }
  auto history_iter = segment_history_.find(forward_segment->Id());
  if (history_iter == segment_history_.end()) {
    auto &inserter = segment_history_[forward_segment->Id()];
    inserter.min_l = std::fabs(sl.l());
    inserter.last_point = point;
    inserter.accumulate_s = 0.0;
    return false;
  } else {
    history_iter->second.min_l =
        std::min(history_iter->second.min_l, std::fabs(sl.l()));
    double dist =
        common::util::DistanceXY(history_iter->second.last_point, point);
    history_iter->second.last_point = point;
    history_iter->second.accumulate_s += dist;
    constexpr double kChangeLaneMinL = 0.25;
    constexpr double kChangeLaneMinLengthFactor = 0.6;
    if (history_iter->second.min_l < kChangeLaneMinL &&
        history_iter->second.accumulate_s >=
            kChangeLaneMinLengthFactor * FLAGS_min_length_for_lane_change) {
      return true;
    }
  }
  return false;
}

bool ReferenceLineProvider::UpdateRoutingResponse(
    const routing::RoutingResponse &routing) {
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  if (!pnc_map_->UpdateRoutingResponse(routing)) {
    AERROR << "Failed to update routing in pnc map";
    return false;
  }
  if (!pnc_map_->IsSameRouting()) {
    segment_history_.clear();
  }
  has_routing_ = true;
  return true;
}

void ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  vehicle_state_ = vehicle_state;
}

bool ReferenceLineProvider::Start() {
  if (!is_initialized_) {
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }
  if (FLAGS_enable_reference_line_provider_thread) {
    thread_.reset(
        new std::thread(&ReferenceLineProvider::GenerateThread, this));
  }
  return true;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_reference_line_provider_thread && thread_ &&
      thread_->joinable()) {
    thread_->join();
  }
}

void ReferenceLineProvider::GenerateThread() {
  constexpr int32_t kSleepTime = 200;  // milliseconds
  while (!is_stop_) {
    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(kSleepTime));
    if (!has_routing_) {
      AERROR << "Routing is not ready.";
      continue;
    }
    std::list<ReferenceLine> reference_lines;
    std::list<hdmap::RouteSegments> segments;
    if (!CreateReferenceLine(&reference_lines, &segments)) {
      AERROR << "Fail to get reference line";
      continue;
    }
    std::unique_lock<std::mutex> lock(reference_lines_mutex__);
    reference_lines_ = reference_lines;
    route_segments_ = segments;
    lock.unlock();
    cv_has_reference_line_.notify_one();
  }
}

bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);
  if (FLAGS_enable_reference_line_provider_thread) {
    std::unique_lock<std::mutex> lock(reference_lines_mutex__);
    cv_has_reference_line_.wait(lock,
                                [this]() { return !reference_lines_.empty(); });
    reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
    segments->assign(route_segments_.begin(), route_segments_.end());
    lock.unlock();
    return true;
  } else {
    return CreateReferenceLine(reference_lines, segments);
  }
}

void ReferenceLineProvider::PrioritzeChangeLane(
    std::list<hdmap::RouteSegments> *route_segments) {
  CHECK_NOTNULL(route_segments);
  auto iter = route_segments->begin();
  while (iter != route_segments->end()) {
    if (!iter->IsOnSegment()) {
      route_segments->splice(route_segments->begin(), *route_segments, iter);
      break;
    }
    ++iter;
  }
}

bool ReferenceLineProvider::CreateRouteSegments(
    const common::VehicleState &vehicle_state, double look_backward_distance,
    double look_forward_distance, std::list<hdmap::RouteSegments> *segments) {
  common::math::Vec2d point;
  point.set_x(vehicle_state.x());
  point.set_y(vehicle_state.y());
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!pnc_map_->GetRouteSegments(vehicle_state, look_backward_distance,
                                    look_forward_distance, segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }
  bool is_allow_change_lane = IsAllowChangeLane(point, *segments);
  for (auto iter = segments->begin(); iter != segments->end();) {
    if (!is_allow_change_lane && !iter->IsOnSegment()) {
      iter = segments->erase(iter);
    } else {
      ++iter;
    }
  }
  if (FLAGS_prioritize_change_lane) {
    PrioritzeChangeLane(segments);
  }
  return !segments->empty();
}

double LookForwardDistance(const VehicleState &state) {
  return (state.linear_velocity() * FLAGS_look_forward_time_sec >
          FLAGS_look_forward_min_distance)
             ? FLAGS_look_forward_distance
             : FLAGS_look_forward_min_distance;
}

bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  common::VehicleState vehicle_state;
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    vehicle_state = vehicle_state_;
  }
  double look_forward_distance = LookForwardDistance(vehicle_state);
  double look_backward_distance = FLAGS_look_backward_distance;
  if (!CreateRouteSegments(vehicle_state, look_backward_distance,
                           look_forward_distance, segments)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }
  if (!FLAGS_enable_reference_line_stitching) {
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
    return true;
  } else {  // stitching reference line
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
  }
  return true;
}

bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,
                                                RouteSegments *segments,
                                                ReferenceLine *reference_line) {
  RouteSegments segment_properties;
  segment_properties.SetProperties(*segments);
  auto prev_segment = route_segments_.begin();
  auto prev_ref = reference_lines_.begin();
  while (prev_segment != route_segments_.end()) {
    if (prev_segment->IsConnectedSegment(*segments)) {
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }
  if (prev_segment == route_segments_.end()) {
    if (!route_segments_.empty() && segments->IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(*segments, reference_line);
  }
  common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;
  if (!prev_segment->GetProjection(vec2d, &sl_point, &waypoint)) {
    AWARN << "Vehicle current point: " << vec2d.DebugString()
          << " not on previous reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  const double prev_segment_length = RouteSegments::Length(*prev_segment);
  const double remain_s = prev_segment_length - sl_point.s();
  const double look_forward_required_distance = LookForwardDistance(state);
  if (remain_s > look_forward_required_distance) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Reference line remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    return true;
  }
  double future_start_s =
      std::max(sl_point.s(), prev_segment_length -
                                 FLAGS_reference_line_stitch_overlap_distance);
  double future_end_s =
      prev_segment_length + FLAGS_look_forward_extend_distance;
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  if (!pnc_map_->ExtendSegments(*prev_segment, future_start_s, future_end_s,
                                &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);
  }
  lock.unlock();
  hdmap::Path path;
  hdmap::PncMap::CreatePathFromLaneSegments(shifted_segments, &path);
  ReferenceLine new_ref(path);
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line)) {
    AWARN << "Failed to smooth forward shifted reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!reference_line->Stitch(*prev_ref)) {
    AWARN << "Failed to stitch reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }
  *segments = shifted_segments;
  segments->SetProperties(segment_properties);
  common::SLPoint sl;
  if (!reference_line->XYToSL(vec2d, &sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched reference line";
  }
  if (sl.s() > FLAGS_look_backward_distance * 1.5) {
    ADEBUG << "reference line back side is " << sl.s()
           << ", shrink reference line: origin lenght: "
           << reference_line->Length();
    if (!reference_line->Shrink(vec2d, FLAGS_look_backward_distance,
                                std::numeric_limits<double>::infinity())) {
      AWARN << "Failed to shrink reference line";
    }
    if (!segments->Shrink(vec2d, FLAGS_look_backward_distance,
                          std::numeric_limits<double>::infinity())) {
      AWARN << "Failed to shrink route segment";
    }
  }
  return true;
}

bool ReferenceLineProvider::IsReferenceLineSmoothValid(
    const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  constexpr double kReferenceLineDiffCheckStep = 10.0;
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);

    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {
      AERROR << "Fail to change xy point on smoothed reference line to sl "
                "point respect to raw reference line.";
      return false;
    }

    const double diff = std::fabs(sl_new.l());
    if (diff > FLAGS_smoothed_reference_line_max_diff) {
      AERROR << "Fail to provide reference line because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}

void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,
    std::vector<AnchorPoint> *anchor_points) const {
  CHECK_NOTNULL(anchor_points);
  const double interval = smoother_config_.max_constraint_interval();
  int num_of_anchors =
      std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                              &anchor_s);
  for (const double s : anchor_s) {
    anchor_points->emplace_back();
    auto &last_anchor = anchor_points->back();
    auto ref_point = reference_line.GetReferencePoint(s);
    last_anchor.path_point = ref_point.ToPathPoint(s);
    last_anchor.longitudinal_bound =
        smoother_config_.longitudinal_boundary_bound();
    last_anchor.lateral_bound = smoother_config_.lateral_boundary_bound();
  }
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}

bool ReferenceLineProvider::SmoothRouteSegment(const RouteSegments &segments,
                                               ReferenceLine *reference_line) {
  hdmap::Path path;
  hdmap::PncMap::CreatePathFromLaneSegments(segments, &path);
  return SmoothReferenceLine(ReferenceLine(path), reference_line);
}

bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const ReferenceLine &prefix_ref, const ReferenceLine &raw_ref,
    ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_ref;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);
  // modify anchor points based on prefix_ref
  for (auto &point : anchor_points) {
    common::SLPoint sl_point;
    Vec2d xy{point.path_point.x(), point.path_point.y()};
    if (!prefix_ref.XYToSL(xy, &sl_point)) {
      AERROR << "Failed to get projection for point: " << xy.DebugString();
      return false;
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue;
    }
    auto prefix_ref_point = prefix_ref.GetNearestReferencepoint(sl_point.s());
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
    point.path_point.set_dkappa(prefix_ref_point.dkappa());
    point.longitudinal_bound = 1e-6;
    point.lateral_bound = 1e-6;
    point.enforced = true;
    break;
  }

  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, reference_line)) {
    AERROR << "Failed to smooth prefixed reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_ref, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

bool ReferenceLineProvider::SmoothReferenceLine(
    const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_reference_line;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_reference_line, &anchor_points);
  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_reference_line, reference_line)) {
    AERROR << "Failed to smooth reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
