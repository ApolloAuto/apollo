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
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/map/hdmap/hdmap_util.h"
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

using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleState;
using apollo::common::adapter::AdapterManager;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneWaypoint;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::RouteSegments;

ReferenceLineProvider::~ReferenceLineProvider() {
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

ReferenceLineProvider::ReferenceLineProvider(const hdmap::HDMap *base_map) {
  if (!FLAGS_use_navigation_mode) {
    pnc_map_.reset(new hdmap::PncMap(base_map));
  }
  CHECK(common::util::GetProtoFromFile(FLAGS_smoother_config_filename,
                                       &smoother_config_))
      << "Failed to load smoother config file "
      << FLAGS_smoother_config_filename;
  if (smoother_config_.has_qp_spline()) {
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_spiral()) {
    smoother_.reset(new SpiralReferenceLineSmoother(smoother_config_));
  } else {
    CHECK(false) << "unknown smoother config "
                 << smoother_config_.DebugString();
  }
  is_initialized_ = true;
}  // namespace planning

bool ReferenceLineProvider::UpdateRoutingResponse(
    const routing::RoutingResponse &routing) {
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);
  routing_ = routing;
  has_routing_ = true;
  return true;
}

std::vector<routing::LaneWaypoint>
ReferenceLineProvider::FutureRouteWaypoints() {
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  return pnc_map_->FutureRouteWaypoints();
}

void ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
  vehicle_state_ = vehicle_state;
}

bool ReferenceLineProvider::Start() {
  if (FLAGS_use_navigation_mode) {
    return true;
  }
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

void ReferenceLineProvider::UpdateReferenceLine(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &route_segments) {
  if (reference_lines.size() != route_segments.size()) {
    AERROR << "The calculated reference line size(" << reference_lines.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  if (reference_lines_.size() != reference_lines.size()) {
    reference_lines_ = reference_lines;
    route_segments_ = route_segments;
    return;
  }
  auto segment_iter = route_segments.begin();
  auto internal_iter = reference_lines_.begin();
  auto internal_segment_iter = route_segments_.begin();
  for (auto iter = reference_lines.begin(); iter != reference_lines.end();
       ++iter, ++segment_iter, ++internal_iter, ++internal_segment_iter) {
    if (iter->reference_points().empty()) {
      *internal_iter = *iter;
      *internal_segment_iter = *segment_iter;
      continue;
    }
    if (common::util::SamePointXY(iter->reference_points().front(),
                                  internal_iter->reference_points().front()) &&
        common::util::SamePointXY(iter->reference_points().back(),
                                  internal_iter->reference_points().back()) &&
        std::fabs(iter->Length() - internal_iter->Length()) <
            common::math::kMathEpsilon) {
      continue;
    }
    *internal_iter = *iter;
    *internal_segment_iter = *segment_iter;
  }
}

void ReferenceLineProvider::GenerateThread() {
  constexpr int32_t kSleepTime = 50;  // milliseconds
  while (!is_stop_) {
    std::this_thread::yield();
    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(kSleepTime));
    double start_time = Clock::NowInSeconds();
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
    UpdateReferenceLine(reference_lines, segments);
    double end_time = Clock::NowInSeconds();
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    last_calculation_time_ = end_time - start_time;
  }
}

double ReferenceLineProvider::LastTimeDelay() {
  if (FLAGS_enable_reference_line_provider_thread &&
      !FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    return last_calculation_time_;
  } else {
    return last_calculation_time_;
  }
}

bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  if (FLAGS_use_navigation_mode) {
    double start_time = Clock::NowInSeconds();
    bool result = GetReferenceLinesFromRelativeMap(
        AdapterManager::GetRelativeMap()->GetLatestObserved(), reference_lines,
        segments);
    if (!result) {
      AERROR << "Failed to get reference line from relative map";
    }
    double end_time = Clock::NowInSeconds();
    last_calculation_time_ = end_time - start_time;
    return result;
  }

  if (FLAGS_enable_reference_line_provider_thread) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);

    if (!reference_lines_.empty()) {
      reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
      segments->assign(route_segments_.begin(), route_segments_.end());
      return true;
    } else {
      AWARN << "Reference line is NOT ready.";
      return false;
    }
  } else {
    double start_time = Clock::NowInSeconds();
    if (!CreateReferenceLine(reference_lines, segments)) {
      AERROR << "Failed to create reference line";
      return false;
    }
    UpdateReferenceLine(*reference_lines, *segments);
    double end_time = Clock::NowInSeconds();
    last_calculation_time_ = end_time - start_time;
    return true;
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

bool ReferenceLineProvider::GetReferenceLinesFromRelativeMap(
    const relative_map::MapMsg &relative_map,
    std::list<ReferenceLine> *reference_line,
    std::list<hdmap::RouteSegments> *segments) {
  if (relative_map.navigation_path_size() <= 0) {
    return false;
  }
  auto *hdmap = HDMapUtil::BaseMapPtr();
  for (const auto path_pair : relative_map.navigation_path()) {
    const auto &lane_id = path_pair.first;
    const auto &path_points = path_pair.second.path().path_point();
    auto lane_ptr = hdmap->GetLaneById(hdmap::MakeMapId(lane_id));
    RouteSegments segment;
    segment.emplace_back(lane_ptr, 0.0, lane_ptr->total_length());
    segment.SetCanExit(true);
    segment.SetId(lane_id);
    segment.SetNextAction(routing::FORWARD);
    segment.SetIsOnSegment(true);
    segment.SetStopForDestination(false);
    segment.SetPreviousAction(routing::FORWARD);
    segments->emplace_back(segment);
    std::vector<ReferencePoint> ref_points;
    for (const auto &path_point : path_points) {
      ref_points.emplace_back(
          MapPathPoint{Vec2d{path_point.x(), path_point.y()},
                       path_point.theta(),
                       LaneWaypoint(lane_ptr, path_point.s())},
          path_point.kappa(), path_point.dkappa());
    }
    reference_line->emplace_back(ref_points.begin(), ref_points.end());
  }
  return true;
}

bool ReferenceLineProvider::CreateRouteSegments(
    const common::VehicleState &vehicle_state,
    const double look_backward_distance, const double look_forward_distance,
    std::list<hdmap::RouteSegments> *segments) {
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!pnc_map_->GetRouteSegments(vehicle_state, look_backward_distance,
                                    look_forward_distance, segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }

  if (FLAGS_prioritize_change_lane) {
    PrioritzeChangeLane(segments);
  }
  return !segments->empty();
}

double ReferenceLineProvider::LookForwardDistance(const VehicleState &state) {
  auto forward_distance = state.linear_velocity() * FLAGS_look_forward_time_sec;

  if (forward_distance > FLAGS_look_forward_short_distance) {
    return FLAGS_look_forward_long_distance;
  }

  return FLAGS_look_forward_short_distance;
}

bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  common::VehicleState vehicle_state;
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }

  routing::RoutingResponse routing;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    routing = routing_;
  }
  bool is_new_routing = false;
  {
    // Update routing in pnc_map
    if (pnc_map_->IsNewRouting(routing)) {
      is_new_routing = true;
      if (!pnc_map_->UpdateRoutingResponse(routing)) {
        AERROR << "Failed to update routing in pnc map";
        return false;
      }
    }
  }

  double look_forward_distance = LookForwardDistance(vehicle_state);
  double look_backward_distance = FLAGS_look_backward_distance;
  if (!CreateRouteSegments(vehicle_state, look_backward_distance,
                           look_forward_distance, segments)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }
  if (is_new_routing || !FLAGS_enable_reference_line_stitching) {
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
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Could not further extend reference line";
    return true;
  }
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

AnchorPoint ReferenceLineProvider::GetAnchorPoint(
    const ReferenceLine &reference_line, double s) const {
  AnchorPoint anchor;
  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
  auto ref_point = reference_line.GetReferencePoint(s);
  if (ref_point.lane_waypoints().empty()) {
    anchor.path_point = ref_point.ToPathPoint(s);
    anchor.lateral_bound = smoother_config_.lateral_boundary_bound();
    return anchor;
  }
  const double adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width = adc_width / 2.0;
  const Vec2d left_vec =
      Vec2d::CreateUnitVec2d(ref_point.heading() + M_PI / 2.0);
  auto waypoint = ref_point.lane_waypoints().front();
  // shift to center
  double left_width = 0.0;
  double right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);
  double total_width = left_width + right_width;
  // only need to track left side width shift
  double shifted_left_width = total_width / 2.0;

  // shift to left (or right) on wide lanes
  if (!(waypoint.lane->lane().left_boundary().virtual_() ||
        waypoint.lane->lane().right_boundary().virtual_()) &&
      total_width > adc_width * smoother_config_.wide_lane_threshold_factor()) {
    if (smoother_config_.driving_side() == ReferenceLineSmootherConfig::RIGHT) {
      shifted_left_width =
          adc_half_width +
          adc_width * smoother_config_.wide_lane_shift_remain_factor();
    } else {
      shifted_left_width = std::fmax(
          adc_half_width,
          total_width -
              (adc_half_width +
               adc_width * smoother_config_.wide_lane_shift_remain_factor()));
    }
  }

  // shift away from curb boundary
  auto left_type = hdmap::LeftBoundaryType(waypoint);
  if (left_type == hdmap::LaneBoundaryType::CURB) {
    shifted_left_width += smoother_config_.curb_shift();
  }
  auto right_type = hdmap::RightBoundaryType(waypoint);
  if (right_type == hdmap::LaneBoundaryType::CURB) {
    shifted_left_width -= smoother_config_.curb_shift();
  }

  ref_point += left_vec * (left_width - shifted_left_width);
  auto shifted_right_width = total_width - shifted_left_width;
  anchor.path_point = ref_point.ToPathPoint(s);
  double effective_width = std::min(shifted_left_width, shifted_right_width) -
                           adc_half_width - FLAGS_reference_line_lateral_buffer;
  anchor.lateral_bound =
      std::max(smoother_config_.lateral_boundary_bound(), effective_width);
  return anchor;
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
    anchor_points->emplace_back(GetAnchorPoint(reference_line, s));
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
    auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
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
