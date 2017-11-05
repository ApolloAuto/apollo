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
 * @file reference_line_provider.cc
 *
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include <utility>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::VehicleState;

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

bool ReferenceLineProvider::UpdateRoutingResponse(
    const routing::RoutingResponse &routing) {
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  if (!pnc_map_->UpdateRoutingResponse(routing)) {
    AERROR << "Failed to update routing in pnc map";
    return false;
  }
  has_routing_ = true;
  return true;
}

bool ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  vehicle_state_ = vehicle_state;
  if (!pnc_map_->UpdateVehicleState(vehicle_state_)) {
    AERROR << "PncMap failed to update vehicle state: "
           << vehicle_state_.ShortDebugString();
    return false;
  }
  return true;
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
  constexpr size_t kMaxStoredReferenceLineGroups = 3;
  while (!is_stop_) {
    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(kSleepTime));
    if (!has_routing_) {
      AERROR << "Routing is not ready.";
      continue;
    }
    std::list<ReferenceLine> reference_lines;
    std::list<hdmap::RouteSegments> segments;
    if (!CreateReferenceLineFromRouting(&reference_lines, &segments)) {
      AERROR << "Fail to get reference line";
      continue;
    }
    std::unique_lock<std::mutex> lock(reference_line_groups_mutex_);
    reference_line_groups_.emplace_back(reference_lines);
    route_segment_groups_.emplace_back(segments);
    while (reference_line_groups_.size() > kMaxStoredReferenceLineGroups) {
      reference_line_groups_.pop_front();
      route_segment_groups_.pop_front();
    }
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
    std::unique_lock<std::mutex> lock(reference_line_groups_mutex_);
    cv_has_reference_line_.wait(
        lock, [this]() { return !reference_line_groups_.empty(); });
    reference_lines->assign(reference_line_groups_.back().begin(),
                            reference_line_groups_.back().end());
    segments->assign(route_segment_groups_.back().begin(),
                     route_segment_groups_.back().end());
    lock.unlock();
    return true;
  } else {
    return CreateReferenceLineFromRouting(reference_lines, segments);
  }
}

bool ReferenceLineProvider::CreateReferenceLineFromRouting(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  std::vector<hdmap::RouteSegments> route_segments;
  double look_forward_distance =
      (vehicle_state_.linear_velocity() * FLAGS_look_forward_time_sec >
       FLAGS_look_forward_min_distance)
          ? FLAGS_look_forward_distance
          : FLAGS_look_forward_min_distance;
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!pnc_map_->GetRouteSegments(FLAGS_look_backward_distance,
                                    look_forward_distance, &route_segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }
  for (const auto &lanes : route_segments) {
    hdmap::Path hdmap_path;
    hdmap::PncMap::CreatePathFromLaneSegments(lanes, &hdmap_path);
    if (FLAGS_enable_smooth_reference_line) {
      ReferenceLine raw_reference_line(hdmap_path);
      ReferenceLine reference_line;
      if (!smoother_->Smooth(raw_reference_line, &reference_line)) {
        AERROR << "Failed to smooth reference line";
        continue;
      }

      bool is_valid_reference_line = true;
      const double kReferenceLineDiffCheckResolution = 5.0;
      for (int s = 0.0; s < raw_reference_line.Length();
           s += kReferenceLineDiffCheckResolution) {
        auto xy_old = raw_reference_line.GetReferencePoint(s);
        auto xy_new = reference_line.GetReferencePoint(s);
        const double diff = xy_old.DistanceTo(xy_new);

        if (diff > FLAGS_smoothed_reference_line_max_diff) {
          AERROR << "Fail to provide reference line because too large diff "
                    "between smoothed and raw reference lines. diff: "
                 << diff;
          is_valid_reference_line = false;
          break;
        }
      }
      if (is_valid_reference_line) {
        reference_lines->emplace_back(std::move(reference_line));
        segments->emplace_back(lanes);
      }
    } else {
      reference_lines->emplace_back(hdmap_path);
      segments->emplace_back(lanes);
    }
  }

  if (reference_lines->empty()) {
    AERROR << "No smooth reference lines available";
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
