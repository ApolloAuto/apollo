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
    const ReferenceLineSmootherConfig &smoother_config) {
  pnc_map_.reset(new hdmap::PncMap(hdmap_));
  smoother_config_ = smoother_config;
  std::vector<double> init_t_knots;
  spline_solver_.reset(new Spline2dSolver(init_t_knots, 1));
  is_initialized_ = true;
}

void ReferenceLineProvider::UpdateRoutingResponse(
    const routing::RoutingResponse &routing) {
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  pnc_map_->UpdateRoutingResponse(routing);
  has_routing_ = true;
}

bool ReferenceLineProvider::Start() {
  if (!is_initialized_) {
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }
  const auto &func = [this] { Generate(); };
  thread_.reset(new std::thread(func));
  return true;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

void ReferenceLineProvider::Generate() {
  while (!is_stop_) {
    const auto &curr_adc_position =
        common::VehicleState::instance()->pose().position();
    const auto adc_point_enu = common::util::MakePointENU(
        curr_adc_position.x(), curr_adc_position.y(), curr_adc_position.z());
    if (!has_routing_) {
      AERROR << "Routing is not ready.";
      constexpr int32_t kRoutingNotReadySleepTimeMs = 500;  // milliseconds
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(
          kRoutingNotReadySleepTimeMs));
      continue;
    }
    if (!CreateReferenceLineFromRouting(adc_point_enu)) {
      AERROR << "Fail to create reference line at position: "
             << curr_adc_position.ShortDebugString();
    }
    ADEBUG << "ReferenceLine smoothed with adc position: "
           << curr_adc_position.ShortDebugString();

    constexpr int32_t kReferenceLineProviderSleepTime = 200;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(
        kReferenceLineProviderSleepTime));
  }
}

std::vector<ReferenceLine> ReferenceLineProvider::GetReferenceLines() {
  // TODO(all): implement this function using the current adc position and the
  // existing reference lines. It is required that the current reference lines
  // can cover thoroughly the current adc position so that planning can be make
  // with a minimum planning distance of 100 meters ahead and 10 meters
  // backward.
  std::vector<ReferenceLine> reference_lines;
  if (reference_line_groups_.empty()) {
    return reference_lines;
  }
  std::lock_guard<std::mutex> lock(reference_line_groups_mutex_);
  return reference_line_groups_.back();
}

bool ReferenceLineProvider::CreateReferenceLineFromRouting(
    const common::PointENU &position) {
  std::vector<hdmap::RouteSegments> route_segments;

  const auto &adc_speed = common::VehicleState::instance()->linear_velocity();
  double look_forward_distance = (adc_speed * FLAGS_look_forward_time_sec >
                                  FLAGS_look_forward_min_distance)
                                     ? FLAGS_look_forward_distance
                                     : FLAGS_look_forward_min_distance;
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!pnc_map_->GetRouteSegments(position, FLAGS_look_backward_distance,
                                    look_forward_distance, &route_segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }

  ReferenceLineSmoother smoother;
  smoother.Init(smoother_config_);

  std::vector<ReferenceLine> reference_lines;
  for (const auto &segments : route_segments) {
    hdmap::Path hdmap_path;
    hdmap::PncMap::CreatePathFromLaneSegments(segments, &hdmap_path);
    if (FLAGS_enable_smooth_reference_line) {
      ReferenceLine raw_reference_line(hdmap_path);
      ReferenceLine reference_line;
      if (!smoother.Smooth(raw_reference_line, &reference_line,
                           spline_solver_.get())) {
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
        reference_lines.push_back(std::move(reference_line));
        reference_lines.back().set_change_lane_type(
            segments.change_lane_type());
      }
    } else {
      reference_lines.emplace_back(hdmap_path);
      reference_lines.back().set_change_lane_type(segments.change_lane_type());
    }
  }

  if (reference_lines.empty()) {
    AERROR << "No smooth reference lines available";
    return false;
  }

  if (!reference_lines.empty()) {
    std::lock_guard<std::mutex> lock(reference_line_groups_mutex_);
    reference_line_groups_.push_back(reference_lines);
    const size_t kMaxStoredReferenceLineGroups = 3;
    while (reference_line_groups_.size() > kMaxStoredReferenceLineGroups) {
      reference_line_groups_.pop_front();
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
