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

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/reference_line_provider.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::VehicleState;

ReferenceLineProvider::~ReferenceLineProvider() {
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

void ReferenceLineProvider::Init(
    const hdmap::PncMap *pnc_map,
    const routing::RoutingResponse &routing_response,
    const ReferenceLineSmootherConfig &smoother_config) {
  pnc_map_ = pnc_map;
  routing_response_ = routing_response;
  smoother_config_ = smoother_config;
  is_initialized_ = true;
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

void ReferenceLineProvider::Generate() {
  // TODO: implement this function.

  while (true) {
    const auto &curr_adc_position =
        common::VehicleState::instance()->pose().position();
    const auto adc_point_enu = common::util::MakePointENU(
        curr_adc_position.x(), curr_adc_position.y(), curr_adc_position.z());

    if (!CreateReferenceLineFromRouting(adc_point_enu, routing_response_)) {
      AERROR << "Fail to create reference line at position: "
             << curr_adc_position.ShortDebugString();
    };
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(500));
  }
}

std::vector<ReferenceLine> ReferenceLineProvider::GetReferenceLines() {
  // TODO: implement this function using the current adc position and the
  // existing reference lines. It is required that the current reference lines
  // can cover thoroughly the current adc position so that planning can be make
  // with a minimum planning distance of 100 meters ahead and 10 meters
  // backward.
  while (reference_line_groups_.empty()) {
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(20));
  }
  std::lock_guard<std::mutex> lock(reference_line_groups_mutex_);
  return reference_line_groups_.back();
}

bool ReferenceLineProvider::CreateReferenceLineFromRouting(
    const common::PointENU &position, const routing::RoutingResponse &routing) {
  std::vector<std::vector<hdmap::LaneSegment>> route_segments;

  const double kBackwardDistance = 20;
  const double kForwardDistance = 100;
  if (!pnc_map_->GetLaneSegmentsFromRouting(routing, position,
                                            kBackwardDistance, kForwardDistance,
                                            &route_segments)) {
    AERROR << "Failed to extract segments from routing";
    return false;
  }

  ReferenceLineSmoother smoother;
  smoother.Init(smoother_config_);

  std::vector<ReferenceLine> reference_lines;
  // TODO: Added code to enable partially smoothed reference line here.
  for (const auto &segments : route_segments) {
    hdmap::Path hdmap_path;
    pnc_map_->CreatePathFromLaneSegments(segments, &hdmap_path);
    if (FLAGS_enable_smooth_reference_line) {
      ReferenceLine reference_line;
      if (!smoother.Smooth(ReferenceLine(hdmap_path), &reference_line)) {
        AERROR << "Failed to smooth reference line";
        continue;
      }
      reference_lines.push_back(std::move(reference_line));
    } else {
      reference_lines.emplace_back(hdmap_path);
    }
  }

  if (reference_lines.empty()) {
    AERROR << "No smooth reference lines available";
    return false;
  }

  std::lock_guard<std::mutex> lock(reference_line_groups_mutex_);
  reference_line_groups_.push_back(reference_lines);
  const size_t kMaxStoredReferenceLineGroups = 3;
  while (reference_line_groups_.size() > kMaxStoredReferenceLineGroups) {
    reference_line_groups_.pop_front();
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
