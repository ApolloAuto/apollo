/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>

#include "absl/strings/str_cat.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfo;

namespace {
// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoints.
using PathBound = std::vector<PathBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id).
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;
}  // namespace

PathBoundsDecider::PathBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Skip the path boundary decision if reusing the path.
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  std::vector<PathBoundary> candidate_path_boundaries;
  // const TaskConfig& config = Decider::config_;

  // Initialization.
  InitPathBoundsDecider(*frame, *reference_line_info);

  // Generate the fallback path boundary.
  PathBound fallback_path_bound;
  Status ret =
      GenerateFallbackPathBound(*reference_line_info, &fallback_path_bound);
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a fallback path bound.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  if (fallback_path_bound.empty()) {
    const std::string msg = "Failed to get a valid fallback path boundary";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (!fallback_path_bound.empty()) {
    CHECK_LE(adc_frenet_l_, std::get<2>(fallback_path_bound[0]));
    CHECK_GE(adc_frenet_l_, std::get<1>(fallback_path_bound[0]));
  }
  // Update the fallback path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> fallback_path_bound_pair;
  for (size_t i = 0; i < fallback_path_bound.size(); ++i) {
    fallback_path_bound_pair.emplace_back(std::get<1>(fallback_path_bound[i]),
                                          std::get<2>(fallback_path_bound[i]));
  }
  candidate_path_boundaries.emplace_back(std::get<0>(fallback_path_bound[0]),
                                         kPathBoundsDeciderResolution,
                                         fallback_path_bound_pair);
  candidate_path_boundaries.back().set_label("fallback");

  // If pull-over is requested, generate pull-over path boundary.
  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  const bool plan_pull_over_path = pull_over_status->plan_pull_over_path();
  if (plan_pull_over_path) {
    PathBound pull_over_path_bound;
    Status ret = GeneratePullOverPathBound(*frame, *reference_line_info,
                                           &pull_over_path_bound);
    if (!ret.ok()) {
      AWARN << "Cannot generate a pullover path bound, do regular planning.";
    } else {
      ACHECK(!pull_over_path_bound.empty());
      CHECK_LE(adc_frenet_l_, std::get<2>(pull_over_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(pull_over_path_bound[0]));

      // Update the fallback path boundary into the reference_line_info.
      std::vector<std::pair<double, double>> pull_over_path_bound_pair;
      for (size_t i = 0; i < pull_over_path_bound.size(); ++i) {
        pull_over_path_bound_pair.emplace_back(
            std::get<1>(pull_over_path_bound[i]),
            std::get<2>(pull_over_path_bound[i]));
      }
      candidate_path_boundaries.emplace_back(
          std::get<0>(pull_over_path_bound[0]), kPathBoundsDeciderResolution,
          pull_over_path_bound_pair);
      candidate_path_boundaries.back().set_label("regular/pullover");

      reference_line_info->SetCandidatePathBoundaries(
          std::move(candidate_path_boundaries));
      ADEBUG << "Completed pullover and fallback path boundaries generation.";

      // set debug info in planning_data
      auto* pull_over_debug = reference_line_info->mutable_debug()
                                  ->mutable_planning_data()
                                  ->mutable_pull_over();
      pull_over_debug->mutable_position()->CopyFrom(
          pull_over_status->position());
      pull_over_debug->set_theta(pull_over_status->theta());
      pull_over_debug->set_length_front(pull_over_status->length_front());
      pull_over_debug->set_length_back(pull_over_status->length_back());
      pull_over_debug->set_width_left(pull_over_status->width_left());
      pull_over_debug->set_width_right(pull_over_status->width_right());

      return Status::OK();
    }
  }

  // If it's a lane-change reference-line, generate lane-change path boundary.
  if (FLAGS_enable_smarter_lane_change &&
      reference_line_info->IsChangeLanePath()) {
    PathBound lanechange_path_bound;
    Status ret = GenerateLaneChangePathBound(*reference_line_info,
                                             &lanechange_path_bound);
    if (!ret.ok()) {
      ADEBUG << "Cannot generate a lane-change path bound.";
      return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
    }
    if (lanechange_path_bound.empty()) {
      const std::string msg = "Failed to get a valid fallback path boundary";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // disable this change when not extending lane bounds to include adc
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc()) {
      CHECK_LE(adc_frenet_l_, std::get<2>(lanechange_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(lanechange_path_bound[0]));
    }
    // Update the fallback path boundary into the reference_line_info.
    std::vector<std::pair<double, double>> lanechange_path_bound_pair;
    for (size_t i = 0; i < lanechange_path_bound.size(); ++i) {
      lanechange_path_bound_pair.emplace_back(
          std::get<1>(lanechange_path_bound[i]),
          std::get<2>(lanechange_path_bound[i]));
    }
    candidate_path_boundaries.emplace_back(
        std::get<0>(lanechange_path_bound[0]), kPathBoundsDeciderResolution,
        lanechange_path_bound_pair);
    candidate_path_boundaries.back().set_label("regular/lanechange");
    RecordDebugInfo(lanechange_path_bound, "", reference_line_info);
    reference_line_info->SetCandidatePathBoundaries(
        std::move(candidate_path_boundaries));
    ADEBUG << "Completed lanechange and fallback path boundaries generation.";
    return Status::OK();
  }

  // Generate regular path boundaries.
  std::vector<LaneBorrowInfo> lane_borrow_info_list;
  lane_borrow_info_list.push_back(LaneBorrowInfo::NO_BORROW);

  if (reference_line_info->is_path_lane_borrow()) {
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    for (const auto& lane_borrow_direction :
         path_decider_status.decided_side_pass_direction()) {
      if (lane_borrow_direction == PathDeciderStatus::LEFT_BORROW) {
        lane_borrow_info_list.push_back(LaneBorrowInfo::LEFT_BORROW);
      } else if (lane_borrow_direction == PathDeciderStatus::RIGHT_BORROW) {
        lane_borrow_info_list.push_back(LaneBorrowInfo::RIGHT_BORROW);
      }
    }
  }

  // Try every possible lane-borrow option:
  // PathBound regular_self_path_bound;
  // bool exist_self_path_bound = false;
  for (const auto& lane_borrow_info : lane_borrow_info_list) {
    PathBound regular_path_bound;
    std::string blocking_obstacle_id = "";
    std::string borrow_lane_type = "";
    Status ret = GenerateRegularPathBound(
        *reference_line_info, lane_borrow_info, &regular_path_bound,
        &blocking_obstacle_id, &borrow_lane_type);
    if (!ret.ok()) {
      continue;
    }
    if (regular_path_bound.empty()) {
      continue;
    }
    // disable this change when not extending lane bounds to include adc
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc()) {
      CHECK_LE(adc_frenet_l_, std::get<2>(regular_path_bound[0]));
      CHECK_GE(adc_frenet_l_, std::get<1>(regular_path_bound[0]));
    }

    // Update the path boundary into the reference_line_info.
    std::vector<std::pair<double, double>> regular_path_bound_pair;
    for (size_t i = 0; i < regular_path_bound.size(); ++i) {
      regular_path_bound_pair.emplace_back(std::get<1>(regular_path_bound[i]),
                                           std::get<2>(regular_path_bound[i]));
    }
    candidate_path_boundaries.emplace_back(std::get<0>(regular_path_bound[0]),
                                           kPathBoundsDeciderResolution,
                                           regular_path_bound_pair);
    std::string path_label = "";
    switch (lane_borrow_info) {
      case LaneBorrowInfo::LEFT_BORROW:
        path_label = "left";
        break;
      case LaneBorrowInfo::RIGHT_BORROW:
        path_label = "right";
        break;
      default:
        path_label = "self";
        // exist_self_path_bound = true;
        // regular_self_path_bound = regular_path_bound;
        break;
    }
    // RecordDebugInfo(regular_path_bound, "", reference_line_info);
    candidate_path_boundaries.back().set_label(
        absl::StrCat("regular/", path_label, "/", borrow_lane_type));
    candidate_path_boundaries.back().set_blocking_obstacle_id(
        blocking_obstacle_id);
  }

  // Remove redundant boundaries.
  // RemoveRedundantPathBoundaries(&candidate_path_boundaries);

  // Success
  reference_line_info->SetCandidatePathBoundaries(
      std::move(candidate_path_boundaries));
  ADEBUG << "Completed regular and fallback path boundaries generation.";
  return Status::OK();
}

void PathBoundsDecider::InitPathBoundsDecider(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  common::TrajectoryPoint planning_start_point = frame.PlanningStartPoint();
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }
  ADEBUG << "Plan at the starting point: x = "
         << planning_start_point.path_point().x()
         << ", y = " << planning_start_point.path_point().y()
         << ", and angle = " << planning_start_point.path_point().theta();

  // Initialize some private variables.
  // ADC s/l info.
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  adc_frenet_s_ = adc_sl_info.first[0];
  adc_frenet_l_ = adc_sl_info.second[0];
  adc_frenet_sd_ = adc_sl_info.first[1];
  adc_frenet_ld_ = adc_sl_info.second[1] * adc_frenet_sd_;
  double offset_to_map = 0.0;
  reference_line.GetOffsetToMap(adc_frenet_s_, &offset_to_map);
  adc_l_to_lane_center_ = adc_frenet_l_ + offset_to_map;

  // ADC's lane width.
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_frenet_s_, &lane_left_width,
                                   &lane_right_width)) {
    AWARN << "Failed to get lane width at planning start point.";
    adc_lane_width_ = kDefaultLaneWidth;
  } else {
    adc_lane_width_ = lane_left_width + lane_right_width;
  }
}

common::TrajectoryPoint PathBoundsDecider::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

Status PathBoundsDecider::GenerateRegularPathBound(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
    std::string* const blocking_obstacle_id,
    std::string* const borrow_lane_type) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!GetBoundaryFromLanesAndADC(reference_line_info, lane_borrow_info, 0.1,
                                  path_bound, borrow_lane_type)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // TODO(jiacheng): once ready, limit the path boundary based on the
  //                 actual road boundary to avoid getting off-road.

  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = *path_bound;
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id->empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }
  // PathBoundsDebugString(*path_bound);

  // 4. Adjust the boundary considering dynamic obstacles
  // TODO(all): may need to implement this in the future.

  ADEBUG << "Completed generating path boundaries.";
  return Status::OK();
}

Status PathBoundsDecider::GenerateLaneChangePathBound(
    const ReferenceLineInfo& reference_line_info,
    std::vector<std::tuple<double, double, double>>* const path_bound) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  std::string dummy_borrow_lane_type;
  if (!GetBoundaryFromLanesAndADC(reference_line_info,
                                  LaneBorrowInfo::NO_BORROW, 0.1, path_bound,
                                  &dummy_borrow_lane_type, true)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 3. Remove the S-length of target lane out of the path-bound.
  GetBoundaryFromLaneChangeForbiddenZone(reference_line_info, path_bound);

  PathBound temp_path_bound = *path_bound;
  std::string blocking_obstacle_id;
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }

  ADEBUG << "Completed generating path boundaries.";
  return Status::OK();
}

Status PathBoundsDecider::GeneratePullOverPathBound(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    PathBound* const path_bound) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on road boundary
  if (!GetBoundaryFromRoads(reference_line_info, path_bound)) {
    const std::string msg =
        "Failed to decide a rough boundary based on road boundary.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  ConvertBoundarySAxisFromLaneCenterToRefLine(reference_line_info, path_bound);
  if (adc_frenet_l_ < std::get<1>(path_bound->front()) ||
      adc_frenet_l_ > std::get<2>(path_bound->front())) {
    const std::string msg =
        "ADC is outside road boundary already. Cannot generate pull-over path";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 2. Update boundary by lane boundary for pull_over
  UpdatePullOverBoundaryByLaneBoundary(reference_line_info, path_bound);
  // PathBoundsDebugString(*path_bound);

  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = *path_bound;
  std::string blocking_obstacle_id;
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_bound, &blocking_obstacle_id)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  auto* pull_over_status = injector_->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  // If already found a pull-over position, simply check if it's valid.
  int curr_idx = -1;
  if (pull_over_status->has_position()) {
    curr_idx = IsPointWithinPathBound(
        reference_line_info, pull_over_status->position().x(),
        pull_over_status->position().y(), *path_bound);
  }

  // If haven't found a pull-over position, search for one.
  if (curr_idx < 0) {
    auto pull_over_type = pull_over_status->pull_over_type();
    pull_over_status->Clear();
    pull_over_status->set_pull_over_type(pull_over_type);
    pull_over_status->set_plan_pull_over_path(true);

    std::tuple<double, double, double, int> pull_over_configuration;
    if (!SearchPullOverPosition(frame, reference_line_info, *path_bound,
                                &pull_over_configuration)) {
      const std::string msg = "Failed to find a proper pull-over position.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    curr_idx = std::get<3>(pull_over_configuration);

    // If have found a pull-over position, update planning-context
    pull_over_status->mutable_position()->set_x(
        std::get<0>(pull_over_configuration));
    pull_over_status->mutable_position()->set_y(
        std::get<1>(pull_over_configuration));
    pull_over_status->mutable_position()->set_z(0.0);
    pull_over_status->set_theta(std::get<2>(pull_over_configuration));
    pull_over_status->set_length_front(FLAGS_obstacle_lon_start_buffer);
    pull_over_status->set_length_back(FLAGS_obstacle_lon_end_buffer);
    pull_over_status->set_width_left(
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0);
    pull_over_status->set_width_right(
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0);

    ADEBUG << "Pull Over: x[" << pull_over_status->position().x() << "] y["
           << pull_over_status->position().y() << "] theta["
           << pull_over_status->theta() << "]";
  }

  // Trim path-bound properly
  while (static_cast<int>(path_bound->size()) - 1 >
         curr_idx + kNumExtraTailBoundPoint) {
    path_bound->pop_back();
  }
  for (size_t idx = curr_idx + 1; idx < path_bound->size(); ++idx) {
    std::get<1>((*path_bound)[idx]) = std::get<1>((*path_bound)[curr_idx]);
    std::get<2>((*path_bound)[idx]) = std::get<2>((*path_bound)[curr_idx]);
  }

  return Status::OK();
}

Status PathBoundsDecider::GenerateFallbackPathBound(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info, path_bound)) {
    const std::string msg = "Failed to initialize fallback path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  // 2. Decide a rough boundary based on lane info and ADC's position
  std::string dummy_borrow_lane_type;
  if (!GetBoundaryFromLanesAndADC(reference_line_info,
                                  LaneBorrowInfo::NO_BORROW, 0.5, path_bound,
                                  &dummy_borrow_lane_type, true)) {
    const std::string msg =
        "Failed to decide a rough fallback boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(*path_bound);

  ADEBUG << "Completed generating fallback path boundaries.";
  return Status::OK();
}

int PathBoundsDecider::IsPointWithinPathBound(
    const ReferenceLineInfo& reference_line_info, const double x,
    const double y,
    const std::vector<std::tuple<double, double, double>>& path_bound) {
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);
  if (point_sl.s() > std::get<0>(path_bound.back()) ||
      point_sl.s() <
          std::get<0>(path_bound.front()) - kPathBoundsDeciderResolution * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }
  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         std::get<0>(path_bound[idx_after]) < point_sl.s()) {
    ++idx_after;
  }
  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << std::get<1>(path_bound[idx_after]) << ", "
         << std::get<2>(path_bound[idx_after]) << "].";
  ADEBUG << "The point is at: " << point_sl.l();
  int idx_before = idx_after - 1;
  if (std::get<1>(path_bound[idx_before]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_before]) >= point_sl.l() &&
      std::get<1>(path_bound[idx_after]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_after]) >= point_sl.l()) {
    return idx_after;
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

bool PathBoundsDecider::FindDestinationPullOverS(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::vector<std::tuple<double, double, double>>& path_bound,
    double* pull_over_s) {
  // destination_s based on routing_end
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint destination_sl;
  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  reference_line.XYToSL(routing_end.pose(), &destination_sl);
  const double destination_s = destination_sl.s();
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  // Check if destination is some distance away from ADC.
  ADEBUG << "Destination s[" << destination_s << "] adc_end_s[" << adc_end_s
         << "]";
  if (destination_s - adc_end_s < config_.path_bounds_decider_config()
                                      .pull_over_destination_to_adc_buffer()) {
    AERROR << "Destination is too close to ADC. distance["
           << destination_s - adc_end_s << "]";
    return false;
  }

  // Check if destination is within path-bounds searching scope.
  const double destination_to_pathend_buffer =
      config_.path_bounds_decider_config()
          .pull_over_destination_to_pathend_buffer();
  if (destination_s + destination_to_pathend_buffer >=
      std::get<0>(path_bound.back())) {
    AERROR << "Destination is not within path_bounds search scope";
    return false;
  }

  *pull_over_s = destination_s;
  return true;
}

bool PathBoundsDecider::FindEmergencyPullOverS(
    const ReferenceLineInfo& reference_line_info, double* pull_over_s) {
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const double min_turn_radius = common::VehicleConfigHelper::Instance()
                                     ->GetConfig()
                                     .vehicle_param()
                                     .min_turn_radius();
  const double adjust_factor =
      config_.path_bounds_decider_config()
          .pull_over_approach_lon_distance_adjust_factor();
  const double pull_over_distance = min_turn_radius * 2 * adjust_factor;
  *pull_over_s = adc_end_s + pull_over_distance;

  return true;
}

bool PathBoundsDecider::SearchPullOverPosition(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::vector<std::tuple<double, double, double>>& path_bound,
    std::tuple<double, double, double, int>* const pull_over_configuration) {
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();

  // search direction
  bool search_backward = false;  // search FORWARD by default

  double pull_over_s = 0.0;
  if (pull_over_status.pull_over_type() ==
      PullOverStatus::EMERGENCY_PULL_OVER) {
    if (!FindEmergencyPullOverS(reference_line_info, &pull_over_s)) {
      AERROR << "Failed to find emergency_pull_over s";
      return false;
    }
    search_backward = false;  // search FORWARD from target position
  } else if (pull_over_status.pull_over_type() == PullOverStatus::PULL_OVER) {
    if (!FindDestinationPullOverS(frame, reference_line_info, path_bound,
                                  &pull_over_s)) {
      AERROR << "Failed to find pull_over s upon destination arrival";
      return false;
    }
    search_backward = true;  // search BACKWARD from target position
  } else {
    return false;
  }

  int idx = 0;
  if (search_backward) {
    // 1. Locate the first point before destination.
    idx = static_cast<int>(path_bound.size()) - 1;
    while (idx >= 0 && std::get<0>(path_bound[idx]) > pull_over_s) {
      --idx;
    }
  } else {
    // 1. Locate the first point after emergency_pull_over s.
    while (idx < static_cast<int>(path_bound.size()) &&
           std::get<0>(path_bound[idx]) < pull_over_s) {
      ++idx;
    }
  }
  if (idx < 0 || idx >= static_cast<int>(path_bound.size())) {
    AERROR << "Failed to find path_bound index for pull over s";
    return false;
  }

  // Search for a feasible location for pull-over.
  const double pull_over_space_length =
      kPulloverLonSearchCoeff *
          VehicleConfigHelper::GetConfig().vehicle_param().length() -
      FLAGS_obstacle_lon_start_buffer - FLAGS_obstacle_lon_end_buffer;
  const double pull_over_space_width =
      (kPulloverLatSearchCoeff - 1.0) *
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  // 2. Find a window that is close to road-edge.
  // (not in any intersection)
  bool has_a_feasible_window = false;
  while ((search_backward && idx >= 0 &&
          std::get<0>(path_bound[idx]) - std::get<0>(path_bound.front()) >
              pull_over_space_length) ||
         (!search_backward && idx < static_cast<int>(path_bound.size()) &&
          std::get<0>(path_bound.back()) - std::get<0>(path_bound[idx]) >
              pull_over_space_length)) {
    int j = idx;
    bool is_feasible_window = true;

    // Check if the point of idx is within intersection.
    double pt_ref_line_s = std::get<0>(path_bound[idx]);
    double pt_ref_line_l = 0.0;
    common::SLPoint pt_sl;
    pt_sl.set_s(pt_ref_line_s);
    pt_sl.set_l(pt_ref_line_l);
    common::math::Vec2d pt_xy;
    reference_line_info.reference_line().SLToXY(pt_sl, &pt_xy);
    common::PointENU hdmap_point;
    hdmap_point.set_x(pt_xy.x());
    hdmap_point.set_y(pt_xy.y());
    ADEBUG << "Pull-over position might be around (" << pt_xy.x() << ", "
           << pt_xy.y() << ")";
    std::vector<std::shared_ptr<const JunctionInfo>> junctions;
    HDMapUtil::BaseMap().GetJunctions(hdmap_point, 1.0, &junctions);
    if (!junctions.empty()) {
      AWARN << "Point is in PNC-junction.";
      idx = search_backward ? idx - 1 : idx + 1;
      continue;
    }

    while ((search_backward && j >= 0 &&
            std::get<0>(path_bound[idx]) - std::get<0>(path_bound[j]) <
                pull_over_space_length) ||
           (!search_backward && j < static_cast<int>(path_bound.size()) &&
            std::get<0>(path_bound[j]) - std::get<0>(path_bound[idx]) <
                pull_over_space_length)) {
      double curr_s = std::get<0>(path_bound[j]);
      double curr_right_bound = std::fabs(std::get<1>(path_bound[j]));
      double curr_road_left_width = 0;
      double curr_road_right_width = 0;
      reference_line_info.reference_line().GetRoadWidth(
          curr_s, &curr_road_left_width, &curr_road_right_width);
      ADEBUG << "s[" << curr_s << "] curr_road_left_width["
             << curr_road_left_width << "] curr_road_right_width["
             << curr_road_right_width << "]";
      if (curr_road_right_width - (curr_right_bound + adc_half_width) >
          config_.path_bounds_decider_config().pull_over_road_edge_buffer()) {
        AERROR << "Not close enough to road-edge. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }
      const double right_bound = std::get<1>(path_bound[j]);
      const double left_bound = std::get<2>(path_bound[j]);
      ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
             << "]";
      if (left_bound - right_bound < pull_over_space_width) {
        AERROR << "Not wide enough to fit ADC. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }

      j = search_backward ? j - 1 : j + 1;
    }
    if (j < 0) {
      return false;
    }
    if (is_feasible_window) {
      has_a_feasible_window = true;
      const auto& reference_line = reference_line_info.reference_line();
      // estimate pull over point to have the vehicle keep same safety distance
      // to front and back
      const auto& vehicle_param =
          VehicleConfigHelper::GetConfig().vehicle_param();
      const double back_clear_to_total_length_ratio =
          (0.5 * (kPulloverLonSearchCoeff - 1.0) * vehicle_param.length() +
           vehicle_param.back_edge_to_center()) /
          vehicle_param.length() / kPulloverLonSearchCoeff;

      int start_idx = j;
      int end_idx = idx;
      if (!search_backward) {
        start_idx = idx;
        end_idx = j;
      }
      auto pull_over_idx = static_cast<size_t>(
          back_clear_to_total_length_ratio * static_cast<double>(end_idx) +
          (1.0 - back_clear_to_total_length_ratio) *
              static_cast<double>(start_idx));

      const auto& pull_over_point = path_bound[pull_over_idx];
      const double pull_over_s = std::get<0>(pull_over_point);
      const double pull_over_l =
          std::get<1>(pull_over_point) + pull_over_space_width / 2.0;
      common::SLPoint pull_over_sl_point;
      pull_over_sl_point.set_s(pull_over_s);
      pull_over_sl_point.set_l(pull_over_l);

      common::math::Vec2d pull_over_xy_point;
      reference_line.SLToXY(pull_over_sl_point, &pull_over_xy_point);
      const double pull_over_x = pull_over_xy_point.x();
      const double pull_over_y = pull_over_xy_point.y();

      // set the pull over theta to be the nearest lane theta rather than
      // reference line theta in case of reference line theta not aligned with
      // the lane
      const auto& reference_point =
          reference_line.GetReferencePoint(pull_over_s);
      double pull_over_theta = reference_point.heading();
      hdmap::LaneInfoConstPtr lane;
      double s = 0.0;
      double l = 0.0;
      auto point =
          common::util::PointFactory::ToPointENU(pull_over_x, pull_over_y);
      if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
              point, 5.0, pull_over_theta, M_PI_2, &lane, &s, &l) == 0) {
        pull_over_theta = lane->Heading(s);
      }
      *pull_over_configuration =
          std::make_tuple(pull_over_x, pull_over_y, pull_over_theta,
                          static_cast<int>(pull_over_idx));
      break;
    }

    idx = search_backward ? idx - 1 : idx + 1;
  }

  return has_a_feasible_window;
}

void PathBoundsDecider::RemoveRedundantPathBoundaries(
    std::vector<PathBoundary>* const candidate_path_boundaries) {
  // 1. Check to see if both "left" and "right" exist.
  bool is_left_exist = false;
  std::vector<std::pair<double, double>> left_boundary;
  bool is_right_exist = false;
  std::vector<std::pair<double, double>> right_boundary;
  for (const auto& path_boundary : *candidate_path_boundaries) {
    if (path_boundary.label().find("left") != std::string::npos) {
      is_left_exist = true;
      left_boundary = path_boundary.boundary();
    }
    if (path_boundary.label().find("right") != std::string::npos) {
      is_right_exist = true;
      right_boundary = path_boundary.boundary();
    }
  }
  // 2. Check if "left" is contained by "right", and vice versa.
  if (!is_left_exist || !is_right_exist) {
    return;
  }
  bool is_left_redundant = false;
  bool is_right_redundant = false;
  if (IsContained(left_boundary, right_boundary)) {
    is_left_redundant = true;
  }
  if (IsContained(right_boundary, left_boundary)) {
    is_right_redundant = true;
  }

  // 3. If one contains the other, then remove the redundant one.
  for (size_t i = 0; i < candidate_path_boundaries->size(); ++i) {
    const auto& path_boundary = (*candidate_path_boundaries)[i];
    if (path_boundary.label().find("right") != std::string::npos &&
        is_right_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
    if (path_boundary.label().find("left") != std::string::npos &&
        is_left_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
  }
}

bool PathBoundsDecider::IsContained(
    const std::vector<std::pair<double, double>>& lhs,
    const std::vector<std::pair<double, double>>& rhs) {
  if (lhs.size() > rhs.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].first < rhs[i].first) {
      return false;
    }
    if (lhs[i].second > rhs[i].second) {
      return false;
    }
  }
  return true;
}

bool PathBoundsDecider::InitPathBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  const auto& reference_line = reference_line_info.reference_line();

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  for (double curr_s = adc_frenet_s_;
       curr_s < std::fmin(adc_frenet_s_ +
                              std::fmax(kPathBoundsDeciderHorizon,
                                        reference_line_info.GetCruiseSpeed() *
                                            FLAGS_trajectory_time_length),
                          reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  // Return.
  if (path_bound->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

bool PathBoundsDecider::GetBoundaryFromRoads(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // Go through every point, update the boudnary based on the road boundary.
  double past_road_left_width = adc_lane_width_ / 2.0;
  double past_road_right_width = adc_lane_width_ / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      curr_road_left_width += refline_offset_to_lane_center;
      curr_road_right_width -= refline_offset_to_lane_center;
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;
    ADEBUG << "At s = " << curr_s
           << ", left road bound = " << curr_road_left_width
           << ", right road bound = " << curr_road_right_width
           << ", offset from refline to lane-center = "
           << refline_offset_to_lane_center;

    // 2. Update into path_bound.
    if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                      path_bound)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_bound);
  return true;
}

bool PathBoundsDecider::GetBoundaryFromLanes(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, PathBound* const path_bound,
    std::string* const borrow_lane_type) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // Go through every point, update the boundary based on lane-info.
  double past_lane_left_width = adc_lane_width_ / 2.0;
  double past_lane_right_width = adc_lane_width_ / 2.0;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);

    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // The left-width and right-width are w.r.t. lane-center, not ref-line.
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;
    if (CheckLaneBoundaryType(reference_line_info, curr_s, lane_borrow_info)) {
      hdmap::Id neighbor_lane_id;
      if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
        // Borrowing left neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow left forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow left reverse neighbor lane.";
        } else {
          ADEBUG << "There is no left neighbor lane.";
        }
      } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
        // Borrowing right neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow right forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow right reverse neighbor lane.";
        } else {
          ADEBUG << "There is no right neighbor lane.";
        }
      }
    }

    // 3. Get the proper boundary
    double curr_left_bound =
        curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);
    double curr_right_bound = -curr_lane_right_width -
                              (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
                                   ? curr_neighbor_lane_width
                                   : 0.0);
    ADEBUG << "At s = " << curr_s << ", left_lane_bound = " << curr_left_bound
           << ", right_lane_bound = " << curr_right_bound;

    // 4. Update the boundary.
    if (!UpdatePathBoundary(i, curr_left_bound, curr_right_bound, path_bound)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }
  TrimPathBounds(path_blocked_idx, path_bound);

  if (lane_borrow_info == LaneBorrowInfo::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }
  return true;
}

bool PathBoundsDecider::GetBoundaryFromADC(
    const ReferenceLineInfo& reference_line_info, double ADC_extra_buffer,
    PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());

  // Calculate the ADC's lateral boundary.
  static constexpr double kMaxLateralAccelerations = 1.5;
  double ADC_lat_decel_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                                adc_frenet_ld_ * adc_frenet_ld_ /
                                kMaxLateralAccelerations / 2.0;
  double curr_left_bound_adc =
      GetBufferBetweenADCCenterAndEdge() + ADC_extra_buffer +
      std::fmax(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);
  double curr_right_bound_adc =
      -GetBufferBetweenADCCenterAndEdge() - ADC_extra_buffer +
      std::fmin(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);

  // Expand the boundary in case ADC falls outside.
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_left_bound = std::get<2>((*path_bound)[i]);
    curr_left_bound = std::fmax(curr_left_bound_adc, curr_left_bound);
    double curr_right_bound = std::get<1>((*path_bound)[i]);
    curr_right_bound = std::fmin(curr_right_bound_adc, curr_right_bound);
    UpdatePathBoundary(i, curr_left_bound, curr_right_bound, path_bound);
  }
  return true;
}

// TODO(jiacheng): this function is to be retired soon.
bool PathBoundsDecider::GetBoundaryFromLanesAndADC(
    const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo& lane_borrow_info, double ADC_buffer,
    PathBound* const path_bound, std::string* const borrow_lane_type,
    bool is_fallback_lanechange) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  bool is_left_lane_boundary = true;
  bool is_right_lane_boundary = true;
  const double boundary_buffer = 0.05;  // meter

  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width_ / 2.0;
  double past_lane_right_width = adc_lane_width_ / 2.0;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      // check if lane boundary is also road boundary
      double curr_road_left_width = 0.0;
      double curr_road_right_width = 0.0;
      if (reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                      &curr_road_right_width)) {
        is_left_lane_boundary =
            (std::abs(curr_road_left_width - curr_lane_left_width) >
             boundary_buffer);
        is_right_lane_boundary =
            (std::abs(curr_road_right_width - curr_lane_right_width) >
             boundary_buffer);
      }
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;
    if (CheckLaneBoundaryType(reference_line_info, curr_s, lane_borrow_info)) {
      hdmap::Id neighbor_lane_id;
      if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
        // Borrowing left neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow left forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow left reverse neighbor lane.";
        } else {
          ADEBUG << "There is no left neighbor lane.";
        }
      } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
        // Borrowing right neighbor lane.
        if (reference_line_info.GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          ADEBUG << "Borrow right forward neighbor lane.";
        } else if (reference_line_info.GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          ADEBUG << "Borrow right reverse neighbor lane.";
        } else {
          ADEBUG << "There is no right neighbor lane.";
        }
      }
    }

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    static constexpr double kMaxLateralAccelerations = 1.5;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                              adc_frenet_ld_ * adc_frenet_ld_ /
                              kMaxLateralAccelerations / 2.0;

    double curr_left_bound_lane =
        curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);

    double curr_right_bound_lane =
        -curr_lane_right_width -
        (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;

    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc() ||
        is_fallback_lanechange) {
      // extend path bounds to include ADC in fallback or change lane path
      // bounds.
      double curr_left_bound_adc =
          std::fmax(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) +
          GetBufferBetweenADCCenterAndEdge() + ADC_buffer;
      curr_left_bound =
          std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;

      double curr_right_bound_adc =
          std::fmin(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) -
          GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
      curr_right_bound =
          std::fmin(curr_right_bound_lane, curr_right_bound_adc) -
          offset_to_map;
    } else {
      curr_left_bound = curr_left_bound_lane - offset_to_map;
      curr_right_bound = curr_right_bound_lane - offset_to_map;
    }

    ADEBUG << "At s = " << curr_s
           << ", left_lane_bound = " << curr_lane_left_width
           << ", right_lane_bound = " << curr_lane_right_width
           << ", offset = " << offset_to_map;

    // 4. Update the boundary.
    if (!UpdatePathBoundaryWithBuffer(i, curr_left_bound, curr_right_bound,
                                      path_bound, is_left_lane_boundary,
                                      is_right_lane_boundary)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_bound);

  if (lane_borrow_info == LaneBorrowInfo::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }

  return true;
}

// update boundaries with corresponding one-side lane boundary for pull over
// (1) use left lane boundary for normal PULL_OVER type
// (2) use left/right(which is opposite to pull over direction
//     (pull over at closer road side) lane boundary for EMERGENCY_PULL_OVER
void PathBoundsDecider::UpdatePullOverBoundaryByLaneBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  const auto pull_over_type = pull_over_status.pull_over_type();
  if (pull_over_type != PullOverStatus::PULL_OVER &&
      pull_over_type != PullOverStatus::EMERGENCY_PULL_OVER) {
    return;
  }

  for (size_t i = 0; i < path_bound->size(); ++i) {
    const double curr_s = std::get<0>((*path_bound)[i]);
    double left_bound = 3.0;
    double right_bound = 3.0;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      left_bound = curr_lane_left_width + offset_to_lane_center;
      right_bound = curr_lane_right_width + offset_to_lane_center;
    }
    ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
           << "]";
    if (pull_over_type == PullOverStatus::PULL_OVER) {
      std::get<2>((*path_bound)[i]) = left_bound;
    } else if (pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER) {
      // TODO(all): use left/right lane boundary accordingly
      std::get<2>((*path_bound)[i]) = left_bound;
    }
  }
}

void PathBoundsDecider::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    std::get<1>((*path_bound)[i]) -= refline_offset_to_lane_center;
    std::get<2>((*path_bound)[i]) -= refline_offset_to_lane_center;
  }
}

void PathBoundsDecider::GetBoundaryFromLaneChangeForbiddenZone(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // If there is a pre-determined lane-change starting position, then use it;
  // otherwise, decide one.
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  if (lane_change_status->is_clear_to_change_lane()) {
    ADEBUG << "Current position is clear to change lane. No need prep s.";
    lane_change_status->set_exist_lane_change_start_position(false);
    return;
  }
  double lane_change_start_s = 0.0;
  if (lane_change_status->exist_lane_change_start_position()) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_status->lane_change_start_position(),
                          &point_sl);
    lane_change_start_s = point_sl.s();
  } else {
    // TODO(jiacheng): train ML model to learn this.
    lane_change_start_s = FLAGS_lane_change_prepare_length + adc_frenet_s_;

    // Update the decided lane_change_start_s into planning-context.
    common::SLPoint lane_change_start_sl;
    lane_change_start_sl.set_s(lane_change_start_s);
    lane_change_start_sl.set_l(0.0);
    common::math::Vec2d lane_change_start_xy;
    reference_line.SLToXY(lane_change_start_sl, &lane_change_start_xy);
    lane_change_status->set_exist_lane_change_start_position(true);
    lane_change_status->mutable_lane_change_start_position()->set_x(
        lane_change_start_xy.x());
    lane_change_status->mutable_lane_change_start_position()->set_y(
        lane_change_start_xy.y());
  }

  // Remove the target lane out of the path-boundary, up to the decided S.
  if (lane_change_start_s < adc_frenet_s_) {
    // If already passed the decided S, then return.
    // lane_change_status->set_exist_lane_change_start_position(false);
    return;
  }
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    if (curr_s > lane_change_start_s) {
      break;
    }
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_map = 0.0;
    reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
    }
    curr_lane_left_width -= offset_to_map;
    curr_lane_right_width += offset_to_map;

    std::get<1>((*path_bound)[i]) =
        adc_frenet_l_ > curr_lane_left_width
            ? curr_lane_left_width + GetBufferBetweenADCCenterAndEdge()
            : std::get<1>((*path_bound)[i]);
    std::get<1>((*path_bound)[i]) =
        std::fmin(std::get<1>((*path_bound)[i]), adc_frenet_l_ - 0.1);
    std::get<2>((*path_bound)[i]) =
        adc_frenet_l_ < -curr_lane_right_width
            ? -curr_lane_right_width - GetBufferBetweenADCCenterAndEdge()
            : std::get<2>((*path_bound)[i]);
    std::get<2>((*path_bound)[i]) =
        std::fmax(std::get<2>((*path_bound)[i]), adc_frenet_l_ + 0.1);
  }
}

// Currently, it processes each obstacle based on its frenet-frame
// projection. Therefore, it might be overly conservative when processing
// obstacles whose headings differ from road-headings a lot.
// TODO(all): (future work) this can be improved in the future.
bool PathBoundsDecider::GetBoundaryFromStaticObstacles(
    const PathDecision& path_decision, PathBound* const path_boundaries,
    std::string* const blocking_obstacle_id) {
  // Preprocessing.
  auto indexed_obstacles = path_decision.obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles);
  ADEBUG << "There are " << sorted_obstacles.size() << " obstacles.";
  double center_line = adc_frenet_l_;
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle
  // is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        const double curr_obstacle_s = std::get<1>(curr_obstacle);
        const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        ADEBUG << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
               << "] curr_obstacle_l_min[" << curr_obstacle_l_min
               << "] curr_obstacle_l_max[" << curr_obstacle_l_max
               << "] center_line[" << center_line << "]";
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
          }
          if (!UpdatePathBoundaryAndCenterLineWithBuffer(
                  i, *left_bounds.begin(), *right_bounds.begin(),
                  path_boundaries, &center_line)) {
            path_blocked_idx = static_cast<int>(i);
            *blocking_obstacle_id = curr_obstacle_id;
            break;
          }
        } else {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }
        // Update the bounds and center_line.
        std::get<1>((*path_boundaries)[i]) = std::fmax(
            std::get<1>((*path_boundaries)[i]),
            *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
        std::get<2>((*path_boundaries)[i]) = std::fmin(
            std::get<2>((*path_boundaries)[i]),
            *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
        if (std::get<1>((*path_boundaries)[i]) >
            std::get<2>((*path_boundaries)[i])) {
          ADEBUG << "Path is blocked at s = " << curr_s;
          path_blocked_idx = static_cast<int>(i);
          if (!obs_id_to_direction.empty()) {
            *blocking_obstacle_id = obs_id_to_direction.begin()->first;
          }
          break;
        } else {
          center_line = (std::get<1>((*path_boundaries)[i]) +
                         std::get<2>((*path_boundaries)[i])) /
                        2.0;
        }

        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      std::get<1>((*path_boundaries)[i]) =
          std::fmax(std::get<1>((*path_boundaries)[i]),
                    *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
      std::get<2>((*path_boundaries)[i]) =
          std::fmin(std::get<2>((*path_boundaries)[i]),
                    *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
      if (std::get<1>((*path_boundaries)[i]) >
          std::get<2>((*path_boundaries)[i])) {
        ADEBUG << "Path is blocked at s = " << curr_s;
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
        }
      } else {
        center_line = (std::get<1>((*path_boundaries)[i]) +
                       std::get<2>((*path_boundaries)[i])) /
                      2.0;
      }
    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
std::vector<ObstacleEdge> PathBoundsDecider::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles) {
  std::vector<ObstacleEdge> sorted_obstacles;

  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on those within-scope obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
    if (obstacle->PerceptionSLBoundary().end_s() < adc_frenet_s_) {
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    sorted_obstacles.emplace_back(
        1, obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
    sorted_obstacles.emplace_back(
        0, obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->Id());
  }

  // Sort.
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

std::vector<PathBound> PathBoundsDecider::ConstructSubsequentPathBounds(
    const std::vector<ObstacleEdge>& sorted_obstacles, size_t path_idx,
    size_t obs_idx,
    std::unordered_map<std::string, std::tuple<bool, double>>* const
        obs_id_to_details,
    PathBound* const curr_path_bounds) {
  double left_bounds_from_obstacles = std::numeric_limits<double>::max();
  double right_bounds_from_obstacles = std::numeric_limits<double>::lowest();
  double curr_s = std::get<0>((*curr_path_bounds)[path_idx]);
  //==============================================================
  // If searched through all available s and found a path, return.
  if (path_idx >= curr_path_bounds->size()) {
    ADEBUG << "Completed path bounds search ending at path_idx = " << path_idx;
    return {*curr_path_bounds};
  }

  //==============================================================
  // If there is no obstacle updates at this path_idx.
  if (obs_idx >= sorted_obstacles.size() ||
      std::get<1>(sorted_obstacles[obs_idx]) > curr_s) {
    // 0. Backup the old memory.
    auto old_path_boundary = *curr_path_bounds;
    // 1. Get the boundary from obstacles.
    for (auto it = obs_id_to_details->begin(); it != obs_id_to_details->end();
         ++it) {
      if (std::get<0>(it->second)) {
        // Pass from left.
        right_bounds_from_obstacles =
            std::max(right_bounds_from_obstacles, std::get<1>(it->second));
      } else {
        // Pass from right.
        left_bounds_from_obstacles =
            std::min(left_bounds_from_obstacles, std::get<1>(it->second));
      }
    }
    // 2. Update the path boundary
    bool is_able_to_update = UpdatePathBoundaryWithBuffer(
        path_idx, left_bounds_from_obstacles, right_bounds_from_obstacles,
        curr_path_bounds);
    // 3. Return proper values.
    std::vector<PathBound> ret;
    if (is_able_to_update) {
      ret =
          ConstructSubsequentPathBounds(sorted_obstacles, path_idx + 1, obs_idx,
                                        obs_id_to_details, curr_path_bounds);
    } else {
      TrimPathBounds(static_cast<int>(path_idx), curr_path_bounds);
      ret.push_back(*curr_path_bounds);
    }
    *curr_path_bounds = old_path_boundary;
    return ret;
  }

  //==============================================================
  // If there are obstacle changes
  // 0. Backup the old memory.
  std::unordered_map<std::string, std::tuple<bool, double>>
      old_obs_id_to_details = *obs_id_to_details;
  auto old_path_boundary = *curr_path_bounds;

  // 1. Go through all obstacle changes.
  //    - For exiting obstacle, remove from our memory.
  //    - For entering obstacle, save it to a vector.
  std::vector<ObstacleEdge> new_entering_obstacles;
  size_t new_obs_idx = obs_idx;
  while (new_obs_idx < sorted_obstacles.size() &&
         std::get<1>(sorted_obstacles[new_obs_idx]) <= curr_s) {
    if (!std::get<0>(sorted_obstacles[new_obs_idx])) {
      // For exiting obstacle.
      obs_id_to_details->erase(std::get<4>(sorted_obstacles[new_obs_idx]));
    } else {
      // For entering obstacle.
      new_entering_obstacles.push_back(sorted_obstacles[new_obs_idx]);
    }
    ++new_obs_idx;
  }
  // 2. For new entering obstacles, decide possible pass directions.
  //    (ranked in terms of optimality)
  auto pass_direction_decisions =
      DecidePassDirections(0.0, 0.0, new_entering_obstacles);
  // 3. Try constructing subsequent path-bounds for all possible directions.
  std::vector<PathBound> ret;
  for (size_t i = 0; i < pass_direction_decisions.size(); ++i) {
    // For each possible direction:
    // a. Update the obs_id_to_details
    for (size_t j = 0; j < pass_direction_decisions[i].size(); ++j) {
      if (pass_direction_decisions[i][j]) {
        // Pass from left.
        (*obs_id_to_details)[std::get<4>(new_entering_obstacles[j])] =
            std::make_tuple(true, std::get<3>(new_entering_obstacles[j]));
      } else {
        // Pass from right.
        (*obs_id_to_details)[std::get<4>(new_entering_obstacles[j])] =
            std::make_tuple(false, std::get<2>(new_entering_obstacles[j]));
      }
    }
    // b. Figure out left/right bounds after the updates.
    for (auto it = obs_id_to_details->begin(); it != obs_id_to_details->end();
         ++it) {
      if (std::get<0>(it->second)) {
        // Pass from left.
        right_bounds_from_obstacles =
            std::max(right_bounds_from_obstacles, std::get<1>(it->second));
      } else {
        // Pass from right.
        left_bounds_from_obstacles =
            std::min(left_bounds_from_obstacles, std::get<1>(it->second));
      }
    }
    // c. Update for this path_idx, and construct the subsequent path bounds.
    std::vector<PathBound> curr_dir_path_boundaries;
    bool is_able_to_update = UpdatePathBoundaryWithBuffer(
        path_idx, left_bounds_from_obstacles, right_bounds_from_obstacles,
        curr_path_bounds);
    if (is_able_to_update) {
      curr_dir_path_boundaries = ConstructSubsequentPathBounds(
          sorted_obstacles, path_idx + 1, new_obs_idx, obs_id_to_details,
          curr_path_bounds);
    } else {
      TrimPathBounds(static_cast<int>(path_idx), curr_path_bounds);
      curr_dir_path_boundaries.push_back(*curr_path_bounds);
    }
    // d. Update the path_bounds into the vector, and revert changes
    //    to curr_path_bounds for next cycle.
    ret.insert(ret.end(), curr_dir_path_boundaries.begin(),
               curr_dir_path_boundaries.end());
    *curr_path_bounds = old_path_boundary;
  }
  // 4. Select the best path_bounds in ret.
  *obs_id_to_details = old_obs_id_to_details;
  *curr_path_bounds = old_path_boundary;
  std::sort(ret.begin(), ret.end(),
            [](const PathBound& lhs, const PathBound& rhs) {
              return lhs.size() > rhs.size();
            });
  while (ret.size() > 3) {
    ret.pop_back();
  }
  return ret;
}

std::vector<std::vector<bool>> PathBoundsDecider::DecidePassDirections(
    double l_min, double l_max,
    const std::vector<ObstacleEdge>& new_entering_obstacles) {
  std::vector<std::vector<bool>> decisions;

  // Convert into lateral edges.
  std::vector<ObstacleEdge> lateral_edges;
  lateral_edges.emplace_back(1, std::numeric_limits<double>::lowest(), 0.0, 0.0,
                             "l_min");
  lateral_edges.emplace_back(0, l_min, 0.0, 0.0, "l_min");
  lateral_edges.emplace_back(1, l_max, 0.0, 0.0, "l_max");
  lateral_edges.emplace_back(0, std::numeric_limits<double>::max(), 0.0, 0.0,
                             "l_max");
  for (size_t i = 0; i < new_entering_obstacles.size(); ++i) {
    if (std::get<3>(new_entering_obstacles[i]) < l_min ||
        std::get<2>(new_entering_obstacles[i]) > l_max) {
      continue;
    }
    lateral_edges.emplace_back(1, std::get<2>(new_entering_obstacles[i]), 0.0,
                               0.0, std::get<4>(new_entering_obstacles[i]));
    lateral_edges.emplace_back(0, std::get<3>(new_entering_obstacles[i]), 0.0,
                               0.0, std::get<4>(new_entering_obstacles[i]));
  }
  // Sort the lateral edges for lateral sweep-line algorithm.
  std::sort(lateral_edges.begin(), lateral_edges.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  // Go through the lateral edges and find any possible slot.
  std::vector<double> empty_slot;
  int num_obs = 0;
  for (size_t i = 0; i < lateral_edges.size(); ++i) {
    // Update obstacle overlapping info.
    if (std::get<0>(lateral_edges[i])) {
      ++num_obs;
    } else {
      --num_obs;
    }
    // If there is an empty slot within lane boundary.
    if (num_obs == 0 && i != lateral_edges.size() - 1) {
      empty_slot.push_back(
          (std::get<1>(lateral_edges[i]) + std::get<1>(lateral_edges[i + 1])) /
          2.0);
    }
  }
  // For each empty slot, update a corresponding pass direction
  for (size_t i = 0; i < empty_slot.size(); ++i) {
    double pass_position = empty_slot[i];
    std::vector<bool> pass_direction;
    for (size_t j = 0; j < new_entering_obstacles.size(); ++j) {
      if (std::get<2>(new_entering_obstacles[j]) > pass_position) {
        pass_direction.push_back(false);
      } else {
        pass_direction.push_back(true);
      }
    }
    decisions.push_back(pass_direction);
  }
  // TODO(jiacheng): sort the decisions based on the feasibility.

  return decisions;
}

double PathBoundsDecider::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many
  // factors such as: ADC length, possible turning angle, speed, etc.
  static constexpr double kAdcEdgeBuffer = 0.0;

  return (adc_half_width + kAdcEdgeBuffer);
}

bool PathBoundsDecider::UpdatePathBoundaryWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, bool is_left_lane_bound,
    bool is_right_lane_bound) {
  // substract vehicle width when bound does not come from the lane boundary
  const double default_adc_buffer_coeff = 1.0;
  double left_adc_buffer_coeff =
      (is_left_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : default_adc_buffer_coeff);
  double right_adc_buffer_coeff =
      (is_right_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : default_adc_buffer_coeff);

  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + right_adc_buffer_coeff *
                                  GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(
      std::get<2>((*path_boundaries)[idx]),
      left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

bool PathBoundsDecider::UpdatePathBoundaryAndCenterLineWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, double* const center_line) {
  UpdatePathBoundaryWithBuffer(idx, left_bound, right_bound, path_boundaries);
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
  return true;
}

bool PathBoundsDecider::UpdatePathBoundary(size_t idx, double left_bound,
                                           double right_bound,
                                           PathBound* const path_boundaries) {
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]), right_bound);
  // Update the left bound (l_max):
  double new_l_max =
      std::fmin(std::get<2>((*path_boundaries)[idx]), left_bound);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

void PathBoundsDecider::TrimPathBounds(const int path_blocked_idx,
                                       PathBound* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

void PathBoundsDecider::PathBoundsDebugString(
    const PathBound& path_boundaries) {
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    AWARN << "idx " << i << "; s = " << std::get<0>(path_boundaries[i])
          << "; l_min = " << std::get<1>(path_boundaries[i])
          << "; l_max = " << std::get<2>(path_boundaries[i]);
  }
}

bool PathBoundsDecider::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const LaneBorrowInfo& lane_borrow_info) {
  if (lane_borrow_info == LaneBorrowInfo::NO_BORROW) {
    return false;
  }

  const ReferenceLine& reference_line = reference_line_info.reference_line();
  auto ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }
  if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
      lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
    return false;
  }
  return true;
}

void PathBoundsDecider::RecordDebugInfo(
    const PathBound& path_boundaries, const std::string& debug_name,
    ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  ACHECK(!path_boundaries.empty());
  CHECK_NOTNULL(reference_line_info);

  // Take the left and right path boundaries, and transform them into two
  // PathData so that they can be displayed in simulator.
  std::vector<common::FrenetFramePoint> frenet_frame_left_boundaries;
  std::vector<common::FrenetFramePoint> frenet_frame_right_boundaries;
  for (const PathBoundPoint& path_bound_point : path_boundaries) {
    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(std::get<0>(path_bound_point));
    frenet_frame_point.set_dl(0.0);
    frenet_frame_point.set_ddl(0.0);

    frenet_frame_point.set_l(std::get<1>(path_bound_point));
    frenet_frame_right_boundaries.push_back(frenet_frame_point);
    frenet_frame_point.set_l(std::get<2>(path_bound_point));
    frenet_frame_left_boundaries.push_back(frenet_frame_point);
  }

  auto frenet_frame_left_path =
      FrenetFramePath(std::move(frenet_frame_left_boundaries));
  auto frenet_frame_right_path =
      FrenetFramePath(std::move(frenet_frame_right_boundaries));

  PathData left_path_data;
  left_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  left_path_data.SetFrenetPath(std::move(frenet_frame_left_path));
  PathData right_path_data;
  right_path_data.SetReferenceLine(&(reference_line_info->reference_line()));
  right_path_data.SetFrenetPath(std::move(frenet_frame_right_path));

  // Insert the transformed PathData into the simulator display.
  auto* ptr_display_path_1 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_1->set_name("planning_path_boundary_1");
  ptr_display_path_1->mutable_path_point()->CopyFrom(
      {left_path_data.discretized_path().begin(),
       left_path_data.discretized_path().end()});
  auto* ptr_display_path_2 =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  ptr_display_path_2->set_name("planning_path_boundary_2");
  ptr_display_path_2->mutable_path_point()->CopyFrom(
      {right_path_data.discretized_path().begin(),
       right_path_data.discretized_path().end()});
}

}  // namespace planning
}  // namespace apollo
