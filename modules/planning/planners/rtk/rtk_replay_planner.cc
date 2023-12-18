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

#include "modules/planning/planners/rtk/rtk_replay_planner.h"

#include <memory>
#include <utility>

#include "absl/strings/str_split.h"

#include "cyber/common/log.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

Status RTKReplayPlanner::Init(
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_path) {
  Planner::Init(injector, config_path);
  ReadTrajectoryFile(FLAGS_rtk_trajectory_filename);
  return Status::OK();
}

Status RTKReplayPlanner::Plan(const TrajectoryPoint& planning_start_point,
                              Frame* frame,
                              ADCTrajectory* ptr_computed_trajectory) {
  auto status = Status::OK();
  bool has_plan = false;
  auto it = std::find_if(
      frame->mutable_reference_line_info()->begin(),
      frame->mutable_reference_line_info()->end(),
      [](const ReferenceLineInfo& ref) { return ref.IsChangeLanePath(); });
  if (it != frame->mutable_reference_line_info()->end()) {
    status = PlanOnReferenceLine(planning_start_point, frame, &(*it));
    has_plan =
        (it->IsDrivable() && it->IsChangeLanePath() &&
         it->trajectory().GetSpatialLength() > FLAGS_change_lane_min_length);
    if (!has_plan) {
      AERROR << "Fail to plan for lane change.";
    }
  }

  if (!has_plan || !FLAGS_prioritize_change_lane) {
    for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
      if (reference_line_info.IsChangeLanePath()) {
        continue;
      }
      status = PlanOnReferenceLine(planning_start_point, frame,
                                   &reference_line_info);
      if (status != Status::OK()) {
        AERROR << "planner failed to make a driving plan for: "
               << reference_line_info.Lanes().Id();
      }
    }
  }
  return status;
}

Status RTKReplayPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame*,
    ReferenceLineInfo* reference_line_info) {
  if (complete_rtk_trajectory_.empty() || complete_rtk_trajectory_.size() < 2) {
    const std::string msg =
        "RTKReplayPlanner doesn't have a recorded trajectory or "
        "the recorded trajectory doesn't have enough valid trajectory "
        "points.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::uint32_t matched_index =
      QueryPositionMatchedPoint(planning_init_point, complete_rtk_trajectory_);

  std::uint32_t forward_buffer =
      static_cast<std::uint32_t>(FLAGS_rtk_trajectory_forward);
  // end_index is excluded.
  std::uint32_t end_index = std::min<std::uint32_t>(
      static_cast<std::uint32_t>(complete_rtk_trajectory_.size()),
      matched_index + forward_buffer);

  //  auto* trajectory_points = trajectory_pb->mutable_trajectory_point();
  std::vector<TrajectoryPoint> trajectory_points(
      complete_rtk_trajectory_.begin() + matched_index,
      complete_rtk_trajectory_.begin() + end_index);

  // reset relative time
  double zero_time = complete_rtk_trajectory_[matched_index].relative_time();
  for (auto& trajectory_point : trajectory_points) {
    trajectory_point.set_relative_time(trajectory_point.relative_time() -
                                       zero_time);
  }

  // check if the trajectory has enough points;
  // if not, append the last points multiple times and
  // adjust their corresponding time stamps.
  while (trajectory_points.size() <
         static_cast<size_t>(FLAGS_rtk_trajectory_forward)) {
    const auto& last_point = trajectory_points.rbegin();
    auto new_point = last_point;
    new_point->set_relative_time(new_point->relative_time() +
                                 FLAGS_rtk_trajectory_resolution);
    trajectory_points.push_back(*new_point);
  }
  reference_line_info->SetTrajectory(DiscretizedTrajectory(trajectory_points));
  return Status::OK();
}

void RTKReplayPlanner::ReadTrajectoryFile(const std::string& filename) {
  if (!complete_rtk_trajectory_.empty()) {
    complete_rtk_trajectory_.clear();
  }

  std::ifstream file_in(filename.c_str());
  if (!file_in.is_open()) {
    AERROR << "RTKReplayPlanner cannot open trajectory file: " << filename;
    return;
  }

  std::string line;
  // skip the header line.
  getline(file_in, line);

  while (true) {
    getline(file_in, line);
    if (line == "") {
      break;
    }

    const std::vector<std::string> tokens =
        absl::StrSplit(line, absl::ByAnyChar("\t "));
    if (tokens.size() < 11) {
      AERROR << "RTKReplayPlanner parse line failed; the data dimension does "
                "not match.";
      AERROR << line;
      continue;
    }

    TrajectoryPoint point;
    point.mutable_path_point()->set_x(std::stod(tokens[0]));
    point.mutable_path_point()->set_y(std::stod(tokens[1]));
    point.mutable_path_point()->set_z(std::stod(tokens[2]));

    point.set_v(std::stod(tokens[3]));
    point.set_a(std::stod(tokens[4]));

    point.mutable_path_point()->set_kappa(std::stod(tokens[5]));
    point.mutable_path_point()->set_dkappa(std::stod(tokens[6]));

    point.set_relative_time(std::stod(tokens[7]));

    point.mutable_path_point()->set_theta(std::stod(tokens[8]));

    point.mutable_path_point()->set_s(std::stod(tokens[10]));
    complete_rtk_trajectory_.push_back(std::move(point));
  }

  file_in.close();
}

std::uint32_t RTKReplayPlanner::QueryPositionMatchedPoint(
    const TrajectoryPoint& start_point,
    const std::vector<TrajectoryPoint>& trajectory) const {
  auto func_distance_square = [](const TrajectoryPoint& point, const double x,
                                 const double y) {
    double dx = point.path_point().x() - x;
    double dy = point.path_point().y() - y;
    return dx * dx + dy * dy;
  };
  double d_min =
      func_distance_square(trajectory.front(), start_point.path_point().x(),
                           start_point.path_point().y());
  std::uint32_t index_min = 0;
  for (std::uint32_t i = 1; i < trajectory.size(); ++i) {
    double d_temp =
        func_distance_square(trajectory[i], start_point.path_point().x(),
                             start_point.path_point().y());
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return index_min;
}

}  // namespace planning
}  // namespace apollo
