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

#include "rtk_replay_planner.h"

#include <fstream>

#include "modules/common/log.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::vehicle_state::VehicleState;

RTKReplayPlanner::RTKReplayPlanner() {
  ReadTrajectoryFile(FLAGS_rtk_trajectory_filename);
}

bool RTKReplayPlanner::Plan(
    const TrajectoryPoint& start_point,
    std::vector<TrajectoryPoint>* ptr_discretized_trajectory) {

  if (complete_rtk_trajectory_.empty() || complete_rtk_trajectory_.size() < 2) {
    AERROR << "RTKReplayPlanner doesn't have a recorded trajectory or "
              "the recorded trajectory doesn't have enough valid trajectory "
              "points.";
    return false;
  }

  std::size_t matched_index =
      QueryPositionMatchedPoint(start_point, complete_rtk_trajectory_);

  std::size_t forward_buffer = FLAGS_rtk_trajectory_forward;
  std::size_t end_index =
      matched_index + forward_buffer >= complete_rtk_trajectory_.size()
          ? complete_rtk_trajectory_.size() - 1
          : matched_index + forward_buffer - 1;

  if (ptr_discretized_trajectory->size() > 0) {
    ptr_discretized_trajectory->clear();
  }

  ptr_discretized_trajectory->insert(
      ptr_discretized_trajectory->begin(),
      complete_rtk_trajectory_.begin() + matched_index,
      complete_rtk_trajectory_.begin() + end_index + 1);

  // reset relative time
  double zero_time = complete_rtk_trajectory_[matched_index].relative_time;;
  for (std::size_t i = 0; i < ptr_discretized_trajectory->size(); ++i) {
    (*ptr_discretized_trajectory)[i].relative_time -= zero_time;
  }

  // check if the trajectory has enough points;
  // if not, append the last points multiple times and
  // adjust their corresponding time stamps.
  while (ptr_discretized_trajectory->size() < FLAGS_rtk_trajectory_forward) {
    const auto& last_point = ptr_discretized_trajectory->back();
    ptr_discretized_trajectory->push_back(last_point);
    ptr_discretized_trajectory->back().relative_time += FLAGS_trajectory_resolution;
  }
  return true;
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

    auto tokens = apollo::common::util::StringTokenizer::Split(line, "\t ");
    if (tokens.size() < 11) {
      AERROR << "RTKReplayPlanner parse line failed; the data dimension does not match.";
      AERROR << line;
      continue;
    }

    TrajectoryPoint point;
    point.x = std::stod(tokens[0]);
    point.y = std::stod(tokens[1]);
    point.z = std::stod(tokens[2]);

    point.v = std::stod(tokens[3]);
    point.a = std::stod(tokens[4]);

    point.kappa = std::stod(tokens[5]);
    point.dkappa = std::stod(tokens[6]);

    point.relative_time = std::stod(tokens[7]);

    point.theta = std::stod(tokens[8]);

    point.s = std::stod(tokens[10]);
    complete_rtk_trajectory_.push_back(point);
  }

  file_in.close();
}

std::size_t RTKReplayPlanner::QueryPositionMatchedPoint(
    const TrajectoryPoint& start_point,
    const std::vector<TrajectoryPoint>& trajectory) const {
  auto func_distance_square = [](const PathPoint& point, const double x,
                                 const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
  };
  double d_min = func_distance_square(trajectory.front(), start_point.x,
                                      start_point.y);
  std::size_t index_min = 0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    double d_temp = func_distance_square(trajectory[i], start_point.x,
                                         start_point.y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return index_min;
}

}  // namespace planning
}  // nameapace apollo
