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

#include "modules/planning/planning_base/common/util/common.h"

namespace apollo {
namespace planning {
namespace util {

using apollo::common::util::WithinBound;

/*
 * @brief: build virtual obstacle of stop wall, and add STOP decision
 */
int BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info,
                      double stop_wall_width) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), stop_line_s)) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id,
                                stop_line_s, stop_wall_width);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to add obstacle[" << stop_wall_id << "]";
    return -1;
  }

  // build stop decision
  const double stop_s = stop_line_s - stop_distance;
  const auto& stop_point = reference_line.GetReferencePoint(stop_s);
  const double stop_heading =
      reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const auto& reference_line = reference_line_info->reference_line();

  // create virtual stop wall
  const auto* obstacle =
      frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }

  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << stop_wall_id;
    return -1;
  }

  const auto& stop_wall_box = stop_wall->PerceptionBoundingBox();
  if (!reference_line.IsOnLane(stop_wall_box.center())) {
    ADEBUG << "stop point is not on lane. SKIP STOP decision";
    return 0;
  }

  // build stop decision
  auto stop_point = reference_line.GetReferencePoint(
      stop_wall->PerceptionSLBoundary().start_s() - stop_distance);

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
