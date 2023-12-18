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
 **/

#include "modules/planning/traffic_rules/reference_line_end/reference_line_end.h"

#include <memory>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

bool ReferenceLineEnd::Init(
    const std::string& name,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<ReferenceLineEndConfig>(&config_);
}

Status ReferenceLineEnd::ApplyRule(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();

  ADEBUG << "ReferenceLineEnd length[" << reference_line.Length() << "]";
  for (const auto& segment : reference_line_info->Lanes()) {
    ADEBUG << "   lane[" << segment.lane->lane().id().id() << "]";
  }
  // check
  double remain_s =
      reference_line.Length() - reference_line_info->AdcSlBoundary().end_s();
  if (remain_s > config_.min_reference_line_remain_length()) {
    return Status::OK();
  }

  // create a virtual stop wall at the end of reference line to stop the adc
  std::string virtual_obstacle_id =
      REF_LINE_END_VO_ID_PREFIX + reference_line_info->Lanes().Id();
  double obstacle_start_s =
      reference_line.Length() - 2 * FLAGS_virtual_stop_wall_length;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, obstacle_start_s);
  if (!obstacle) {
    return Status(common::PLANNING_ERROR,
                  "Failed to create reference line end obstacle");
  }
  Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    return Status(
        common::PLANNING_ERROR,
        "Failed to create path obstacle for reference line end obstacle");
  }

  // build stop decision
  const double stop_line_s = obstacle_start_s - config_.stop_distance();
  auto stop_point = reference_line.GetReferencePoint(stop_line_s);

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);
  stop_decision->set_distance_s(-config_.stop_distance());
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(Getname(), stop_wall->Id(), stop);

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
