/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/traffic_decider/pull_over.h"

#include <iomanip>
#include <limits>
#include <vector>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/map_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/tasks/traffic_decider/util.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::Status;
using apollo::common::util::WithinBound;
using apollo::planning::util::GetPlanningStatus;

PullOver::PullOver(const TrafficRuleConfig& config) : TrafficRule(config) {}

Status PullOver::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  if (!IsPullOver()) {
    return Status::OK();
  }

  PointENU stop_point;
  double stop_heading = 0.0;
  if (!GetPullOverStop(&stop_point, &stop_heading)) {
    ADEBUG << "Could not find a safe pull over point";
    return Status::OK();
  }
  if (!BuildPullOverStop(stop_point, stop_heading)) {
    AWARN << "Not able to create a pull over stop";
    return Status::OK();
  }
  return Status::OK();
}

bool PullOver::IsPullOver() const {
  auto* planning_state = GetPlanningStatus()->mutable_planning_state();
  return (planning_state->has_pull_over() &&
      planning_state->pull_over().in_pull_over());
}

bool PullOver::IsValidStop(const PointENU& stop_point,
                           double stop_heading) const {
  // TODO(all) implement this function
  return true;
}

bool PullOver::GetPullOverStop(PointENU* stop_point, double* stop_heading) {
  auto* pull_over_status = GetPlanningStatus()->
      mutable_planning_state()->mutable_pull_over();
  // reuse existing stop point
  if (pull_over_status->has_stop_point() &&
      pull_over_status->has_stop_heading()) {
    if (IsValidStop(pull_over_status->stop_point(),
                    pull_over_status->stop_heading())) {
      *stop_point = pull_over_status->stop_point();
      *stop_heading = pull_over_status->stop_heading();
      return true;
    }
  }
  // calculate new stop point if don't have a pull over stop
  return SearchPullOverStop(stop_point, stop_heading);
}

bool PullOver::SearchPullOverStop(PointENU* stop_point, double* stop_heading) {
  // TODO(all) implement this function
  return false;
}

bool PullOver::BuildPullOverStop(const PointENU& stop_point,
                                 double stop_heading) {
  // check
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint sl;
  reference_line.XYToSL(stop_point, &sl);
  if (sl.s() < 0 || sl.s() > reference_line.Length()) {
    return false;
  }

  // create virtual stop wall
  std::stringstream ss;
  ss << PULL_OVER_VO_ID_PREFIX << std::setprecision(5) << stop_point.x() << ", "
     << stop_point.y();
  const std::string virtual_obstacle_id = ss.str();
  auto* obstacle = frame_->CreateStopObstacle(reference_line_info_,
                                              virtual_obstacle_id, sl.s());
  if (!obstacle) {
    AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
    return false;
  }
  PathObstacle* stop_wall = reference_line_info_->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for " << virtual_obstacle_id;
    return false;
  }

  // build stop decision
  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_PULL_OVER);
  stop_decision->set_distance_s(-config_.pull_over().stop_distance());
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info_->path_decision();
  if (!path_decision->MergeWithMainStop(
          stop.stop(), stop_wall->Id(), reference_line,
          reference_line_info_->AdcSlBoundary())) {
    ADEBUG << "signal " << virtual_obstacle_id << " is not the closest stop.";
    return false;
  }

  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
