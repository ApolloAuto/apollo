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

#include "modules/planning/toolkits/deciders/creep.h"

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool Creep::Init(const ScenarioConfig::ScenarioTaskConfig &config) {
  CHECK(config.has_decider_creep_config());
  config_ = config.decider_creep_config();
  is_init_ = true;
  return true;
}

Status Creep::Process(Frame* frame,
                      ReferenceLineInfo* reference_line_info) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }

  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  BuildStopDecision(frame, reference_line_info);

  return Status::OK();
}



double FindCreepDistance(Frame* frame,
                         ReferenceLineInfo* reference_line_info) {
  // TODO(all)
  return 0.5;
}

// TODO(all): revisit & rewrite
// bool Creep::BuildStopDecision(Frame* frame,
//                              ReferenceLineInfo* reference_line_info,
//                              const PathOverlap& overlap) {
bool Creep::BuildStopDecision(
    Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double creep_distance = FindCreepDistance(frame, reference_line_info);
  double creep_stop_s = adc_front_edge_s + creep_distance;

  // create virtual stop wall
  // TODO(all)
  // std::string virtual_obstacle_id = CREEP_VO_ID_PREFIX + overlap.object_id;
  std::string virtual_obstacle_id = CREEP_VO_ID_PREFIX + std::string("test");
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, creep_stop_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return false;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
    return false;
  }

  // build stop decision
  const double stop_distance = config_.stop_distance();
  const double stop_s = creep_stop_s - stop_distance;
  const auto& reference_line = reference_line_info->reference_line();
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_CREEPER);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision("Creeper", stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
