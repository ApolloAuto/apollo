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

/**
 * @file
 **/

#include "modules/planning/traffic_rules/traffic_light/traffic_light.h"

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool TrafficLight::Init(const std::string& name,
                        const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<TrafficLightConfig>(&config_);
}

Status TrafficLight::ApplyRule(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);

  return Status::OK();
}

void TrafficLight::MakeDecisions(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.enabled()) {
    return;
  }

  const auto& traffic_light_status =
      injector_->planning_context()->planning_status().traffic_light();

  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  // debug info
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();
  signal_light_debug->set_adc_front_s(adc_front_edge_s);
  signal_light_debug->set_adc_speed(
      injector_->vehicle_state()->linear_velocity());

  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info->reference_line().map_path().signal_overlaps();
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    if (traffic_light_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    // check if traffic-light-stop already finished, set by scenario/stage
    bool traffic_light_done = false;
    for (const auto& done_traffic_light_overlap_id :
         traffic_light_status.done_traffic_light_overlap_id()) {
      if (traffic_light_overlap.object_id == done_traffic_light_overlap_id) {
        traffic_light_done = true;
        break;
      }
    }
    if (traffic_light_done) {
      continue;
    }

    // work around incorrect s-projection along round routing
    static constexpr double kSDiscrepanceTolerance = 10.0;
    const auto& reference_line = reference_line_info->reference_line();
    common::SLPoint traffic_light_sl;
    traffic_light_sl.set_s(traffic_light_overlap.start_s);
    traffic_light_sl.set_l(0);
    common::math::Vec2d traffic_light_point;
    reference_line.SLToXY(traffic_light_sl, &traffic_light_point);
    common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                        injector_->vehicle_state()->y()};
    const double distance =
        common::util::DistanceXY(traffic_light_point, adc_position);
    const double s_distance = traffic_light_overlap.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] s_distance["
           << s_distance << "] actual_distance[" << distance << "]";
    if (s_distance >= 0 &&
        fabs(s_distance - distance) > kSDiscrepanceTolerance) {
      ADEBUG << "SKIP traffic_light[" << traffic_light_overlap.object_id
             << "] close in position, but far away along reference line";
      continue;
    }

    auto signal_color =
        frame->GetSignal(traffic_light_overlap.object_id).color();
    const double stop_deceleration = util::GetADCStopDeceleration(
        injector_->vehicle_state(), adc_front_edge_s,
        traffic_light_overlap.start_s);
    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "] stop_deceleration[" << stop_deceleration
           << "]";

    // debug info
    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    signal_debug->set_adc_stop_deceleration(stop_deceleration);
    signal_debug->set_color(signal_color);
    signal_debug->set_light_id(traffic_light_overlap.object_id);
    signal_debug->set_light_stop_s(traffic_light_overlap.start_s);

    // mayaochang add
    if (signal_color == perception::TrafficLight::GREEN ||
        signal_color == perception::TrafficLight::BLACK) {
      continue;
    }

    // Red/Yellow/Unknown: check deceleration
    if (stop_deceleration > config_.max_stop_deceleration()) {
      AWARN << "stop_deceleration too big to achieve.  SKIP red light";
      continue;
    }

    // build stop decision
    ADEBUG << "BuildStopDecision: traffic_light["
           << traffic_light_overlap.object_id << "] start_s["
           << traffic_light_overlap.start_s << "]"
           << "stop_distance: " << config_.stop_distance();
    std::string virtual_obstacle_id =
        TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_overlap.object_id;
    const std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(
        virtual_obstacle_id, traffic_light_overlap.start_s,
        config_.stop_distance(), StopReasonCode::STOP_REASON_SIGNAL,
        wait_for_obstacles, Getname(), frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo
