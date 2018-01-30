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

#include "modules/planning/lattice/behavior_decider/signal_light_scenario.h"

#include <limits>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_internal.pb.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::common::util::WithinBound;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

void SignalLightScenario::Reset() {}

bool SignalLightScenario::Init() {
  exist_ = true;
  return exist_;
}

int SignalLightScenario::ComputeScenarioDecision(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    PlanningTarget* planning_target) {
  CHECK(frame != nullptr);

  if (!FindValidSignalLight(reference_line_info)) {
    ADEBUG << "No valid signal light along reference line";
    return 0;
  }
  ReadSignals();

  for (const hdmap::PathOverlap* signal_light :
       signal_lights_along_reference_line_) {
    const TrafficLight signal = GetSignal(signal_light->object_id);
    double stop_deceleration =
        GetStopDeceleration(reference_line_info, signal_light);

    if ((signal.color() == TrafficLight::RED &&
         stop_deceleration < FLAGS_stop_max_deceleration) ||
        (signal.color() == TrafficLight::UNKNOWN &&
         stop_deceleration < FLAGS_stop_max_deceleration) ||
        (signal.color() == TrafficLight::YELLOW &&
         stop_deceleration < FLAGS_max_deacceleration_for_yellow_light_stop)) {
      CreateStopObstacle(frame, reference_line_info, signal_light);
    }
  }

  return 0;
}

bool SignalLightScenario::FindValidSignalLight(
    ReferenceLineInfo* const reference_line_info) {
  const std::vector<hdmap::PathOverlap>& signal_lights =
      reference_line_info->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() <= 0) {
    ADEBUG << "No signal lights from reference line.";
    return false;
  } else {
    ADEBUG << "Found signal_lights size=" << signal_lights.size();
  }
  signal_lights_along_reference_line_.clear();
  for (const hdmap::PathOverlap& signal_light : signal_lights) {
    if (signal_light.start_s + FLAGS_stop_max_distance_buffer >
        reference_line_info->AdcSlBoundary().end_s()) {
      signal_lights_along_reference_line_.push_back(&signal_light);
    }
  }
  return signal_lights_along_reference_line_.size() > 0;
}

void SignalLightScenario::ReadSignals() {
  detected_signals_.clear();
  detected_signals_.clear();
  if (AdapterManager::GetTrafficLightDetection()->Empty()) {
    return;
  }
  if (AdapterManager::GetTrafficLightDetection()->GetDelaySec() >
      FLAGS_signal_expire_time_sec) {
    AWARN << "traffic signals msg is expired: "
          << AdapterManager::GetTrafficLightDetection()->GetDelaySec();
    return;
  }
  const TrafficLightDetection& detection =
      AdapterManager::GetTrafficLightDetection()->GetLatestObserved();
  for (int j = 0; j < detection.traffic_light_size(); j++) {
    const TrafficLight& signal = detection.traffic_light(j);
    detected_signals_[signal.id()] = &signal;
  }
}

TrafficLight SignalLightScenario::GetSignal(const std::string& signal_id) {
  const auto* result =
      apollo::common::util::FindPtrOrNull(detected_signals_, signal_id);
  if (result == nullptr) {
    TrafficLight traffic_light;
    traffic_light.set_id(signal_id);
    traffic_light.set_color(TrafficLight::UNKNOWN);
    traffic_light.set_confidence(0.0);
    traffic_light.set_tracking_time(0.0);
    return traffic_light;
  }
  return *result;
}

double SignalLightScenario::GetStopDeceleration(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  double adc_speed =
      common::VehicleStateProvider::instance()->linear_velocity();
  if (adc_speed < FLAGS_stop_max_speed) {
    return 0.0;
  }
  double stop_distance = 0.0;
  double adc_front_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_s = signal_light->start_s;

  if (stop_line_s > adc_front_s) {
    stop_distance = stop_line_s - adc_front_s;
  } else {
    stop_distance = stop_line_s + FLAGS_stop_max_distance_buffer - adc_front_s;
  }
  if (stop_distance < 1e-5) {
    // longitudinal_acceleration_lower_bound is a negative value.
    return -FLAGS_longitudinal_acceleration_lower_bound;
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

void SignalLightScenario::CreateStopObstacle(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  const auto& reference_line = reference_line_info->reference_line();
  const double stop_s =
      signal_light->start_s - FLAGS_stop_distance_traffic_light;
  const double box_center_s =
      signal_light->start_s + FLAGS_virtual_stop_wall_length / 2.0;
  if (!WithinBound(0.0, reference_line.Length(), stop_s) ||
      !WithinBound(0.0, reference_line.Length(), box_center_s)) {
    ADEBUG << "signal " << signal_light->object_id
           << " is not on reference line";
    return;
  }
  double heading = reference_line.GetReferencePoint(stop_s).heading();
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  reference_line.GetLaneWidth(signal_light->start_s, &left_lane_width,
                              &right_lane_width);

  auto box_center = reference_line.GetReferencePoint(box_center_s);
  common::math::Box2d stop_box{box_center, heading,
                               FLAGS_virtual_stop_wall_length,
                               left_lane_width + right_lane_width};
  reference_line_info->AddObstacle(frame->AddStaticVirtualObstacle(
      FLAGS_signal_light_virtual_object_id_prefix + signal_light->object_id,
      stop_box));
}

}  // namespace planning
}  // namespace apollo
