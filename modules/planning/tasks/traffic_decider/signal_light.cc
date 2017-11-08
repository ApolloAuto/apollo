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

#include "modules/planning/tasks/traffic_decider/signal_light.h"

#include <limits>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/map_util.h"
#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

SignalLight::SignalLight(const RuleConfig& config) : TrafficRule(config) {}

bool SignalLight::ApplyRule(Frame* frame,
                            ReferenceLineInfo* const reference_line_info) {
  if (!FindValidSignalLight(reference_line_info)) {
    return true;
  }
  ReadSignals();
  MakeDecisions(frame, reference_line_info);
  return true;
}

void SignalLight::ReadSignals() {
  if (AdapterManager::GetTrafficLightDetection()->Empty()) {
    return;
  }
  const TrafficLightDetection& detection =
      AdapterManager::GetTrafficLightDetection()->GetLatestObserved();
  for (int j = 0; j < detection.traffic_light_size(); j++) {
    const TrafficLight& signal = detection.traffic_light(j);
    signals_[signal.id()] = &signal;
  }
}

bool SignalLight::FindValidSignalLight(
    ReferenceLineInfo* const reference_line_info) {
  const std::vector<hdmap::PathOverlap>& signal_lights =
      reference_line_info->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() <= 0) {
    return false;
  }
  for (const hdmap::PathOverlap& signal_light : signal_lights) {
    if (signal_light.start_s + FLAGS_stop_max_distance_buffer >
        reference_line_info->AdcSlBoundary().end_s()) {
      signal_lights_.push_back(&signal_light);
    }
  }
  return signal_lights_.size() > 0;
}

void SignalLight::MakeDecisions(Frame* frame,
                                ReferenceLineInfo* const reference_line_info) {
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();
  signal_light_debug->set_adc_front_s(
      reference_line_info->AdcSlBoundary().end_s());
  signal_light_debug->set_adc_speed(
      common::VehicleStateProvider::instance()->linear_velocity());

  for (const hdmap::PathOverlap* signal_light : signal_lights_) {
    const TrafficLight signal = GetSignal(signal_light->object_id);
    double stop_deceleration =
        GetStopDeceleration(reference_line_info, signal_light);

    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    signal_debug->set_adc_stop_deacceleration(stop_deceleration);
    signal_debug->set_color(signal.color());
    signal_debug->set_light_id(signal_light->object_id);
    signal_debug->set_light_stop_s(signal_light->start_s);

    if ((signal.color() == TrafficLight::RED &&
         stop_deceleration < FLAGS_stop_max_deceleration) ||
        (signal.color() == TrafficLight::UNKNOWN &&
         stop_deceleration < FLAGS_stop_max_deceleration) ||
        (signal.color() == TrafficLight::YELLOW &&
         stop_deceleration < FLAGS_max_deacceleration_for_yellow_light_stop)) {
      CreateStopObstacle(frame, reference_line_info, signal_light);
      signal_debug->set_is_stop_wall_created(true);
    }
  }
}

TrafficLight SignalLight::GetSignal(const std::string& signal_id) {
  const auto* result = apollo::common::util::FindPtrOrNull(signals_, signal_id);
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

double SignalLight::GetStopDeceleration(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  double adc_speed =
      common::VehicleStateProvider::instance()->linear_velocity();
  if (adc_speed < FLAGS_stop_min_speed) {
    return 0.0;
  }
  double stop_distance = 0;
  double adc_front_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_s = signal_light->start_s;

  if (stop_line_s > adc_front_s) {
    stop_distance = stop_line_s - adc_front_s;
  } else {
    stop_distance = stop_line_s + FLAGS_stop_max_distance_buffer - adc_front_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

void SignalLight::CreateStopObstacle(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  common::SLPoint sl_point;
  sl_point.set_s(signal_light->start_s);
  sl_point.set_l(0);
  common::math::Vec2d vec2d;
  const auto& reference_line = reference_line_info->reference_line();
  reference_line.SLToXY(sl_point, &vec2d);

  double heading =
      reference_line.GetReferencePoint(signal_light->start_s).heading();
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  reference_line.GetLaneWidth(signal_light->start_s, &left_lane_width,
                              &right_lane_width);

  common::math::Box2d stop_box{{vec2d.x(), vec2d.y()},
                               heading,
                               FLAGS_virtual_stop_wall_length,
                               left_lane_width + right_lane_width};

  PathObstacle* stop_wall =
      reference_line_info->AddObstacle(frame->AddStaticVirtualObstacle(
          FLAGS_signal_light_virtual_object_id_prefix + signal_light->object_id,
          stop_box));
  auto* path_decision = reference_line_info->path_decision();
  ObjectDecisionType stop;
  stop.mutable_stop();
  path_decision->AddLongitudinalDecision(
      RuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);
}

}  // namespace planning
}  // namespace apollo
