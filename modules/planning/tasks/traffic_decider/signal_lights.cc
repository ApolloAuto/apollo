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

#include <vector>
#include <limits>

#include "modules/planning/tasks/traffic_decider/signal_lights.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

SignalLights::SignalLights() : TrafficRule("SignalLights") {}

bool SignalLights::ApplyRule(Frame *frame,
                             ReferenceLineInfo *const reference_line_info) {
  if (!FLAGS_enable_signal_lights) {
    return true;
  }
  Init();
  if (!FindValidSignalLights(reference_line_info)) {
    return true;
  }
  ReadSignals();
  MakeDecisions(frame, reference_line_info);
  return true;
}

void SignalLights::Init() {
  signals_.clear();
  signal_lights_.clear();
}

void SignalLights::ReadSignals() {
  if (!AdapterManager::GetTrafficLightDetection()->Empty()) {
    return;
  }
  const TrafficLightDetection& detection =
      AdapterManager::GetTrafficLightDetection()->GetLatestObserved();
  for (int j = 0; j < detection.traffic_light_size(); j++) {
    const TrafficLight& signal = detection.traffic_light(j);
    signals_[signal.id()] = &signal;
  }
}

bool SignalLights::FindValidSignalLights(
    ReferenceLineInfo *const reference_line_info) {
  const std::vector<hdmap::PathOverlap> &signal_lights = reference_line_info
      ->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() <= 0) {
    return false;
  }
  for (const hdmap::PathOverlap &signal_light : signal_lights) {
    if (signal_light.start_s + FLAGS_max_distance_for_light_stop_buffer
        > reference_line_info->AdcSlBoundary().end_s()) {
      signal_lights_.push_back(&signal_light);
    }
  }
  return signal_lights_.size() > 0;
}

void SignalLights::MakeDecisions(Frame* frame,
                                 ReferenceLineInfo *const reference_line_info) {
  for (const hdmap::PathOverlap* signal_light : signal_lights_) {
    const TrafficLight signal = GetSignal(signal_light->object_id);
    double stop_deceleration = GetStopDeceleration(reference_line_info,
                                                   signal_light);
    if ((signal.color() == TrafficLight::RED &&
        stop_deceleration < 6) ||
        (signal.color() == TrafficLight::UNKNOWN &&
            stop_deceleration < 6) ||
        (signal.color() == TrafficLight::YELLOW &&
            stop_deceleration < 4)) {
      CreateStopObstacle(frame, reference_line_info, signal_light);
    }
  }
}

const TrafficLight SignalLights::GetSignal(const std::string &signal_id) {
  auto iter = signals_.find(signal_id);
  if (iter == signals_.end()) {
    TrafficLight traffic_light;
    traffic_light.set_id(signal_id);
    traffic_light.set_color(TrafficLight::UNKNOWN);
    traffic_light.set_confidence(0.0);
    traffic_light.set_tracking_time(0.0);
    return traffic_light;
  }
  return *iter->second;
}

double SignalLights::GetStopDeceleration(
    ReferenceLineInfo *const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  double adc_speed = common::VehicleState::instance()->linear_velocity();
  if (adc_speed < FLAGS_min_speed_for_light_stop) {
    return 0.0;
  }
  double stop_distance = 0;
  double adc_front_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_s = signal_light->start_s;

  if (stop_line_s > adc_front_s) {
    stop_distance = stop_line_s - adc_front_s;
  } else {
    stop_distance = stop_line_s +
        FLAGS_max_distance_for_light_stop_buffer - adc_front_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

void SignalLights::CreateStopObstacle(
    Frame* frame,
    ReferenceLineInfo *const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  common::SLPoint sl_point;
  sl_point.set_s(signal_light->start_s);
  sl_point.set_l(0);
  common::math::Vec2d vec2d;
  reference_line_info->reference_line().SLToXY(sl_point, &vec2d);
  double heading = reference_line_info->reference_line().
      GetReferencePoint(signal_light->start_s).heading();
  double left_lane_width;
  double right_lane_width;
  reference_line_info->reference_line().GetLaneWidth(
      signal_light->start_s, &left_lane_width, &right_lane_width);

  common::math::Box2d stop_box{{vec2d.x(), vec2d.y()},
                               heading,
                               FLAGS_virtual_stop_wall_length,
                               left_lane_width + right_lane_width};

  reference_line_info->AddObstacle(
      frame->AddStaticVirtualObstacle(
          FLAGS_signal_light_virtual_object_prefix + signal_light->object_id,
          stop_box));
}

}  // namespace planning
}  // namespace apollo
