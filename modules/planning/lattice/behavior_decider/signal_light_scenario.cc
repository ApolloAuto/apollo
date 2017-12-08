/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com.0, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file signal_light_scenario.cpp
 **/

#include "modules/planning/lattice/behavior_decider/signal_light_scenario.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/log.h"

#include "gflags/gflags.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/common/util/map_util.h"
#include "modules/planning/proto/planning_internal.pb.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;
using apollo::common::util::WithinBound;

void SignalLightScenario::Reset() {}

bool SignalLightScenario::Init() {
  exist_ = true;
  return exist_;
}

int SignalLightScenario::ComputeScenarioDecision(
    Frame* frame,
    ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const std::vector<common::PathPoint>& discretized_reference_line,
    std::vector<PlanningTarget>* const decisions) {
  CHECK(frame != nullptr);

  // Only handles one reference line
  CHECK_GT(discretized_reference_line.size(), 0);

  PlanningTarget ret;
  for (const auto& reference_point : discretized_reference_line) {
    ret.mutable_discretized_reference_line()
        ->add_discretized_reference_line_point()
        ->CopyFrom(reference_point);
  }

  //ret.set_decision_type(PlanningTarget::CRUISE);
  //ret.set_cruise_speed(FLAGS_default_cruise_speed);

  //decisions->emplace_back(std::move(ret));

  return 0;
}

bool SignalLightScenario::FindValidSignalLight(
    ReferenceLineInfo* const reference_line_info) {
  const std::vector<hdmap::PathOverlap>& signal_lights =
      reference_line_info->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() <= 0) {
    ADEBUG << "No signal lights from reference line.";
    return false;
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
    ADEBUG << "traffic signals msg is expired: "
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


}
}
