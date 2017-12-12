/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file signal_light_scenario.h
 **/

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_SIGNAL_LIGHT_SCENARIO_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_SIGNAL_LIGHT_SCENARIO_H_

#include "modules/planning/lattice/behavior_decider/scenario.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

namespace apollo {
namespace planning {

class SignalLightScenario : public Scenario {
 public:
  virtual void Reset() override;

  virtual bool Init() override;

  virtual bool ScenarioExist() const override { return exist_; }

  virtual int ComputeScenarioDecision(
      Frame* frame,
      ReferenceLineInfo* const reference_line_info,
      const common::TrajectoryPoint& init_planning_point,
      const std::array<double, 3>& lon_init_state,
      const std::vector<common::PathPoint>& discretized_reference_line,
      std::vector<PlanningTarget>* const decisions);

 private:
  bool exist_ = false;

  void ReadSignals();
  bool FindValidSignalLight(ReferenceLineInfo* const reference_line_info);

  apollo::perception::TrafficLight GetSignal(const std::string& signal_id);
  double GetStopDeceleration(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light);
  void CreateStopObstacle(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light);

  std::vector<const hdmap::PathOverlap*> signal_lights_along_reference_line_;
  std::unordered_map<std::string, const apollo::perception::TrafficLight*>
      detected_signals_;

  DECLARE_SCENARIO(SignalLightScenario);
};

}  // namespace planning
}  // namespace apollo

#endif
