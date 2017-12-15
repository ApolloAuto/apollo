/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file adc_master_scenario.h
 **/

#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_ADC_MASTER_SCENARIO_H_
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_ADC_MASTER_SCENARIO_H_

#include "modules/planning/lattice/behavior_decider/scenario.h"

namespace apollo {
namespace planning {

class AdcMasterScenario : public Scenario {
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
      PlanningTarget* const decision);

 private:
  bool exist_ = false;

  DECLARE_SCENARIO(AdcMasterScenario);
};

}  // namespace planning
}  // namespace apollo

#endif
