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
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/lattice_sampling_config.pb.h"

namespace apollo {
namespace planning {

class AdcMasterScenario : public Scenario {
public: //implement virtual functions from Scenario
    /**
     * return code:
     * 0 for existing head vehicle and -1 for NOT any head vehicle
     */
    virtual int ComputeScenarioDecision(
      const std::vector<common::PathPoint>& discretized_reference_line,
      std::vector<PlanningTarget*>* const decisions) override;

    virtual void Reset() override;

    virtual bool Init() override;

    virtual bool ScenarioExist() const override { return exist_; }

private:

private:
    bool exist_ = false;
    DECLARE_SCENARIO(AdcMasterScenario);
};

} // namespace planning
} // namespace apollo
#endif
