/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com.0, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file adc_master_scenario.cpp
 **/

#include "modules/planning/lattice/behavior_decider/adc_master_scenario.h"

namespace apollo {
namespace planning {

void AdcMasterScenario::Reset() {
}

bool AdcMasterScenario::Init() {
  exist_ = true;
  return exist_;
}

int AdcMasterScenario::ComputeScenarioDecision(
  const std::vector<common::PathPoint>& discretized_reference_line, 
  std::vector<PlanningTarget*>* const decisions) {
    // TODO: implement the decision here with nearby obstacles
  return 0;
}

}
}
