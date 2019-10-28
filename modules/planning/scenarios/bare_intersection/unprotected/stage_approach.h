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

#pragma once

#include <string>

#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace bare_intersection {

struct BareIntersectionUnprotectedContext;

class BareIntersectionUnprotectedStageApproach : public Stage {
 public:
  explicit BareIntersectionUnprotectedStageApproach(
      const ScenarioConfig::StageConfig& config)
      : Stage(config) {}

 private:
  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;
  BareIntersectionUnprotectedContext* GetContext() {
    return GetContextAs<BareIntersectionUnprotectedContext>();
  }

 private:
  Stage::StageStatus FinishStage(Frame* frame);

 private:
  ScenarioBareIntersectionUnprotectedConfig scenario_config_;
  static uint32_t clear_counter_;
};

}  // namespace bare_intersection
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
