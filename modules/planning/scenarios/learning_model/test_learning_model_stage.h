/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {

class TestLearningModelStage : public Stage {
 public:
  explicit TestLearningModelStage(const ScenarioConfig::StageConfig& config);

  StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  ScenarioConfig config_;
  std::unique_ptr<Stage> stage_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
