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

#include "modules/planning/scenarios/learning_model/test_learning_model_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/learning_model/test_learning_model_stage.h"

namespace apollo {
namespace planning {
namespace scenario {

std::unique_ptr<Stage> TestLearningModelScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (stage_config.stage_type() != ScenarioConfig::TEST_LEARNING_MODEL_STAGE) {
    AERROR << "Test learning model scenario does not support stage type: "
           << ScenarioConfig::StageType_Name(stage_config.stage_type());
    return nullptr;
  }
  return std::unique_ptr<Stage>(new TestLearningModelStage(stage_config));
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
