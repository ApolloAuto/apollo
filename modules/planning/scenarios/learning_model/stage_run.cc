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

#include "modules/planning/scenarios/learning_model/stage_run.h"

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::common::TrajectoryPoint;

Stage::StageStatus LearningModelSampleStageRun::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Run";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok =
      ExecuteTaskOnReferenceLineForOnlineLearning(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "LearningModelSampleStageRun planning error";
    return Stage::RUNNING;
  }

  return FinishStage();
}

Stage::StageStatus LearningModelSampleStageRun::FinishStage() {
  return FinishScenario();
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
