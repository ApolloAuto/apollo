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

#include "modules/planning/tasks/learning_model/learning_model_inference_task.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

LearningModelInferenceTask::LearningModelInferenceTask(
    const TaskConfig &config) : Task(config) {
  ACHECK(config.has_learning_model_inference_task_config());
}

Status LearningModelInferenceTask::Execute(
    Frame *frame,
    ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);

  Task::Execute(frame, reference_line_info);
  return Process(frame);
}

Status LearningModelInferenceTask::Process(Frame *frame) {
  CHECK_NOTNULL(frame);

  const auto& config = config_.learning_model_inference_task_config();
  const auto model_file = config.model_file();
  ADEBUG << model_file;

  // TODO(all): call Inference()

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
