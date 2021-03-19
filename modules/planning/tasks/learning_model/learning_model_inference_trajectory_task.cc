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

#include "modules/planning/tasks/learning_model/learning_model_inference_trajectory_task.h"

#include <string>
#include <vector>

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

LearningModelInferenceTrajectoryTask::LearningModelInferenceTrajectoryTask(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Task(config, injector) {
  ACHECK(config.has_learning_model_inference_trajectory_task_config());
}

Status LearningModelInferenceTrajectoryTask::Execute(
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  Task::Execute(frame, reference_line_info);
  return Process(frame, reference_line_info);
}

Status LearningModelInferenceTrajectoryTask::Process(
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const auto &config =
      config_.learning_model_inference_trajectory_task_config();

  const std::vector<TrajectoryPoint> adc_future_trajectory_points
      = injector_->learning_based_data()
                 ->learning_data_adc_future_trajectory_points();

  const double first_point_relative_time =
      adc_future_trajectory_points.front().relative_time();
  const double last_point_relative_time =
      adc_future_trajectory_points.back().relative_time();
  const auto learning_data_frame = injector_->learning_based_data()
                                            ->GetLatestLearningDataFrame();
  const int frame_num =
      learning_data_frame ? learning_data_frame->frame_num() : -1;
  ADEBUG << "LearningModelInferenceTrajectoryTask: frame_num[" << frame_num
         << "] adc_future_trajectory_points_size["
         << adc_future_trajectory_points.size()
         << "] first_point_relative_time[" << first_point_relative_time
         << "] last_point_relative_time[" << last_point_relative_time << "]";
  if (adc_future_trajectory_points.size() < 0 ||
      first_point_relative_time < 0.0 ||
      last_point_relative_time <
          config.min_adc_future_trajectory_time_length()) {
    const std::string msg =
        absl::StrCat("adc_future_trajectory_point issue. size[",
                     adc_future_trajectory_points.size(),
                     "] first_point_relative_time[", first_point_relative_time,
                     "] last_point_relative_time[", last_point_relative_time);
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  reference_line_info->SetTrajectory(
      DiscretizedTrajectory(adc_future_trajectory_points));

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
