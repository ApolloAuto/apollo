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

#include <string>

#include "absl/strings/str_cat.h"

#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/learning_based/model_inference/trajectory_imitation_inference.h"

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

  LearningDataFrame learning_data_frame;
  learning_data_frame.CopyFrom(frame->learning_data_frame());

  ADEBUG << "LearningModelInferenceTask: frame_num["
         << learning_data_frame.frame_num()
         << "] adc_trajectory_point_size["
         << learning_data_frame.adc_trajectory_point_size() << "]";

  if (learning_data_frame.adc_trajectory_point_size() <= 0) {
    const std::string msg = "learning_data adc_trajectory_point empty";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  TrajectoryConvRnnInference trajectory_conv_rnn_inference(config);
  if (!trajectory_conv_rnn_inference.Inference(&learning_data_frame)) {
    const std::string msg = "TrajectoryConvRnnInference Inference failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const int adc_future_trajectory_point_size =
      learning_data_frame.output().adc_future_trajectory_point_size();
  ADEBUG << "   adc_future_trajectory_point_size["
         << adc_future_trajectory_point_size << "]";
  if (adc_future_trajectory_point_size <= 0) {
    const std::string msg = "adc_future_trajectory_point empty";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<common::TrajectoryPoint> trajectory_points;
  ConvertTrajectory(learning_data_frame.output(), &trajectory_points);
  frame->set_learning_data_adc_future_trajectory_points(trajectory_points);

  return Status::OK();
}

void LearningModelInferenceTask::ConvertTrajectory(
    const LearningOutput& learning_out_put,
    std::vector<common::TrajectoryPoint>* trajectory_points) {
  for (int i = 0; i < learning_out_put.adc_future_trajectory_point_size();
      ++i) {
    // CommonTrajectoryPointFeature => common::TrajectoryPoint
    apollo::common::TrajectoryPoint trajectory_point;
    CommonTrajectoryPointFeature tpf
        = learning_out_put.adc_future_trajectory_point(i).trajectory_point();
    auto path_point = trajectory_point.mutable_path_point();
    path_point->set_x(tpf.path_point().x());
    path_point->set_y(tpf.path_point().y());
    path_point->set_z(tpf.path_point().z());
    path_point->set_theta(tpf.path_point().theta());
    // path_point->set_kappa();
    path_point->set_s(tpf.path_point().s());
    // path_point->set_dkappa();
    // path_point->set_ddkappa();
    path_point->set_lane_id(tpf.path_point().lane_id());
    // path_point->set_x_derivative();
    // path_point->set_y_derivative();

    trajectory_point.set_v(tpf.v());
    trajectory_point.set_a(tpf.a());
    trajectory_point.set_relative_time(tpf.relative_time());
    // trajectory_point.set_da();
    // trajectory_point.set_steer();
    trajectory_point.mutable_gaussian_info()->CopyFrom(tpf.gaussian_info());

    trajectory_points->push_back(trajectory_point);
  }
}

}  // namespace planning
}  // namespace apollo
