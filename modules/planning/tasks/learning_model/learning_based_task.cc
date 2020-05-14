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

#include "modules/planning/tasks/learning_model/learning_based_task.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

LearningBasedTask::LearningBasedTask(const TaskConfig &config)
    : Task(config), device_(torch::kCPU) {
  ACHECK(config.has_learning_based_task_config());
}

Status LearningBasedTask::Execute(Frame *frame,
                                  ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(frame);
}

Status LearningBasedTask::Process(Frame *frame) {
  auto& config = config_.learning_based_task_config();
  const auto model_file = config.model_file();
  if (apollo::cyber::common::PathExists(model_file)) {
    try {
      model_ = torch::jit::load(model_file, device_);
    }
    catch (const c10::Error& e) {
      AERROR << "error loading the model:" << model_file;
      return Status(ErrorCode::PLANNING_ERROR,
                    "learning based task model file not exist");
    }
  }
  input_feature_num_ = config.input_feature_num();

  std::vector<torch::jit::IValue> input_features;
  ExtractFeatures(frame, &input_features);
  InferenceModel(input_features, frame);

  return Status::OK();
}

bool LearningBasedTask::ExtractFeatures(
    Frame* frame,
    std::vector<torch::jit::IValue> *input_features) {
  // TODO(all): generate learning features.
  // TODO(all): adapt to new input feature shapes
  std::vector<torch::jit::IValue> tuple;
  tuple.push_back(torch::zeros({2, 3, 224, 224}));
  tuple.push_back(torch::zeros({2, 14}));
  // assumption: future learning model use one dimension input features.
  input_features->push_back(torch::ivalue::Tuple::create(tuple));

  return true;
}

bool LearningBasedTask::InferenceModel(
    const std::vector<torch::jit::IValue> &input_features,
    Frame* frame) {
  auto reference_line_infos = frame->mutable_reference_line_infos();
  if (reference_line_infos->empty()) {
    AERROR << "no reference is found.";
    return false;
  }
  // FIXME(all): current only pick up the first reference line to use
  // learning model trajectory.
  for (auto& reference_line_info : *reference_line_infos) {
    reference_line_info.SetDrivable(false);
  }
  auto& picked_reference_line_info = reference_line_infos->front();
  picked_reference_line_info.SetDrivable(true);
  picked_reference_line_info.SetCost(0);

  auto torch_output = model_.forward(input_features);
  ADEBUG << torch_output;
  auto torch_output_tensor = torch_output.toTensor();
  auto output_shapes = torch_output_tensor.sizes();
  if (output_shapes.empty() || output_shapes.size() < 3) {
    AWARN << "invalid response from learning model.";
    return false;
  }

  // TODO(all): only populate path data from learning model
  // for initial version
  std::vector<TrajectoryPoint> trajectory_points;
  for (int i = 0; i < output_shapes[1]; ++i) {
    TrajectoryPoint p;
    p.mutable_path_point()->set_x(
       torch_output_tensor.accessor<float, 3>()[0][i][0]);
    p.mutable_path_point()->set_y(
       torch_output_tensor.accessor<float, 3>()[0][i][1]);
    trajectory_points.push_back(p);
  }
  picked_reference_line_info.SetTrajectory(
     DiscretizedTrajectory(trajectory_points));

  return true;
}

}  // namespace planning
}  // namespace apollo
