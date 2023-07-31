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

#include "modules/planning/planning_base/learning_based/model_inference/trajectory_imitation_libtorch_inference.h"

#include <string>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/planning_base/common/util/math_util.h"
#include "modules/planning/planning_base/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"

namespace apollo {
namespace planning {
TrajectoryImitationLibtorchInference::TrajectoryImitationLibtorchInference(
    const LearningModelInferenceTaskConfig& config)
    : ModelInference(config), device_(torch::kCPU) {}

bool TrajectoryImitationLibtorchInference::LoadCNNModel() {
  // run a fake inference at init time as first inference is relative slow
  torch::Tensor input_feature_tensor = torch::zeros({1, 12, 200, 200});
  std::vector<torch::jit::IValue> torch_inputs;
  torch_inputs.push_back(input_feature_tensor.to(device_));
  try {
    auto torch_output_tensor =
        model_.forward(torch_inputs).toTensor().to(torch::kCPU);
  } catch (const c10::Error& e) {
    AERROR << "Fail to do initial inference on CNN Model";
    return false;
  }
  return true;
}

bool TrajectoryImitationLibtorchInference::LoadCNNLSTMModel() {
  // run a fake inference at init time as first inference is relative slow
  torch::Tensor input_feature_tensor = torch::zeros({1, 12, 200, 200});
  torch::Tensor current_v = torch::zeros({1, 1});
  std::vector<torch::jit::IValue> torch_inputs;
  torch_inputs.push_back(
      c10::ivalue::Tuple::create({std::move(input_feature_tensor.to(device_)),
                                  std::move(current_v.to(device_))}));
  try {
    auto torch_output_tensor =
        model_.forward(torch_inputs).toTensor().to(torch::kCPU);
  } catch (const c10::Error& e) {
    AERROR << "Fail to do initial inference on HISTORY_UNCONDITIONED_CNN_LSTM "
              "Model";
    return false;
  }
  return true;
}

bool TrajectoryImitationLibtorchInference::LoadModel() {
  if (config_.use_cuda() && torch::cuda::is_available()) {
    ADEBUG << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
    try {
      model_ = torch::jit::load(config_.gpu_model_file(), device_);
    } catch (const c10::Error& e) {
      AERROR << "Failed to load model on to device";
      return false;
    }
  } else {
    model_ = torch::jit::load(config_.cpu_model_file(), device_);
  }

  torch::set_num_threads(1);
  switch (config_.model_type()) {
    case LearningModelInferenceTaskConfig::CNN: {
      if (!LoadCNNModel()) {
        return false;
      }
      break;
    }
    case LearningModelInferenceTaskConfig::CNN_LSTM: {
      if (!LoadCNNLSTMModel()) {
        return false;
      }
      break;
    }
    default: {
      AERROR << "Configured model type not defined and implemented";
      break;
    }
  }
  return true;
}

void TrajectoryImitationLibtorchInference::output_postprocessing(
    const at::Tensor& torch_output_tensor,
    LearningDataFrame* const learning_data_frame) {
  const int past_points_size = learning_data_frame->adc_trajectory_point_size();
  const auto& cur_traj_point =
      learning_data_frame->adc_trajectory_point(past_points_size - 1);
  const double cur_time_sec = cur_traj_point.timestamp_sec();
  const auto& cur_path_point = cur_traj_point.trajectory_point().path_point();
  const double cur_x = cur_path_point.x();
  const double cur_y = cur_path_point.y();
  const double cur_heading = cur_path_point.theta();
  ADEBUG << "cur_x[" << cur_x << "], cur_y[" << cur_y << "], cur_heading["
         << cur_heading << "], cur_v[" << cur_traj_point.trajectory_point().v()
         << "]";

  learning_data_frame->mutable_output()->clear_adc_future_trajectory_point();
  const double delta_t = config_.trajectory_delta_t();
  auto torch_output = torch_output_tensor.accessor<float, 3>();
  for (int i = 0; i < torch_output_tensor.size(1); ++i) {
    const double dx = static_cast<double>(torch_output[0][i][0]);
    const double dy = static_cast<double>(torch_output[0][i][1]);
    const double dtheta = static_cast<double>(torch_output[0][i][2]);
    const double v = static_cast<double>(torch_output[0][i][3]);
    ADEBUG << "dx[" << dx << "], dy[" << dy << "], dtheta[" << dtheta << "], v["
           << v << "]";
    const double time_sec = cur_time_sec + delta_t * (i + 1);
    apollo::common::math::Vec2d offset(dx, dy);
    apollo::common::math::Vec2d rotated_offset = offset.rotate(cur_heading);
    const double x = cur_x + rotated_offset.x();
    const double y = cur_y + rotated_offset.y();
    const double heading =
        apollo::common::math::NormalizeAngle(dtheta + cur_heading);

    auto* traj_point = learning_data_frame->mutable_output()
                           ->add_adc_future_trajectory_point();
    traj_point->set_timestamp_sec(time_sec);
    traj_point->mutable_trajectory_point()->mutable_path_point()->set_x(x);
    traj_point->mutable_trajectory_point()->mutable_path_point()->set_y(y);
    traj_point->mutable_trajectory_point()->mutable_path_point()->set_theta(
        heading);
    traj_point->mutable_trajectory_point()->set_v(v);
    traj_point->mutable_trajectory_point()->set_relative_time(delta_t *
                                                              (i + 1));
  }
}

bool TrajectoryImitationLibtorchInference::DoCNNMODELInference(
    LearningDataFrame* const learning_data_frame) {
  const int past_points_size = learning_data_frame->adc_trajectory_point_size();
  if (past_points_size == 0) {
    AERROR << "No current trajectory point status";
    return false;
  }

  auto input_renderering_start_time = std::chrono::system_clock::now();

  cv::Mat input_feature;
  if (!BirdviewImgFeatureRenderer::Instance()->RenderMultiChannelEnv(
          *learning_data_frame, &input_feature)) {
    AERROR << "Render multi-channel input image failed";
    return false;
  }

  auto input_renderering_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> rendering_diff =
      input_renderering_end_time - input_renderering_start_time;
  ADEBUG << "trajectory imitation model input renderer used time: "
         << rendering_diff.count() * 1000 << " ms.";

  auto input_preprocessing_start_time = std::chrono::system_clock::now();

  cv::Mat input_feature_float;
  input_feature.convertTo(input_feature_float, CV_32F, 1.0 / 255);
  torch::Tensor input_feature_tensor =
      torch::from_blob(input_feature_float.data,
                       {1, input_feature_float.rows, input_feature_float.cols,
                        input_feature_float.channels()});
  input_feature_tensor = input_feature_tensor.permute({0, 3, 1, 2});
  for (int i = 0; i < input_feature_float.channels(); ++i) {
    input_feature_tensor[0][i] = input_feature_tensor[0][i].sub(0.5).div(0.5);
  }

  auto input_preprocessing_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> preprocessing_diff =
      input_preprocessing_end_time - input_preprocessing_start_time;
  ADEBUG << "trajectory imitation model input preprocessing used time: "
         << preprocessing_diff.count() * 1000 << " ms.";

  auto inference_start_time = std::chrono::system_clock::now();

  std::vector<torch::jit::IValue> torch_inputs;
  torch_inputs.push_back(input_feature_tensor.to(device_));
  at::Tensor torch_output_tensor =
      model_.forward(torch_inputs).toTensor().to(torch::kCPU);

  auto inference_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> inference_diff =
      inference_end_time - inference_start_time;
  ADEBUG << "trajectory imitation model inference used time: "
         << inference_diff.count() * 1000 << " ms.";

  output_postprocessing(torch_output_tensor, learning_data_frame);

  return true;
}

bool TrajectoryImitationLibtorchInference::DoCNNLSTMMODELInference(
    LearningDataFrame* const learning_data_frame) {
  const int past_points_size = learning_data_frame->adc_trajectory_point_size();
  if (past_points_size == 0) {
    AERROR << "No current trajectory point status";
    return false;
  }

  auto input_renderering_start_time = std::chrono::system_clock::now();

  cv::Mat input_feature;
  if (!BirdviewImgFeatureRenderer::Instance()->RenderMultiChannelEnv(
          *learning_data_frame, &input_feature)) {
    AERROR << "Render multi-channel input image failed";
    return false;
  }

  auto input_renderering_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> rendering_diff =
      input_renderering_end_time - input_renderering_start_time;
  ADEBUG << "trajectory imitation model input renderer used time: "
         << rendering_diff.count() * 1000 << " ms.";

  auto input_preprocessing_start_time = std::chrono::system_clock::now();

  cv::Mat input_feature_float;
  input_feature.convertTo(input_feature_float, CV_32F, 1.0 / 255);
  torch::Tensor input_feature_tensor =
      torch::from_blob(input_feature_float.data,
                       {1, input_feature_float.rows, input_feature_float.cols,
                        input_feature_float.channels()});
  input_feature_tensor = input_feature_tensor.permute({0, 3, 1, 2});
  for (int i = 0; i < input_feature_float.channels(); ++i) {
    input_feature_tensor[0][i] = input_feature_tensor[0][i].sub(0.5).div(0.5);
  }

  const auto& current_traj_point =
      learning_data_frame
          ->adc_trajectory_point(
              learning_data_frame->adc_trajectory_point_size() - 1)
          .trajectory_point();

  torch::Tensor current_v_tensor = torch::zeros({1, 1});
  const double current_v = current_traj_point.v();
  current_v_tensor[0][0] = current_v;

  auto input_preprocessing_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> preprocessing_diff =
      input_preprocessing_end_time - input_preprocessing_start_time;
  ADEBUG << "trajectory imitation model input preprocessing used time: "
         << preprocessing_diff.count() * 1000 << " ms.";

  auto inference_start_time = std::chrono::system_clock::now();

  std::vector<torch::jit::IValue> torch_inputs;
  torch_inputs.push_back(
      c10::ivalue::Tuple::create({std::move(input_feature_tensor.to(device_)),
                                  std::move(current_v_tensor.to(device_))}));
  at::Tensor torch_output_tensor =
      model_.forward(torch_inputs).toTensor().to(torch::kCPU);

  auto inference_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> inference_diff =
      inference_end_time - inference_start_time;
  ADEBUG << "trajectory imitation model inference used time: "
         << inference_diff.count() * 1000 << " ms.";

  output_postprocessing(torch_output_tensor, learning_data_frame);

  return true;
}

bool TrajectoryImitationLibtorchInference::DoInference(
    LearningDataFrame* const learning_data_frame) {
  switch (config_.model_type()) {
    case LearningModelInferenceTaskConfig::CNN: {
      if (!DoCNNMODELInference(learning_data_frame)) {
        return false;
      }
      break;
    }
    case LearningModelInferenceTaskConfig::CNN_LSTM: {
      if (!DoCNNLSTMMODELInference(learning_data_frame)) {
        return false;
      }
      break;
    }
    default: {
      AERROR << "Configured model type not defined and implemented";
      break;
    }
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
