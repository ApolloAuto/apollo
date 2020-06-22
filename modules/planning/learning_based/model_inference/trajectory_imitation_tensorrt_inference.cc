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

#include "modules/planning/learning_based/model_inference/trajectory_imitation_tensorrt_inference.h"

#include <cuda_runtime_api.h>

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"
#include "opencv2/opencv.hpp"
#include "torch/extension.h"
#include "torch/script.h"

namespace apollo {
namespace planning {

#define GPU_CHECK(ans) \
  { GPUAssert((ans), __FILE__, __LINE__); }
inline void GPUAssert(cudaError_t code, const char* file, int line,
                      bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    if (abort) exit(code);
  }
}

// TODO(Jinyun): move to conf
static const int BATCH_SIZE = 1;
static const int IMG_SIZE = 200;
static const int FEATURE_CHANNELS_NUM = 12;
static const int INITIAL_STATES_CHANNELS_NUM = 1;
static const int PREDICTION_STATES_NUM = 4;
static const int PREDICTION_HORIZON = 10;
static const double DELTA_T = 0.2;

TrajectoryImitationTensorRTInference::TrajectoryImitationTensorRTInference(
    const LearningModelInferenceTaskConfig& config)
    : ModelInference(config) {
}

TrajectoryImitationTensorRTInference::~TrajectoryImitationTensorRTInference() {
  GPU_CHECK(cudaFree(trt_buffers_[0]));
  GPU_CHECK(cudaFree(trt_buffers_[1]));
  GPU_CHECK(cudaFree(trt_buffers_[2]));
  GPU_CHECK(cudaFree(trt_buffers_[3]));

  trt_context_->destroy();
  trt_engine_->destroy();
}

void TrajectoryImitationTensorRTInference::DeviceMemoryMalloc() {
  // TODO(Jinyun): move to conf
  GPU_CHECK(cudaMalloc(
      &trt_buffers_[0],
      BATCH_SIZE * FEATURE_CHANNELS_NUM * IMG_SIZE * IMG_SIZE * sizeof(float)));
  GPU_CHECK(
      cudaMalloc(&trt_buffers_[1], BATCH_SIZE * INITIAL_STATES_CHANNELS_NUM *
                                       IMG_SIZE * IMG_SIZE * sizeof(float)));
  GPU_CHECK(
      cudaMalloc(&trt_buffers_[2], BATCH_SIZE * INITIAL_STATES_CHANNELS_NUM *
                                       IMG_SIZE * IMG_SIZE * sizeof(float)));
  GPU_CHECK(cudaMalloc(
      &trt_buffers_[3],
      BATCH_SIZE * PREDICTION_HORIZON * PREDICTION_STATES_NUM * sizeof(float)));
}

bool TrajectoryImitationTensorRTInference::LoadModel() {
  auto start_time = std::chrono::system_clock::now();
  // create a TensorRT model from the onnx model and serialize it to a stream
  // create the builder
  const auto explicit_batch =
      1U << static_cast<uint32_t>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(g_logger_);
  nvinfer1::INetworkDefinition* network =
      builder->createNetworkV2(explicit_batch);

  auto parser = nvonnxparser::createParser(*network, g_logger_);
  if (!parser->parseFromFile(
          config_.model_file().c_str(),
          static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) {
    AERROR << "failed to parse onnx file";
    return false;
  }

  // Build the engine
  builder->setMaxBatchSize(BATCH_SIZE);
  nvinfer1::IBuilderConfig* trt_config = builder->createBuilderConfig();
  // Allow TensorRT to use up to 1GB of GPU memory for tactic selection.
  trt_config->setMaxWorkspaceSize(1 << 30);
  trt_engine_ = builder->buildEngineWithConfig(*network, *trt_config);
  if (trt_engine_ == nullptr) {
    AERROR << "Failed to create TensorRT Engine.";
    return false;
  }

  parser->destroy();
  network->destroy();
  trt_config->destroy();
  builder->destroy();

  trt_context_ = trt_engine_->createExecutionContext();
  if (trt_context_ == nullptr) {
    AERROR << "Failed to create TensorRT Execution Context.";
    return false;
  }

  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  ADEBUG << "initialization of tensorRT engine used time: "
         << diff.count() * 1000 << " ms.";

  DeviceMemoryMalloc();

  return true;
}

bool TrajectoryImitationTensorRTInference::DoInference(
    LearningDataFrame* learning_data_frame) {
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
  cv::Mat initial_point;
  if (!BirdviewImgFeatureRenderer::Instance()->RenderCurrentEgoPoint(
          *learning_data_frame, &initial_point)) {
    AERROR << "Render initial states image failed";
    return false;
  }
  cv::Mat initial_box;
  if (!BirdviewImgFeatureRenderer::Instance()->RenderCurrentEgoBox(
          *learning_data_frame, &initial_box)) {
    AERROR << "Render initial states image failed";
    return false;
  }
  auto input_renderering_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> rendering_diff =
      input_renderering_end_time - input_renderering_start_time;
  ADEBUG << "trajectory imitation model input renderer used time: "
         << rendering_diff.count() * 1000 << " ms.";

  auto input_prepration_start_time = std::chrono::system_clock::now();
  // TODO(Jinyun): use other method to preprocess the data rather than the
  // method from torch::Tensor
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
  cv::Mat initial_point_float;
  initial_point.convertTo(initial_point_float, CV_32F, 1.0 / 255);
  torch::Tensor initial_point_tensor =
      torch::from_blob(initial_point_float.data,
                       {1, initial_point_float.rows, initial_point_float.cols,
                        initial_point_float.channels()});
  initial_point_tensor = initial_point_tensor.permute({0, 3, 1, 2});
  cv::Mat initial_box_float;
  initial_box.convertTo(initial_box_float, CV_32F, 1.0 / 255);
  torch::Tensor initial_box_tensor =
      torch::from_blob(initial_box_float.data,
                       {BATCH_SIZE, initial_box_float.rows,
                        initial_box_float.cols, initial_box_float.channels()});
  initial_box_tensor = initial_box_tensor.permute({0, 3, 1, 2});
  auto input_prepration_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> prepration_diff =
      input_prepration_end_time - input_prepration_start_time;
  ADEBUG << "trajectory imitation model input prepration used time: "
         << prepration_diff.count() * 1000 << " ms.";

  auto inference_start_time = std::chrono::system_clock::now();
  cudaStream_t stream;
  GPU_CHECK(cudaStreamCreate(&stream));
  GPU_CHECK(cudaMemcpyAsync(
      trt_buffers_[0], input_feature_tensor.data_ptr(),
      BATCH_SIZE * FEATURE_CHANNELS_NUM * IMG_SIZE * IMG_SIZE * sizeof(float),
      cudaMemcpyHostToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(trt_buffers_[1], initial_point_tensor.data_ptr(),
                            BATCH_SIZE * INITIAL_STATES_CHANNELS_NUM *
                                IMG_SIZE * IMG_SIZE * sizeof(float),
                            cudaMemcpyHostToDevice, stream));
  GPU_CHECK(cudaMemcpyAsync(trt_buffers_[2], initial_box_tensor.data_ptr(),
                            BATCH_SIZE * INITIAL_STATES_CHANNELS_NUM *
                                IMG_SIZE * IMG_SIZE * sizeof(float),
                            cudaMemcpyHostToDevice, stream));
  trt_context_->enqueueV2(trt_buffers_, stream, nullptr);
  float pred_point[1 * PREDICTION_HORIZON * PREDICTION_STATES_NUM *
                   sizeof(float)] = {0};
  GPU_CHECK(cudaMemcpyAsync(
      pred_point, trt_buffers_[3],
      BATCH_SIZE * PREDICTION_HORIZON * PREDICTION_STATES_NUM * sizeof(float),
      cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
  auto inference_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> inference_diff =
      inference_end_time - inference_start_time;
  ADEBUG << "trajectory imitation model inference used time: "
         << inference_diff.count() * 1000 << " ms.";

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
  for (int i = 0; i < PREDICTION_HORIZON; ++i) {
    const double dx =
        static_cast<double>(pred_point[i * PREDICTION_STATES_NUM]);
    const double dy =
        static_cast<double>(pred_point[i * PREDICTION_STATES_NUM + 1]);
    const double dtheta =
        static_cast<double>(pred_point[i * PREDICTION_STATES_NUM + 2]);
    const double v =
        static_cast<double>(pred_point[i * PREDICTION_STATES_NUM + 3]);
    ADEBUG << "dx[" << dx << "], dy[" << dy << "], dtheta[" << dtheta << "], v["
           << v << "] ";
    const double time_sec = cur_time_sec + DELTA_T * (i + 1);
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
    traj_point->mutable_trajectory_point()->set_relative_time(DELTA_T *
                                                              (i + 1));
  }

  // release the stream and the buffers
  cudaStreamDestroy(stream);
  return true;
}

}  // namespace planning
}  // namespace apollo
