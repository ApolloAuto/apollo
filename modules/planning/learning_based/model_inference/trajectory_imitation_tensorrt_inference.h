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
 * @brief Define the trajectory_imitation_tensorrt_inference class
 */

#pragma once

#include <string>
#include <iostream>

#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "modules/planning/learning_based/model_inference/model_inference.h"

namespace apollo {
namespace planning {

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger {
 public:
  explicit Logger(Severity severity = Severity::kERROR)
      : reportable_severity_(severity) {}

  void log(Severity severity, const char* msg) override {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportable_severity_) return;

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        std::cerr << "INTERNAL_ERROR: ";
        break;
      case Severity::kERROR:
        std::cerr << "ERROR: ";
        break;
      case Severity::kWARNING:
        std::cerr << "WARNING: ";
        break;
      case Severity::kINFO:
        std::cerr << "INFO: ";
        break;
      default:
        std::cerr << "UNKNOWN: ";
        break;
    }
    std::cerr << msg << std::endl;
  }

  Severity reportable_severity_;
};

class TrajectoryImitationTensorRTInference : public ModelInference {
 public:
  /**
   * @brief Constructor
   */
  explicit TrajectoryImitationTensorRTInference(
      const LearningModelInferenceTaskConfig& config);

  /**
   * @brief Destructor
   */
  ~TrajectoryImitationTensorRTInference();

  /**
   * @brief Get the name of model inference
   */
  std::string GetName() override {
    return "TRAJECTORY_IMITATION_TENSORRT_INFERENCE";
  };

  /**
   * @brief Memory allocation for device memory
   */
  void DeviceMemoryMalloc();

  /**
   * @brief load a learned model
   */
  bool LoadModel() override;

  /**
   * @brief inference a learned model
   * @param learning_data_frame input and output intermediate for inference
   */
  bool DoInference(LearningDataFrame* learning_data_frame) override;

 private:
   /**
   * @brief inference a CONV_RNN model
   * @param learning_data_frame input and output intermediate for inference
   */
  bool DoCONVRNNMODELInference(LearningDataFrame* learning_data_frame);

  /**
   * @brief inference a CNN model
   * @param learning_data_frame input and output intermediate for inference
   */
  bool DoCNNMODELInference(LearningDataFrame* learning_data_frame);

  /**
   * @brief inference a CNN_LSTM model
   * @param learning_data_frame input and output intermediate for inference
   */
  bool DoCNNLSTMMODELInference(LearningDataFrame* learning_data_frame);

  Logger g_logger_;
  nvinfer1::IExecutionContext* trt_context_;
  nvinfer1::ICudaEngine* trt_engine_;
  void* trt_buffers_[4]{0};
};

}  // namespace planning
}  // namespace apollo
