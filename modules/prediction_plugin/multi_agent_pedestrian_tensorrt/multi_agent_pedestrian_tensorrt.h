/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <map>
#include <utility>

#include "NvInfer.h"
#include "NvInferRuntime.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/prediction/evaluator/model_manager/model/model_base.h"

namespace apollo {
namespace prediction {

class MultiAgentPedestrianTensorrt : public ModelBase {
 public:
    /**
     * @brief Construct a new Multi Agent Vectornet Tensorrt object
     * 
     */
    MultiAgentPedestrianTensorrt() {}

    /**
     * @brief Destroy the Multi Agent Vectornet Tensorrt object
     * 
     */
    ~MultiAgentPedestrianTensorrt() {
        Destory();
    }

    /**
     * @brief parse model description class and load the model
     *
     * @param model_config class to describe model
     * @return init result, true for success
     */
    virtual bool Init();

    /**
     * @brief performing network inference
     *
     * @param input_buffer vector of input tensor
     * @param input_size size of input_buffer
     * @param output_buffer vector of output tensor
     * @param output_size size of output_buffer
     * @return init result, true for success
     */
    virtual bool Inference(
            const std::vector<void*>& input_buffer,
            unsigned int input_size,
            std::vector<void*>* output_buffer,
            unsigned int output_size);

    /**
     * @brief load the model from file
     *
     * @return loading result, true for success
     */
    virtual bool LoadModel();

    /**
     * @brief free all memory requested, gpu or cpu
     *
     * @return memory release result, true for success
     */
    virtual void Destory();

 private:
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    std::vector<void*> buffers_;
    cudaStream_t stream_;

    int max_agent_num = 50;

    // input output name based on onnx model
    const char* INPUT_MULTI_OBSTACLE_POS_NAME = "multi_obstacle_pos";
    const char* INPUT_MULTI_OBSTACLE_POS_STEP_NAME = "multi_obstacle_pos_step";
    const char* INPUT_VECTOR_DATA_NAME = "vector_data";
    const char* INPUT_BOOL_VECTOR_MASK_NAME = "bool_vector_mask";
    const char* INPUT_BOOL_POLYLINE_MASK_NAME = "bool_polyline_mask";
    const char* INPUT_RAND_MASK_NAME = "rand_mask";
    const char* INPUT_POLYLINE_ID_NAME = "polyline_id";
    const char* INPUT_OBS_POSITION_NAME = "obs_position";
    const char* OUTPUT_NAME = "predict";
    // input output index
    int input_multi_obstacle_pos_index_;
    int input_multi_obstacle_pos_step_index_;
    int input_vector_data_index_;
    int input_bool_vector_mask_index_;
    int input_bool_polyline_mask_index_;
    int input_rand_mask_index_;
    int input_polyline_id_index_;
    int input_obs_position_index_;
    int output_index_;
    std::mutex mtx;

    class RTLogger : public nvinfer1::ILogger {
        /**
         * @brief tensorrt internal log function
         *
         * @param severity level of log
         * @param msg message to be logged
         */
        void log(Severity severity, const char* msg) noexcept override {
            switch (severity) {
            case Severity::kINTERNAL_ERROR:
            case Severity::kERROR:
                AERROR << msg;
                break;
            case Severity::kWARNING:
                AWARN << msg;
                break;
            case Severity::kINFO:
            case Severity::kVERBOSE:
                ADEBUG << msg;
                break;
            default:
                break;
            }
        }
    } rt_gLogger;
};
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::prediction::MultiAgentPedestrianTensorrt, ModelBase)

}  // namespace prediction
}  // namespace apollo
