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

#include "modules/prediction_plugin/multi_agent_vehicle_tensorrt/multi_agent_vehicle_tensorrt.h"

#include <fstream>
#include <cxxabi.h>
#include <cuda_runtime_api.h>

#include "cyber/common/file.h"
#include "modules/prediction/proto/prediction_conf.pb.h"

namespace apollo {
namespace prediction {

bool MultiAgentVehicleTensorrt::LoadModel() {
    std::ifstream engine_file(model_path_, std::ios::binary);
    engine_file.seekg(0, std::ifstream::end);
    int64_t engine_size = engine_file.tellg();
    ACHECK(engine_size > 0);
    engine_file.seekg(0, std::ifstream::beg);

    void* engine_blob = malloc(engine_size);
    engine_file.read(reinterpret_cast<char*>(engine_blob), engine_size);

    runtime_ = nvinfer1::createInferRuntime(rt_gLogger);
    if (!runtime_) {
        AERROR << "create runtime failed";
        return false;
    }

    engine_ = runtime_->deserializeCudaEngine(engine_blob, engine_size);
    if (!engine_) {
        AERROR << "create engine failed";
        return false;
    }

    context_ = engine_->createExecutionContext();
    if (!context_) {
        AERROR << "create context failed";
        return false;
    }
    // check model input output
    ACHECK(engine_->getNbBindings() == 9);

    input_multi_obstacle_pos_index_ =
        engine_->getBindingIndex(INPUT_MULTI_OBSTACLE_POS_NAME);
    input_multi_obstacle_pos_step_index_ =
        engine_->getBindingIndex(INPUT_MULTI_OBSTACLE_POS_STEP_NAME);
    input_vector_data_index_ =
        engine_->getBindingIndex(INPUT_VECTOR_DATA_NAME);
    input_bool_vector_mask_index_ =
        engine_->getBindingIndex(INPUT_BOOL_VECTOR_MASK_NAME);
    input_bool_polyline_mask_index_ =
        engine_->getBindingIndex(INPUT_BOOL_POLYLINE_MASK_NAME);
    input_rand_mask_index_ =
        engine_->getBindingIndex(INPUT_RAND_MASK_NAME);
    input_polyline_id_index_ =
        engine_->getBindingIndex(INPUT_POLYLINE_ID_NAME);
    input_obs_position_index_ =
        engine_->getBindingIndex(INPUT_OBS_POSITION_NAME);
    output_index_ =
        engine_->getBindingIndex(OUTPUT_NAME);

    buffers_ = std::vector<void*>{nullptr, nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr, nullptr, nullptr};

    // size based on the shape of model input and output
    cudaMalloc(&buffers_[input_multi_obstacle_pos_index_],
        1 * max_agent_num * 20 * 2 * sizeof(float));
    cudaMalloc(&buffers_[input_multi_obstacle_pos_step_index_],
        1 * max_agent_num * 20 * 2 * sizeof(float));
    cudaMalloc(&buffers_[input_vector_data_index_],
        1 * 450 * 50 * 9 * sizeof(float));
    cudaMalloc(&buffers_[input_bool_vector_mask_index_],
        1 * 450 * 50 * sizeof(bool));
    cudaMalloc(&buffers_[input_bool_polyline_mask_index_],
        1 * 450 * sizeof(bool));
    cudaMalloc(&buffers_[input_rand_mask_index_],
        1 * 450 * sizeof(bool));
    cudaMalloc(&buffers_[input_polyline_id_index_],
        1 * 450 * 2 * sizeof(float));
    cudaMalloc(&buffers_[input_obs_position_index_],
        1 * max_agent_num * 3 * sizeof(float));
    cudaMalloc(&buffers_[output_index_],
        1 * max_agent_num * 30 * 2 * sizeof(float));

    cudaStreamCreate(&stream_);

    return true;
}

bool MultiAgentVehicleTensorrt::Inference(
        const std::vector<void*>& input_buffer,
        unsigned int input_size,
        std::vector<void*>* output_buffer,
        unsigned int output_size) {
    ACHECK(input_size == input_buffer.size() && input_size == 8);
    ACHECK(output_size == output_buffer->size() && output_size == 1);

    if (init_ == 0) {
        Init();
    }

    // ensure thread safe for inference
    std::lock_guard<std::mutex> lck(mtx);

    cudaMemcpyAsync(
            buffers_[input_multi_obstacle_pos_index_],
            input_buffer[0],
            1 * max_agent_num * 20 * 2 * sizeof(float),
            cudaMemcpyHostToDevice,
            stream_);
    cudaMemcpyAsync(
            buffers_[input_multi_obstacle_pos_step_index_],
            input_buffer[1],
            1 * max_agent_num * 20 * 2 * sizeof(float),
            cudaMemcpyHostToDevice,
            stream_);
    cudaMemcpyAsync(
            buffers_[input_vector_data_index_],
            input_buffer[2],
            1 * 450 * 50 * 9 * sizeof(float),
            cudaMemcpyHostToDevice,
            stream_);
    cudaMemcpyAsync(
            buffers_[input_bool_vector_mask_index_],
            input_buffer[3],
            1 * 450 * 50 * sizeof(bool),
            cudaMemcpyHostToDevice,
            stream_);
    cudaMemcpyAsync(
            buffers_[input_bool_polyline_mask_index_],
            input_buffer[4],
            1 * 450 * sizeof(bool),
            cudaMemcpyHostToDevice,
            stream_);
    cudaMemcpyAsync(
            buffers_[input_rand_mask_index_],
            input_buffer[5],
            1 * 450 * sizeof(bool),
            cudaMemcpyHostToDevice,
            stream_);
    cudaMemcpyAsync(
            buffers_[input_polyline_id_index_],
            input_buffer[6],
            1 * 450 * 2 * sizeof(float),
            cudaMemcpyHostToDevice,
            stream_);
    cudaMemcpyAsync(
            buffers_[input_obs_position_index_],
            input_buffer[7],
            1 * max_agent_num * 3 * sizeof(float),
            cudaMemcpyHostToDevice,
            stream_);
    if (!context_->enqueueV2(buffers_.data(), stream_, nullptr)) {
        return false;
    }
    cudaMemcpyAsync(
            (*output_buffer)[0], buffers_[output_index_],
            1 * max_agent_num * 30 * 2 * sizeof(float),
            cudaMemcpyDeviceToHost,
            stream_);
    cudaStreamSynchronize(stream_);
    return true;
}

void MultiAgentVehicleTensorrt::Destory() {
    cudaStreamDestroy(stream_);
    cudaFree(buffers_[input_multi_obstacle_pos_index_]);
    cudaFree(buffers_[input_multi_obstacle_pos_step_index_]);
    cudaFree(buffers_[input_vector_data_index_]);
    cudaFree(buffers_[input_bool_vector_mask_index_]);
    cudaFree(buffers_[input_bool_polyline_mask_index_]);
    cudaFree(buffers_[input_rand_mask_index_]);
    cudaFree(buffers_[input_polyline_id_index_]);
    cudaFree(buffers_[input_obs_position_index_]);
    cudaFree(buffers_[output_index_]);
}

bool MultiAgentVehicleTensorrt::Init() {
    ModelConf model_config;
    int status = 0;

    if (init_ != 0) {
        return true;
    }

    std::string class_name = abi::__cxa_demangle(
        typeid(*this).name(), 0, 0, &status);
    std::string default_config_path
        = apollo::cyber::plugin_manager::PluginManager::Instance()->
        GetPluginConfPath<ModelBase>(class_name, "conf/default_conf.pb.txt");

    if (!cyber::common::GetProtoFromFile(default_config_path, &model_config)) {
        AERROR << "Unable to load model conf file: " << default_config_path;
        return false;
    }
    model_path_ = model_config.model_path();
    init_ = 1;

    return LoadModel();
}

}  // namespace prediction
}  // namespace apollo
