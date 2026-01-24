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

#pragma once

#include <NvInferPlugin.h>
#include <NvInferRuntimeCommon.h>
#include <NvInferVersion.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "NvInfer.h"
#include "NvOnnxParser.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace inference {

class MultiBatchInference : public Inference {
public:
    MultiBatchInference() {}

    virtual ~MultiBatchInference();

    /**
     * @brief Convert ONNX to TensorRT model
     * @param[in] model_file ONNX model file path
     * @details Load ONNX model, and convert it to TensorRT model
     */
    bool OnnxToTRTModel(const std::string &model_file);

    void SetOnnxOptimizationProfile(nvinfer1::IOptimizationProfile *profile, const BlobMap &input_shapes);

    void inference();
    bool LoadCache(const std::string &path);
    bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
    void Infer() override;
    void init_blob(std::vector<std::string> *names);
    base::BlobPtr<float> get_blob(const std::string &name) override;

    void FileInit();

    void SetStream(cudaStream_t stream) {
        stream_ = stream;
    }

private:
    std::string model_file_;
    float score_threshold_;
    std::vector<std::string> output_names_;
    std::vector<std::string> input_names_;
    BlobMap blobs_;

    std::mutex mutex_;
    int bindingIndex_ = 0;
    nvinfer1::INetworkDefinition *network_ = nullptr;
    nvinfer1::ICudaEngine *engine_ = nullptr;
    nvinfer1::IExecutionContext *context_ = nullptr;
    nvinfer1::IBuilder *builder_ = nullptr;

    std::vector<void *> buffers_;
    cudaStream_t stream_ = 0;
    int workspaceSize_ = 1;

    int num_classes_;
    int kBatchSize;
    bool dynamic_flag_ = false;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
        apollo::perception::inference::MultiBatchInference,
        apollo::perception::inference::Inference)
}  // namespace inference
}  // namespace perception
}  // namespace apollo
