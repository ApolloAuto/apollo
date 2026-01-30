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
#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR == 8
#include "modules/perception/common/inference/tensorrt/rt_legacy.h"
#endif
#endif

#include <sys/stat.h>

#include <fstream>

#include <cuda_runtime_api.h>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/common/inference/model_util.h"
#include "modules/perception/common/util.h"
#include "modules/perception/perception_plugin/inference/onnx_multi_batch/onnx_multi_batch_infer.h"

// Logger for TensorRT info/warning/errors
class OnnxLogger : public nvinfer1::ILogger {
public:
    OnnxLogger(Severity severity = Severity::kWARNING) : mReportableSeverity(severity) {}
    nvinfer1::ILogger& getTRTLogger() {
        return *this;
    }
    Severity getReportableSeverity() const {
        return mReportableSeverity;
    }

    Severity mReportableSeverity;

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
} onnx_gLogger;

namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

#define GPU_CHECK(ans) \
    { GPUAssert((ans), __FILE__, __LINE__); }
inline void GPUAssert(cudaError_t code, const char* file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort)
            exit(code);
    }
}

base::BlobPtr<float> MultiBatchInference::get_blob(const std::string& name) {
    auto iter = blobs_.find(name);
    if (iter == blobs_.end()) {
        return nullptr;
    }
    return iter->second;
}

MultiBatchInference::~MultiBatchInference() {
    if (network_) {
        network_->destroy();
    }
    if (builder_) {
        builder_->destroy();
    }
    if (context_) {
        context_->destroy();
    }
    if (engine_) {
        engine_->destroy();
    }
}

bool MultiBatchInference::LoadCache(const std::string& path) {
    struct stat buffer;
    if (stat(path.c_str(), &buffer) != 0) {
        AERROR << "[INFO] cannot find model cache : " << path << ", it will take minutes to generate...";
        return false;
    }
    return true;
}

void MultiBatchInference::FileInit() {
    model_file_ = proto_file_;
    output_names_ = net_output_names_;
    input_names_ = net_input_names_;
}

bool MultiBatchInference::Init(const std::map<std::string, std::vector<int>>& shapes) {
    BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
    BASE_GPU_CHECK(cudaStreamCreate(&stream_));

    FileInit();

    // create a TensorRT model from the onnx model and load it into an engine
    OnnxToTRTModel(model_file_);

    if (engine_ == nullptr) {
        AERROR << "Fail to load obstacle ONNX model";
        return false;
    }

    // create execution context from the engine
    context_ = engine_->createExecutionContext();
    if (context_ == nullptr) {
        AERROR << "Fail to create Exceution Context";
        return false;
    }
    for (const auto& name : output_names_) {
        auto iter = shapes.find(name);
        if (iter != shapes.end()) {
            auto blob = std::make_shared<Blob<float>>(iter->second);
            blobs_.emplace(name, blob);
        }
    }

    for (const auto& name : input_names_) {
        auto iter = shapes.find(name);
        if (iter != shapes.end()) {
            auto blob = std::make_shared<Blob<float>>(iter->second);
            blobs_.emplace(name, blob);
        }
    }
    buffers_.resize(input_names_.size() + output_names_.size());

    // init_blob(&input_names_);
    // init_blob(&output_names_);

    return true;
}

void MultiBatchInference::SetOnnxOptimizationProfile(
        nvinfer1::IOptimizationProfile* profile,
        const BlobMap& input_shapes) {
    // Optimizat every input
    // TODO(huanglei14) Distinguish inputs that do not require optimization
    for (int i = 0; i < network_->getNbInputs(); ++i) {
        nvinfer1::ITensor* nv_input = network_->getInput(i);
        const char* nv_input_name = nv_input->getName();
        nvinfer1::Dims nv_input_dims = nv_input->getDimensions();
        auto it = input_shapes.find(nv_input_name);
        if (it != input_shapes.end()) {
            CHECK_EQ(nv_input_dims.nbDims, input_shapes.at(nv_input_name)->num_axes());
        }
        AINFO << "nv_input_name: " << nv_input_name;
        AINFO << "nv_input_dims: ";
        AINFO << "\t" << nv_input_dims.d[0];
        if (nv_input_dims.d[0] > 0) {
            continue;
        }

        for (int j = 1; j < nv_input_dims.nbDims; ++j) {
            AINFO << "\t" << nv_input_dims.d[j];
            if (it != input_shapes.end()) {
                // input dims should be dynamic, or equal to those specified by init
                // options
                CHECK(nv_input_dims.d[j] < 0 || nv_input_dims.d[j] == it->second->shape(j));
                nv_input_dims.d[j] = it->second->shape(j);
            } else {
                // input should have fixed dims if not specfied by init options
                CHECK(nv_input_dims.d[j] > 0);
            }
        }

        nv_input_dims.d[0] = 1;
        profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kMIN, nv_input_dims);
        nv_input_dims.d[0] = std::max(1, static_cast<int>(max_batch_size_ / 2));
        profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kOPT, nv_input_dims);
        nv_input_dims.d[0] = max_batch_size_;
        profile->setDimensions(nv_input_name, nvinfer1::OptProfileSelector::kMAX, nv_input_dims);
    }
}

bool MultiBatchInference::OnnxToTRTModel(const std::string& model_file) {
    if (gpu_id_ < 0) {
        AERROR << "must use gpu mode";
        return false;
    }
    // create the builder
    builder_ = nvinfer1::createInferBuilder(onnx_gLogger);
    network_ = builder_->createNetworkV2(
            1U << static_cast<int>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));

    // parse onnx model
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network_, onnx_gLogger.getTRTLogger());

    const char* onnx_filename = model_file.c_str();
    int verbosity = static_cast<int>(onnx_gLogger.getReportableSeverity());
    if (!parser->parseFromFile(onnx_filename, verbosity)) {
        AERROR << "Failure while parsing ONNX file: " << onnx_filename;
        return false;
    }

    // Build the engine
    builder_->setMaxBatchSize(max_batch_size_);
    nvinfer1::IBuilderConfig* config = builder_->createBuilderConfig();
    config->setMaxWorkspaceSize(1 << 30);
    bool use_fp16 = builder_->platformHasFastFp16();
    if (use_fp16) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    config->setFlag(nvinfer1::BuilderFlag::kDEBUG);

    auto trt_cache_path = model_file + ".trt.engine";

    if (!LoadCache(trt_cache_path)) {
        nvinfer1::IOptimizationProfile* profile = builder_->createOptimizationProfile();
        SetOnnxOptimizationProfile(profile, blobs_);
        config->addOptimizationProfile(profile);

        engine_ = builder_->buildEngineWithConfig(*network_, *config);
        nvinfer1::IHostMemory* ser_mem = engine_->serialize();

        if (ser_mem) {
            nvinfer1::IHostMemory* serialized_model(ser_mem);
            std::ofstream f(trt_cache_path, std::ios::binary);

            f.write(reinterpret_cast<char*>(serialized_model->data()), serialized_model->size());
            serialized_model->destroy();
        }
    } else {
        AINFO << "Loading TensorRT engine from serialized model file...";
        std::ifstream planFile(trt_cache_path);

        if (!planFile.is_open()) {
            AERROR << "Could not open serialized model";
            return false;
        }
        initLibNvInferPlugins(&onnx_gLogger, "");

        AINFO << "success open serialized model";
        std::stringstream planBuffer;
        planBuffer << planFile.rdbuf();
        std::string plan = planBuffer.str();
        nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(onnx_gLogger);

        engine_ = runtime->deserializeCudaEngine(static_cast<const void*>(plan.data()), plan.size(), nullptr);
        runtime->destroy();
    }
    config->destroy();
    parser->destroy();
    return true;
}

void MultiBatchInference::inference() {
    AINFO << "Do Inference";
}

void MultiBatchInference::init_blob(std::vector<std::string>* names) {
    for (auto name : *names) {
        auto blob = get_blob(name);

        std::vector<int> shape(blob->shape().begin(), blob->shape().end());
        blob.reset(new apollo::perception::base::Blob<float>(shape));

        buffers_[bindingIndex_] = blob->mutable_gpu_data();

        bindingIndex_ += 1;
    }
}

void MultiBatchInference::Infer() {
    BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
    BASE_GPU_CHECK(cudaStreamSynchronize(stream_));
    CHECK(input_names_.size() > 0 && blobs_.size() > 0);
    auto input_shape = blobs_[input_names_[0]]->shape();
    kBatchSize = input_shape[0];
    if (kBatchSize == 0) {
        AINFO << "batch size is zero.";
        return;
    }

    int batch_size = max_batch_size_;

    // binding input blob
    for (auto name : input_names_) {
        auto blob = get_blob(name);
        if (blob != nullptr) {
            int32_t index = engine_->getBindingIndex(name.c_str());
            // careful about this line
            // batch_size = std::min<int>(batch_size, blob->shape(0));
            batch_size = blob->shape(0);
            // set buffer
            buffers_[index] = const_cast<float*>(blob->gpu_data());

            // set Dim
            nvinfer1::Dims dims = network_->getInput(index)->getDimensions();
            dims.d[0] = batch_size;
            context_->setOptimizationProfile(0);
            context_->setBindingDimensions(index, dims);
        }
    }

    // bingding output blob
    for (auto name : output_names_) {
        auto blob = get_blob(name);
        if (blob != nullptr) {
            blob->mutable_gpu_data();
            int32_t index = engine_->getBindingIndex(name.c_str());
            buffers_[index] = blob->mutable_gpu_data();
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        context_->enqueueV2(&buffers_[0], stream_, nullptr);
    }

    BASE_GPU_CHECK(cudaStreamSynchronize(stream_));
    for (auto name : output_names_) {
        auto blob = get_blob(name);
        if (blob != nullptr) {
            blob->mutable_gpu_data();
        }
    }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
