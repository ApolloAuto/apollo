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

#include "modules/perception/inference/onnx/onnx_obstacle_detector.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

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

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger {
 public:
  explicit Logger(Severity severity = Severity::kWARNING)
      : reportable_severity(severity) {}

  void log(Severity severity, const char* msg) override {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportable_severity) return;

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

  Severity reportable_severity;
};

Logger g_logger_;

OnnxObstacleDetector::OnnxObstacleDetector(
  const std::string &model_file,
  int num_classes,
  const std::string &input_image_file,
  const std::string &input_name_file,
  const std::string &prediction_image_path)
  : model_file_(model_file),
    num_classes_(num_classes),
    image_path_(input_image_file),
    names_file_path_(input_name_file),
    prediction_image_path_(prediction_image_path) {}

OnnxObstacleDetector::OnnxObstacleDetector(
  const std::string &model_file,
  const std::vector<std::string> &outputs,
  const std::vector<std::string> &inputs)
  : model_file_(model_file), output_names_(outputs), input_names_(inputs) {}

OnnxObstacleDetector::~OnnxObstacleDetector() {}

void OnnxObstacleDetector::OnnxToTRTModel(
    const std::string& model_file,  // name of the onnx model
    nvinfer1::IHostMemory** trt_model_stream) {  // output buffer for the TensorRT model
  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);

  int kBatchSize = 1;

  // create the builder
  const auto explicit_batch =
      static_cast<uint32_t>(kBatchSize) << static_cast<uint32_t>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(g_logger_);

  nvinfer1::INetworkDefinition* network =
      builder->createNetworkV2(explicit_batch);

  auto parser = nvonnxparser::createParser(*network, g_logger_);
  if (!parser->parseFromFile(model_file.c_str(), verbosity)) {
    std::string msg("failed to parse onnx file");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  // Build the engine
  builder->setMaxBatchSize(kBatchSize);
  nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
  config->setMaxWorkspaceSize(1<<20);
  config->setFlag(nvinfer1::BuilderFlag::kFP16);
  nvinfer1::ICudaEngine* engine =
      builder->buildEngineWithConfig(*network, *config);

  // serialize the engine, then close everything down
  *trt_model_stream = engine->serialize();

  parser->destroy();
  network->destroy();
  config->destroy();
  builder->destroy();
  
  engine->destroy();
}

void OnnxObstacleDetector::TRTStreamToContext(
  const nvinfer1::IHostMemory* yolov4_trt_model_stream, 
  nvinfer1::IExecutionContext** context_ptr) {

  nvinfer1::ICudaEngine* engine;
  nvinfer1::IRuntime* runtime;

  // deserialize the engine
  runtime = nvinfer1::createInferRuntime(g_logger_);
  if (runtime == nullptr) {
    std::cerr << "Failed to create TensorRT Runtime object." << std::endl;
  }

  engine = runtime->deserializeCudaEngine(
      yolov4_trt_model_stream->data(), yolov4_trt_model_stream->size(), nullptr);

  if (engine == nullptr) {
    std::cerr << "Failed to create TensorRT Engine." << std::endl;
  }

  std::cout << "Deserialize engine success" << std::endl;

  *context_ptr = engine->createExecutionContext();
  if (*context_ptr == nullptr) {
    std::cerr << "Failed to create TensorRT Execution Context." << std::endl;
  }

  std::cout << "Create context success" << std::endl;
}

void OnnxObstacleDetector::TRTStreamToContext(
  const std::vector<char>& trt_model_stream,
  nvinfer1::IExecutionContext** context_ptr) {

  nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(g_logger_);
  if (runtime == nullptr) {
    std::cerr << "Failed to create TensorRT Runtime object." << std::endl;
  }

  nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(trt_model_stream.data(), trt_model_stream.size(), nullptr);

  if (engine == nullptr) {
    std::cerr << "Failed to create TensorRT Engine." << std::endl;
  }

  std::cout << "Deserialize engine success" << std::endl;

  *context_ptr = engine->createExecutionContext();
  if (*context_ptr == nullptr) {
    std::cerr << "Failed to create TensorRT Execution Context." << std::endl;
  }

  std::cout << "Create context success" << std::endl;
}

void OnnxObstacleDetector::postProcessing(cv::Mat& img,
  float* output, int row_cnt,
  int num_classes,
  const std::vector<std::string>& names) {
  //TODO, only draw the bbox with higheest confidence.
  int col_cnt = num_classes + 4;

  int maxid_i; // instance id
  int maxid_j; // class id
  float maxv = -1;
  for (int i=0; i<row_cnt; i++){
    for (int j=4; j<col_cnt; j++){
      float v = output[i*col_cnt+j];
      if (maxv < v && v > -2 && v < 2){
        maxv = v;
        maxid_i = i;
        maxid_j = j;
      }
    }
  }

  int width = img.cols;
  int height = img.rows;
  float* best_box = output + col_cnt * maxid_i;
  int x1 = int((best_box[0] - best_box[2] / 2.0) * width);
  int y1 = int((best_box[1] - best_box[3] / 2.0) * height);
  int x2 = int((best_box[0] + best_box[2] / 2.0) * width);
  int y2 = int((best_box[1] + best_box[3] / 2.0) * height);

  cv::rectangle(img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(0, 255, 0));
  cv::putText(img, names[maxid_j-4], cv::Point2f(x1,y1), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,0,255,255), 1);
}

void OnnxObstacleDetector::inference(nvinfer1::IExecutionContext* yolov4_context_, 
  int num_classes, const std::vector<std::string>& names, 
  const std::string& image_path, const std::string& prediction_image_path) {

  cv::Mat img_ori = cv::imread(image_path);

  if (img_ori.empty()){
    std::cout << "Load image fail: " << image_path << std::endl;
    return;
  }
  else{
    std::cout << "Load image success: " << image_path << std::endl;
  }

  int w = 416;
  int h = 416;
  cv::Mat img;
  cv::resize(img_ori, img, cv::Size(w, h));
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

  int tot_length = img.rows * img.cols * 3;

  cudaStream_t stream;
  GPU_CHECK(cudaStreamCreate(&stream));

  int input_length = tot_length * sizeof(float);
  int row_cnt = 10647;
  int output_length = 1 * row_cnt * (num_classes + 4) * sizeof(float); //3577392

  float* input_image_ = (float*) malloc(input_length);

  for (int i=0; i<img.rows; i++){
    for (int j=0; j<img.cols; j++){
      for (int k=0; k<3; k++){
        input_image_[k*img.rows*img.cols + i*img.cols + j] = static_cast<float>(img.data[i*img.cols*3+j*3+k]) / 255.0;
      }
    }
  }

  void* buffers[2];
  GPU_CHECK(cudaMalloc(&buffers[0], input_length));
  GPU_CHECK(cudaMemcpy(buffers[0], input_image_, input_length, cudaMemcpyHostToDevice));

  GPU_CHECK(cudaMalloc(&buffers[1], output_length));
  GPU_CHECK(cudaMemset(buffers[1], 0, output_length));
  
  yolov4_context_->enqueueV2(buffers, stream, nullptr);
  std::cout << "Inference success" << std::endl;

  float* output = (float*) malloc(output_length);
  GPU_CHECK(cudaMemcpy(output, buffers[1], output_length, cudaMemcpyDeviceToHost));

  postProcessing(img_ori, output, row_cnt, num_classes, names);

  cv::imwrite(prediction_image_path, img_ori);
  std::cout << "Save prediction image to " << prediction_image_path << std::endl;

  GPU_CHECK(cudaFree(buffers[0]));
  GPU_CHECK(cudaFree(buffers[1]));
  free(input_image_);
  free(output);
  cudaStreamDestroy(stream);
}

void OnnxObstacleDetector::readNames(
  const std::string& names_file_path,
  std::vector<std::string>& names) {
  std::ifstream f_names(names_file_path);
  std::string s;
  while (std::getline(f_names, s)){
    names.push_back(s);
  }
}

bool OnnxObstacleDetector::Init(const std::map<std::string, std::vector<int>> &shapes) {
  std::cout << NV_TENSORRT_MAJOR  << "." << NV_TENSORRT_MINOR << "." << NV_TENSORRT_PATCH << "." << NV_TENSORRT_BUILD  << std::endl;
  return true;
}

void OnnxObstacleDetector::Infer() {
  nvinfer1::IHostMemory* trt_stream{nullptr};
  OnnxToTRTModel(model_file_, &trt_stream);

  TRTStreamToContext(trt_stream, &context_);

  readNames(names_file_path_, names_);
  inference(context_, num_classes_, names_, image_path_, prediction_image_path_);

  trt_stream->destroy();
}

BlobPtr OnnxObstacleDetector::get_blob(const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

} // namespace inference
} // namespace perception
} // namespace apollo