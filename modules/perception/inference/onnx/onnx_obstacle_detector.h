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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <fstream>
#include <cuda_runtime_api.h>

#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "NvInferVersion.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

using BlobPtr = std::shared_ptr<apollo::perception::base::Blob<float>>;

class OnnxObstacleDetector : public Inference {
public:
  OnnxObstacleDetector(const std::string &model_file,
                        const int num_classes,
                        const std::string &input_image_file,
                        const std::string &input_name_file,
                        const std::string &prediction_image_path);

  OnnxObstacleDetector(const std::string &model_file,
                        const std::vector<std::string> &outputs,
                        const std::vector<std::string> &inputs);

	virtual ~OnnxObstacleDetector();

  void OnnxToTRTModel(const std::string& model_file,
    nvinfer1::IHostMemory** trt_model_stream);

  void TRTStreamToContext(
  const nvinfer1::IHostMemory* yolov4_trt_model_stream, 
  nvinfer1::IExecutionContext** context_ptr);

  void TRTStreamToContext(
  const std::vector<char>& trt_model_stream,
  nvinfer1::IExecutionContext** context_ptr);

  void postProcessing(cv::Mat& img,
  float* output, int row_cnt,
  int num_classes,
  const std::vector<std::string>& names);

  void inference(nvinfer1::IExecutionContext* yolov4_context_, 
  int num_classes, const std::vector<std::string>& names, 
  const std::string& image_path, const std::string& prediction_image_path);

  void readNames(
  const std::string& names_file_path,
  std::vector<std::string>& names);

	bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
	void Infer() override;
  BlobPtr get_blob(const std::string &name) override;

private:
  std::string model_file_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;
  nvinfer1::IExecutionContext* context_;

  int num_classes_;
  std::vector<std::string> names_;
  std::string image_path_;
  std::string names_file_path_; // coco.names
  std::string prediction_image_path_;
};

} // namespace inference
} // namespace perception
} // namespace apollo