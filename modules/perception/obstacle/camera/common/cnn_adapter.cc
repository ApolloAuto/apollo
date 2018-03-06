/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/camera/common/cnn_adapter.h"

#include "modules/perception/obstacle/camera/common/caffe_bridge.hpp"

namespace apollo {
namespace perception {
namespace obstacle {

// CNNCaffe
bool CNNCaffe::init(const std::vector<std::string> &input_names,
                    const std::vector<std::string> &output_names,
                    const std::string &proto_file,
                    const std::string &weight_file, int gpu_id,
                    const std::string &model_root) {
  if (gpu_id >= 0) {
    caffe::Caffe::SetDevice(gpu_id);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::DeviceQuery();
  } else {
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
  }

  // init Net
  net_.reset(new caffe::Net<float>(proto_file, caffe::TEST));
  CHECK((net_ != nullptr));
  net_->CopyTrainedLayersFrom(weight_file);
  gpu_id_ = gpu_id;
  return true;
}

void CNNCaffe::forward() { net_->Forward(); }

boost::shared_ptr<caffe::Blob<float>> CNNCaffe::get_blob_by_name(
    const std::string &name) {
  return net_->blob_by_name(name);
}
// CNNCaffe END

/*
// CNNTensorRT
bool CNNTensorRT::init(const std::vector<std::string> &input_names,
                       const std::vector<std::string> &output_names,
                       const std::string &proto_file,
                       const std::string &weight_file, int gpu_id,
                       const std::string &model_root) {
  // input_names_ = input_names;
  // output_names_ = output_names;
  // init GPU
  gpu_id_ = gpu_id;

  if (gpu_id >= 0) {
    CUDA_CHECK(cudaSetDevice(gpu_id));
  } else {
    AINFO << "must use gpu mode";
    return false;
  }
  anakin::BatchStream calibrationStream;
  nvinfer1::Int8EntropyCalibrator calibrator(calibrationStream, 0, true,
                                             model_root);
  if (_int8_flag) {
    AINFO << model_root;
    inference_.initNet(proto_file.c_str(), weight_file.c_str(), &calibrator);
  } else {
    inference_.initNet(proto_file.c_str(), weight_file.c_str(), nullptr);
  }
  const std::vector<anakin::Tensor<float> *> &input_tensors =
      inference_.getInputTensors();
  const std::vector<anakin::Tensor<float> *> &output_tensors =
      inference_.getOutputTensors();
  //  CHECK(input_tensors.size() == input_names.size()) << "number of input
  //  tensor and name should be equal!"; CHECK(output_tensors.size() ==
  //  output_names.size()) << "number of output tensor and name should be
  //  equal!";

  inference_.execute();
  std::vector<void *> buffers = inference_.getBuffers();
  int cnt = 0;
  for (int i = 0; i < input_tensors.size(); i++, cnt++) {
    boost::shared_ptr<caffe::Blob<float>> blob;
    blob.reset(new caffe::Blob<float>);
    blob->Reshape(input_tensors[i]->dims());

    blobs_.insert(std::make_pair(input_names[i], blob));
    name_buffers_.insert(std::make_pair(input_names[i], buffers[cnt]));
    input_names_.push_back(input_names[i]);
  }
  for (int i = 0; i < output_tensors.size(); i++, cnt++) {
    boost::shared_ptr<caffe::Blob<float>> blob;
    blob.reset(new caffe::Blob<float>);
    blob->Reshape(output_tensors[i]->dims());

    name_buffers_.insert(std::make_pair(output_names[i], buffers[cnt]));
    blobs_.insert(std::make_pair(output_names[i], blob));
    output_names_.push_back(output_names[i]);
  }

  for (auto tmp : input_names) {
    AINFO << tmp;
  }
  for (auto tmp : output_names) {
    AINFO << tmp;
  }
  return true;
}

void CNNTensorRT::forward() {
  CUDA_CHECK(cudaSetDevice(gpu_id_));
  sync_blob(input_names_, false);
  inference_.execute();
  cudaDeviceSynchronize();
  sync_blob(output_names_, true);
}

boost::shared_ptr<caffe::Blob<float>> CNNTensorRT::get_blob_by_name(
    const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}
// CNNTensorRT END
*/

}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
