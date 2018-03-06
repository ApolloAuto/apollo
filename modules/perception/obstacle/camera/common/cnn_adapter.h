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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CNN_ADAPTER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CNN_ADAPTER_H_

#include "caffe/caffe.hpp"

#include "modules/perception/obstacle/camera/common/caffe_bridge.hpp"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/detector/common/util.h"

namespace apollo {
namespace perception {

class CNNAdapter {
 public:
  virtual void forward() = 0;

  virtual boost::shared_ptr<caffe::Blob<float>> get_blob_by_name(
      const std::string &name) = 0;

  virtual bool init(const std::vector<std::string> &input_names,
                    const std::vector<std::string> &output_names,
                    const std::string &proto_file,
                    const std::string &weight_file, int gpu_id,
                    const std::string &model_root = "") = 0;

  virtual bool shape(const std::string &name, std::vector<int> *res) = 0;
  virtual bool reshape_input(const std::string &name,
                             const std::vector<int> &shape) = 0;
};

class CNNCaffe : public CNNAdapter {
 public:
  CNNCaffe() = default;

  virtual bool init(const std::vector<std::string> &input_names,
                    const std::vector<std::string> &output_names,
                    const std::string &proto_file,
                    const std::string &weight_file, int gpu_id,
                    const std::string &model_root = "") override;

  void forward() override;

  boost::shared_ptr<caffe::Blob<float>> get_blob_by_name(
      const std::string &name) override;

  bool shape(const std::string &name, std::vector<int> *res) override {
    auto blob = get_blob_by_name(name);
    if (blob == nullptr) {
      return false;
    }
    *res = blob->shape();
    return true;
  }

  bool reshape_input(const std::string &name,
                     const std::vector<int> &shape) override {
    auto blob = get_blob_by_name(name);
    if (blob == nullptr) {
      return false;
    }
    blob->Reshape(shape);
    forward();
    return true;
  }

 private:
  boost::shared_ptr<caffe::Net<float>> net_;
  int gpu_id_ = 0;
};

class CNNAnakin : public CNNAdapter {
 public:
  CNNAnakin() = default;

  virtual bool init(const std::vector<std::string> &input_names,
                    const std::vector<std::string> &output_names,
                    const std::string &proto_file,
                    const std::string &weight_file, int gpu_id,
                    const std::string &model_root = "") override;

  void forward() override;

  boost::shared_ptr<caffe::Blob<float>> get_blob_by_name(
      const std::string &name) override;

  bool shape(const std::string &name, std::vector<int> *res) override {
    auto blob = get_blob_by_name(name);
    if (blob == nullptr) {
      return false;
    }
    *res = blob->shape();
    return true;
  }

  bool reshape_input(const std::string &name,
                     const std::vector<int> &shape) override {
    inference_.SetDevice(gpu_id_);
    anakin::Net<float> *net =
        static_cast<anakin::Net<float> *>(inference_.getNetwork());
    int layer_idx = net->getLayerIdx("input");
    if (layer_idx == -1) {
      return false;
    }
    auto layer =
        static_cast<anakin::InputLayer<float> *>(net->getLayer(layer_idx));

    auto blob_iter = blobs_.find(name);
    if (blob_iter == blobs_.end()) {
      return false;
    }

    auto &dims = layer->get_input_dim();
    dims.resize(shape.size());
    dims.assign(shape.begin(), shape.end());
    blob_iter->second->Reshape(shape);

    cudaDeviceSynchronize();
    inference_.execute();
    cudaDeviceSynchronize();

    for (const auto &name : output_names_) {
      if (!sync_blob(name, true)) {
        return false;
      }
    }

    return true;
  }

 protected:
  bool sync_blob(const string &name, bool tensor_to_blob_flag) {
    anakin::Net<float> *net =
        static_cast<anakin::Net<float> *>(inference_.getNetwork());
    auto blob_iter = blobs_.find(name);
    int tensor_idx = net->getTensorIdx(name);
    if (tensor_idx == -1 || blob_iter == blobs_.end()) {
      AINFO << "cannot find " << name << " tensor ";
      return true;
    }
    auto tensor = net->getTensor(tensor_idx);
    if (tensor_to_blob_flag) {
      blob_iter->second->Reshape(tensor->dims());
      cudaMemcpy(blob_iter->second->mutable_gpu_data(),
                 (float *)tensor->gpu_data(), tensor->count() * sizeof(float),
                 cudaMemcpyDeviceToDevice);
    } else {
      tensor->Reshape(blob_iter->second->shape());
      cudaMemcpy(tensor->gpu_mutable_data(),
                 (float *)blob_iter->second->gpu_data(),
                 tensor->count() * sizeof(float), cudaMemcpyDeviceToDevice);
    }

    return true;
  }
  bool create_blob(const std::string &name) {
    anakin::Net<float> *net;
    net = static_cast<anakin::Net<float> *>(inference_.getNetwork());
    int tensor_idx = net->getTensorIdx(name);
    if (tensor_idx == -1) {
      return false;
    }
    auto tensor = net->getTensor(tensor_idx);

    boost::shared_ptr<caffe::Blob<float>> blob;
    blob.reset(new caffe::Blob<float>);

    // TODO: unit test not covered
    blob->Reshape(tensor->dims());
    blobs_.insert(std::make_pair(name, blob));
    return true;
  }
  void sync_blob(const std::vector<anakin::Tensor<float> *> &v_tensors,
                 bool tensor_to_blob_flag) {
    for (auto tensor : v_tensors) {
      std::string name = tensor->getName();
      auto blob = blobs_.find(name);
      if (blob != blobs_.end()) {
        CHECK(blob->second->count() == tensor->count())
            << "Blob and Tensor should have the save count(" << name
            << "): " << blob->second->count() << " V.S. " << tensor->count();
        if (tensor_to_blob_flag) {
          cudaMemcpy(blob->second->mutable_gpu_data(),
                     (float *)tensor->gpu_data(),
                     tensor->count() * sizeof(float), cudaMemcpyDeviceToDevice);
        } else {
          cudaMemcpy(tensor->gpu_mutable_data(),
                     (float *)blob->second->gpu_data(),
                     tensor->count() * sizeof(float), cudaMemcpyDeviceToDevice);
        }
      }
    }
  }

 private:
  ::anakin::Infer<float> inference_;
  std::map<std::string, boost::shared_ptr<caffe::Blob<float>>> blobs_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  int gpu_id_ = 0;
};

class CNNTensorRT : public CNNAdapter {
 public:
  CNNTensorRT(bool int8_flag) : _int8_flag(int8_flag){};

  virtual bool init(const std::vector<std::string> &input_names,
                    const std::vector<std::string> &output_names,
                    const std::string &proto_file,
                    const std::string &weight_file, int gpu_id,
                    const std::string &model_root = "") override;

  void forward() override;

  boost::shared_ptr<caffe::Blob<float>> get_blob_by_name(
      const std::string &name) override;

  bool shape(const std::string &name, std::vector<int> *res) override {
    auto blob = get_blob_by_name(name);
    if (blob == nullptr) {
      return false;
    }
    *res = blob->shape();
    return true;
  }

  bool reshape_input(const std::string &name,
                     const std::vector<int> &shape) override {
    return true;
  }

 protected:
  void sync_blob(const std::vector<std::string> &v_names,
                 bool buffer_to_blob_flag) {
    for (auto name : v_names) {
      auto blob = get_blob_by_name(name);
      int count = blob->count();
      if (buffer_to_blob_flag) {
        float *gpu_data = blob->mutable_gpu_data();
        cudaMemcpy(gpu_data, (float *)name_buffers_[name],
                   count * sizeof(float), cudaMemcpyDeviceToDevice);
      } else {
        const float *gpu_data = blob->gpu_data();
        cudaMemcpy((float *)name_buffers_[name], gpu_data,
                   count * sizeof(float), cudaMemcpyDeviceToDevice);
      }
    }
  }

 private:
  ::anakin::RTInfer<float> inference_;
  // name  blob //
  std::map<std::string, boost::shared_ptr<caffe::Blob<float>>> blobs_;
  std::map<std::string, void *> name_buffers_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  int gpu_id_ = 0;
  bool _int8_flag = false;
};
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CNN_ADAPTER_H_
