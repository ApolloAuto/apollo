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

#include <string>
#include <vector>

#include "caffe/caffe.hpp"

#include "modules/perception/cuda_util/util.h"

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

  bool init(const std::vector<std::string> &input_names,
            const std::vector<std::string> &output_names,
            const std::string &proto_file, const std::string &weight_file,
            int gpu_id, const std::string &model_root = "") override;

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

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_CNN_ADAPTER_H_
