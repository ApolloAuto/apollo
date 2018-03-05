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

#ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_COMMON_CAFFE_BRIDGE_HPP_
#define ADU_PERCEPTION_OBSTACLE_CAMERA_COMMON_CAFFE_BRIDGE_HPP_

#include <caffe/caffe.hpp>

#include "infer.h"

namespace apollo {
namespace perception {
namespace obstacle {

template <typename Dtype>
bool tensor_to_blob(const anakin::Tensor<Dtype> &tensor,
                    caffe::Blob<Dtype> *blob) {
  if (blob == nullptr) {
    return false;
  }
  blob->Reshape(tensor.dims());
  memcpy(blob->mutable_cpu_data(), tensor.cpu_data(),
         sizeof(Dtype) * blob->count());
  return true;
}

template bool tensor_to_blob(const anakin::Tensor<float> &tensor,
                             caffe::Blob<float> *blob);
#if 0
template bool tensor_to_blob(const anakin::Tensor<double> &tensor, caffe::Blob<double> *blob);
#endif
}  // namespace obstacle
}  // namespace perception
}  // namespace apollo

#endif  // ADU_PERCEPTION_OBSTACLE_CAMERA_COMMON_CAFFE_BRIDGE_HPP_
