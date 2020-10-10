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

#pragma once

#include <memory>
#include <vector>

#include "modules/perception/base/blob.h"

namespace apollo {
namespace perception {
namespace inference {

template <typename Dtype>
class Layer {
 public:
  virtual void ForwardGPU(
      const std::vector<std::shared_ptr<base::Blob<Dtype>>> &bottom,
      const std::vector<std::shared_ptr<base::Blob<Dtype>>> &top) = 0;
  virtual void ForwardCPU(
      const std::vector<std::shared_ptr<base::Blob<Dtype>>> &bottom,
      const std::vector<std::shared_ptr<base::Blob<Dtype>>> &top) = 0;
  Layer() = default;
  virtual ~Layer() = default;
  void set_gpu_id(const int &gpu_id) { gpu_id_ = gpu_id; }

 protected:
  int gpu_id_ = 0;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
