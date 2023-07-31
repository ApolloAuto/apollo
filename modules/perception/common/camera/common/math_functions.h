/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include "modules/perception/common/base/object_types.h"

namespace apollo {
namespace perception {
namespace camera {

template <typename Dtype>
inline Dtype sigmoid(Dtype x) {
  return static_cast<Dtype>(1.0) /
         static_cast<Dtype>(1.0 + exp(static_cast<double>(-x)));
}
template <typename Dtype>
inline Dtype gaussian(Dtype x, Dtype mu, Dtype sigma) {
  return static_cast<Dtype>(
      std::exp(-(x - mu) * (x - mu) / (2 * sigma * sigma)));
}
template <typename Dtype>
Dtype sqr(Dtype x) {
  return x * x;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
