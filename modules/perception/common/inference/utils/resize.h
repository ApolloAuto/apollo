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

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/image.h"

namespace apollo {
namespace perception {
namespace inference {

bool ResizeGPU(const base::Image8U &src,
               std::shared_ptr<apollo::perception::base::Blob<float>> dst,
               int stepwidth, int start_axis);

bool ResizeGPU(const apollo::perception::base::Blob<uint8_t> &src_gpu,
               std::shared_ptr<apollo::perception::base::Blob<float>> dst,
               int stepwidth, int start_axis, int mean_b, int mean_g,
               int mean_r, bool channel_axis, float scale);

bool ResizeGPU(const base::Image8U &src,
               std::shared_ptr<apollo::perception::base::Blob<float>> dst,
               int stepwidth, int start_axis, float mean_b, float mean_g,
               float mean_r, bool channel_axis, float scale);

bool ImageZeroPadding(const base::Image8U &src, base::Image8U *dst,
                      int stepwidth, int left_pad, int right_pad, int top_pad,
                      int bottom_pad, int value, cudaStream_t stream,
                      bool same_order);
}  // namespace inference
}  // namespace perception
}  // namespace apollo
