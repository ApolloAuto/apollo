/******************************************************************************
COPYRIGHT

All contributions by the University of California:
Copyright (c) 2014-2017 The Regents of the University of California (Regents)
All rights reserved.

All other contributions:
Copyright (c) 2014-2017, the respective contributors
All rights reserved.

Caffe uses a shared copyright model: each contributor holds copyright over
their contributions to Caffe. The project versioning records all such
contribution and copyright details. If a contributor wants to further mark
their specific copyright on a particular contribution, they should indicate
their copyright solely in the commit message of the change when it is
committed.

LICENSE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

CONTRIBUTION AGREEMENT

By contributing to the BVLC/caffe repository through pull-request, comment,
or otherwise, the contributor releases their content to the
license and copyright terms herein.
 *****************************************************************************/

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

#include <limits>
#include <memory>
#include <vector>

#include "modules/perception/inference/layer.h"

namespace apollo {
namespace perception {
namespace inference {

template <typename Dtype>
class ROIPoolingLayer : public Layer<Dtype> {
 public:
  ROIPoolingLayer(int pooled_h, int pooled_w, bool use_floor,
                  float spatial_scale, int channels, int max_objs = 1000)
      : channels_(0),
        height_(0),
        width_(0),
        pooled_height_(pooled_h),
        pooled_width_(pooled_w),
        use_floor_(use_floor),
        float_max_(std::numeric_limits<float>::max()),
        spatial_scale_(spatial_scale) {
    max_idx_.Reshape(max_objs, channels, pooled_height_, pooled_width_);
  }
  void ForwardGPU(const std::vector<std::shared_ptr<base::Blob<Dtype>>> &bottom,
                  const std::vector<std::shared_ptr<base::Blob<Dtype>>> &top);
  void ForwardCPU(const std::vector<std::shared_ptr<base::Blob<Dtype>>> &bottom,
                  const std::vector<std::shared_ptr<base::Blob<Dtype>>> &top);

 private:
  base::Blob<int> max_idx_;
  int channels_;
  int height_;
  int width_;
  int pooled_height_;
  int pooled_width_;
  bool use_floor_;
  const float float_max_;
  float spatial_scale_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
