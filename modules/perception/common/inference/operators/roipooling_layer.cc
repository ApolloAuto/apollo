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

#include "modules/perception/common/inference/operators/roipooling_layer.h"

#include <algorithm>

#include "modules/perception/common/base/blob.h"

namespace apollo {
namespace perception {
namespace inference {

template <typename Dtype>
void ROIPoolingLayer<Dtype>::ForwardCPU(
    const std::vector<std::shared_ptr<base::Blob<Dtype>>> &bottom,
    const std::vector<std::shared_ptr<base::Blob<Dtype>>> &top) {
  channels_ = bottom[0]->channels();
  height_ = bottom[0]->height();
  width_ = bottom[0]->width();
  top[0]->Reshape(bottom[1]->num(), channels_, pooled_height_, pooled_width_);
  auto feat_bottom = bottom[0];
  auto roi_bottom = bottom[1];
  const Dtype *bottom_data = feat_bottom->cpu_data();
  const Dtype *bottom_rois = roi_bottom->cpu_data();
  // Number of ROIs
  int num_rois = roi_bottom->num();
  int batch_size = feat_bottom->num();
  int top_count = top[0]->count();
  Dtype *top_data = top[0]->mutable_cpu_data();
  memset(top_data, -1, static_cast<size_t>(float_max_ * sizeof(Dtype)));
  int *argmax_data = max_idx_.mutable_cpu_data();
  memset(argmax_data, -1, top_count * sizeof(int));
  // For each ROI R = [batch_index x1 y1 x2 y2]: max pool over R
  for (int n = 0; n < num_rois; ++n) {
    int roi_batch_ind = static_cast<int>(bottom_rois[0]);
    int roi_start_w = 0;
    int roi_start_h = 0;
    int roi_end_w = 0;
    int roi_end_h = 0;
    if (use_floor_) {
      roi_start_w = static_cast<int>(floor(bottom_rois[1] * spatial_scale_));
      roi_start_h = static_cast<int>(floor(bottom_rois[2] * spatial_scale_));
      roi_end_w = static_cast<int>(floor(bottom_rois[3] * spatial_scale_));
      roi_end_h = static_cast<int>(floor(bottom_rois[4] * spatial_scale_));
    } else {
      roi_start_w = static_cast<int>(round(bottom_rois[1] * spatial_scale_));
      roi_start_h = static_cast<int>(round(bottom_rois[2] * spatial_scale_));
      roi_end_w = static_cast<int>(round(bottom_rois[3] * spatial_scale_));
      roi_end_h = static_cast<int>(round(bottom_rois[4] * spatial_scale_));
    }
    CHECK_GE(roi_batch_ind, 0);
    CHECK_LT(roi_batch_ind, batch_size);

    int roi_height = std::max(roi_end_h - roi_start_h + 1, 1);
    int roi_width = std::max(roi_end_w - roi_start_w + 1, 1);
    const Dtype bin_size_h =
        static_cast<Dtype>(roi_height) / static_cast<Dtype>(pooled_height_);
    const Dtype bin_size_w =
        static_cast<Dtype>(roi_width) / static_cast<Dtype>(pooled_width_);

    const Dtype *batch_data = bottom_data + bottom[0]->offset(roi_batch_ind);

    for (int c = 0; c < channels_; ++c) {
      for (int ph = 0; ph < pooled_height_; ++ph) {
        for (int pw = 0; pw < pooled_width_; ++pw) {
          // Compute pooling region for this output unit:
          int hstart =
              static_cast<int>(floor(static_cast<Dtype>(ph) * bin_size_h));
          int wstart =
              static_cast<int>(floor(static_cast<Dtype>(pw) * bin_size_w));
          int hend =
              static_cast<int>(ceil(static_cast<Dtype>(ph + 1) * bin_size_h));
          int wend =
              static_cast<int>(ceil(static_cast<Dtype>(pw + 1) * bin_size_w));

          hstart = std::min(std::max(hstart + roi_start_h, 0), height_);
          hend = std::min(std::max(hend + roi_start_h, 0), height_);
          wstart = std::min(std::max(wstart + roi_start_w, 0), width_);
          wend = std::min(std::max(wend + roi_start_w, 0), width_);

          bool is_empty = (hend <= hstart) || (wend <= wstart);
          const int pool_index = ph * pooled_width_ + pw;
          if (is_empty) {
            top_data[pool_index] = 0;
            argmax_data[pool_index] = -1;
          }

          for (int h = hstart; h < hend; ++h) {
            for (int w = wstart; w < wend; ++w) {
              const int index = h * width_ + w;
              if (batch_data[index] > top_data[pool_index]) {
                top_data[pool_index] = batch_data[index];
                argmax_data[pool_index] = index;
              }
            }
          }
        }
      }
      // Increment all data pointers by one channel
      batch_data += feat_bottom->offset(0, 1);
      top_data += top[0]->offset(0, 1);
      argmax_data += max_idx_.offset(0, 1);
    }
    // Increment ROI data pointer
    bottom_rois += roi_bottom->offset(1);
  }
}
template class ROIPoolingLayer<float>;
template class ROIPoolingLayer<double>;

}  // namespace inference
}  // namespace perception
}  // namespace apollo
