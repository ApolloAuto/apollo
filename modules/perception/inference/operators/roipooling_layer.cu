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
#include "modules/perception/inference/operators/roipooling_layer.h"

#include <algorithm>
#include <vector>
#include "modules/perception/base/blob.h"

namespace apollo {
namespace perception {
namespace inference {
template<typename Dtype>
__global__ void ROIPoolForward(const int nthreads,
                               const Dtype *bottom_data,
                               const bool use_floor,
                               const Dtype spatial_scale,
                               const int channels,
                               const int height,
                               const int width,
                               const int pooled_height,
                               const int pooled_width,
                               const Dtype *bottom_rois,
                               Dtype *top_data,
                               int *argmax_data) {
  for (int index = blockIdx.x * blockDim.x + threadIdx.x;
       index < (nthreads);
       index += blockDim.x * gridDim.x) {
    // (n, c, ph, pw) is an element in the pooled output
    int pw = index % pooled_width;
    int ph = (index / pooled_width) % pooled_height;
    int c = (index / pooled_width / pooled_height) % channels;
    int n = index / pooled_width / pooled_height / channels;

    bottom_rois += n * 5;
    int roi_batch_ind = bottom_rois[0];

    int roi_start_w = 0;
    int roi_start_h = 0;
    int roi_end_w = 0;
    int roi_end_h = 0;
    if (use_floor) {
      roi_start_w = floor(bottom_rois[1] * spatial_scale);
      roi_start_h = floor(bottom_rois[2] * spatial_scale);
      roi_end_w = floor(bottom_rois[3] * spatial_scale);
      roi_end_h = floor(bottom_rois[4] * spatial_scale);
    } else {
      roi_start_w = round(bottom_rois[1] * spatial_scale);
      roi_start_h = round(bottom_rois[2] * spatial_scale);
      roi_end_w = round(bottom_rois[3] * spatial_scale);
      roi_end_h = round(bottom_rois[4] * spatial_scale);
    }

    // Force malformed ROIs to be 1x1
    int roi_width = max(roi_end_w - roi_start_w + 1, 1);
    int roi_height = max(roi_end_h - roi_start_h + 1, 1);
    Dtype bin_size_h = static_cast<Dtype>(roi_height)
        / static_cast<Dtype>(pooled_height);
    Dtype bin_size_w = static_cast<Dtype>(roi_width)
        / static_cast<Dtype>(pooled_width);

    int hstart = static_cast<int>(floor(static_cast<Dtype>(ph)
                                            * bin_size_h));
    int wstart = static_cast<int>(floor(static_cast<Dtype>(pw)
                                            * bin_size_w));
    int hend = static_cast<int>(ceil(static_cast<Dtype>(ph + 1)
                                         * bin_size_h));
    int wend = static_cast<int>(ceil(static_cast<Dtype>(pw + 1)
                                         * bin_size_w));

    // Add roi offsets and clip to input boundaries
    hstart = min(max(hstart + roi_start_h, 0), height);
    hend = min(max(hend + roi_start_h, 0), height);
    wstart = min(max(wstart + roi_start_w, 0), width);
    wend = min(max(wend + roi_start_w, 0), width);
    bool is_empty = (hend <= hstart) || (wend <= wstart);

    // Define an empty pooling region to be zero
    Dtype maxval = is_empty ? 0 : -FLT_MAX;
    // If nothing is pooled, argmax = -1 causes nothing to be backprop'd
    int maxidx = -1;
    bottom_data += (roi_batch_ind * channels + c) * height * width;
    for (int h = hstart; h < hend; ++h) {
      for (int w = wstart; w < wend; ++w) {
        int bottom_index = h * width + w;
        if (bottom_data[bottom_index] > maxval) {
          maxval = bottom_data[bottom_index];
          maxidx = bottom_index;
        }
      }
    }
    top_data[index] = maxval;
    argmax_data[index] = maxidx;
  }
}
template<typename Dtype>
void ROIPoolingLayer<Dtype>::ForwardGPU(const std::vector<std::shared_ptr<
                                            base::Blob<Dtype>>> &bottom,
                                        const std::vector<std::shared_ptr<
                                            base::Blob<Dtype>>> &top) {
  auto feat_b = bottom[0];
  auto roi_b = bottom[1];
  channels_ = feat_b->channels();
  height_ = feat_b->height();
  width_ = feat_b->width();

  top[0]->Reshape(roi_b->num(), channels_, pooled_height_, pooled_width_);
  max_idx_.Reshape(roi_b->num(), channels_, pooled_height_, pooled_width_);

  const Dtype *bottom_data = feat_b->gpu_data();
  const Dtype *bottom_rois = roi_b->gpu_data();
  Dtype *top_data = top[0]->mutable_gpu_data();
  int *argmax_data = max_idx_.mutable_gpu_data();
  int count = top[0]->count();
  const int thread_size = 512;
  int block_size = (count + thread_size - 1) / thread_size;
  ROIPoolForward<Dtype> << < block_size, thread_size >> > (
      count, bottom_data, use_floor_, spatial_scale_, channels_, height_,
          width_,
          pooled_height_, pooled_width_, bottom_rois, top_data, argmax_data);
}
template void ROIPoolingLayer<double>::ForwardGPU( \
      const std::vector<std::shared_ptr<base::Blob<double>>> & bottom, \
      const std::vector<std::shared_ptr<base::Blob<double>>> & top);
template void ROIPoolingLayer<float>::ForwardGPU( \
      const std::vector<std::shared_ptr<base::Blob<float>>> & bottom, \
      const std::vector<std::shared_ptr<base::Blob<float>>> & top);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
