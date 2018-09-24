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
#include "modules/perception/base/blob.h"
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
  memset(top_data, -1, FLT_MAX * sizeof(Dtype));
  int *argmax_data = max_idx_.mutable_cpu_data();
  memset(argmax_data, -1, top_count * sizeof(int));
  // For each ROI R = [batch_index x1 y1 x2 y2]: max pool over R
  for (int n = 0; n < num_rois; ++n) {
    int roi_batch_ind = bottom_rois[0];
    int roi_start_w = 0;
    int roi_start_h = 0;
    int roi_end_w = 0;
    int roi_end_h = 0;
    if (use_floor_) {
      roi_start_w = floor(bottom_rois[1] * spatial_scale_);
      roi_start_h = floor(bottom_rois[2] * spatial_scale_);
      roi_end_w = floor(bottom_rois[3] * spatial_scale_);
      roi_end_h = floor(bottom_rois[4] * spatial_scale_);
    } else {
      roi_start_w = round(bottom_rois[1] * spatial_scale_);
      roi_start_h = round(bottom_rois[2] * spatial_scale_);
      roi_end_w = round(bottom_rois[3] * spatial_scale_);
      roi_end_h = round(bottom_rois[4] * spatial_scale_);
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
