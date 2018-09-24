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
#ifndef INFERENCE_OPERATORS_ROIPOOLING_LAYER_H_
#define INFERENCE_OPERATORS_ROIPOOLING_LAYER_H_
#include <float.h>
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
  float spatial_scale_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
#endif  // INFERENCE_OPERATORS_ROIPOOLING_LAYER_H_
