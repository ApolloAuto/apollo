/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/traffic_light/rectify/cropbox.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/perception/traffic_light/base/utils.h"

namespace apollo {
namespace perception {
namespace traffic_light {

void CropBox::GetCropBox(const cv::Size &size,
                         const std::vector<LightPtr> &lights,
                         cv::Rect *cropbox) {
  int lights_num = lights.size();
  if (lights_num == 0) {
    AINFO << "No valid HD-map corrdinates info";
    ClearBox(cropbox);
    return;
  }
  int rows = size.height;
  int cols = size.width;
  float xr = 0;
  float yb = 0;
  float xl = cols - 1;
  float yt = rows - 1;
  bool initialized = false;
  // min max: for all hdmap boxes
  for (int i = 0; i < lights_num; i++) {
    Light &light = *lights[i];
    if (!BoxIsValid(light.region.projection_roi, size)) {
      continue;
    }
    initialized = true;
    if (xl > light.region.projection_roi.x) {
      xl = light.region.projection_roi.x;
    }
    if (yt > light.region.projection_roi.y) {
      yt = light.region.projection_roi.y;
    }
    if (xr <
        light.region.projection_roi.x + light.region.projection_roi.width) {
      xr = light.region.projection_roi.x + light.region.projection_roi.width;
    }
    if (yb <
        light.region.projection_roi.y + light.region.projection_roi.height) {
      yb = light.region.projection_roi.y + light.region.projection_roi.height;
    }
  }
  if (!initialized) {
    ClearBox(cropbox);
    return;
  }
  // scale
  float center_x = (xr + xl) / 2;
  float center_y = (yb + yt) / 2;
  float resize_width = (xr - xl) * crop_scale_;
  float resize_height = (yb - yt) * crop_scale_;
  float resize = std::max(resize_width, resize_height);
  resize_width = resize_height =
      (resize < min_crop_size_) ? min_crop_size_ : resize;

  // float pad_t = (resize_height - (yb - yt)) / 2;
  // float pad_l = (resize_width - (xr - xl)) / 2;
  // float pad_b = pad_t;
  // float pad_r = pad_l;
  // clamp
  xl = center_x - resize_width / 2;
  xl = (xl < 0) ? 0 : xl;
  yt = center_y - resize_height / 2;
  yt = (yt < 0) ? 0 : yt;
  xr = center_x + resize_width / 2;
  xr = (xr >= cols) ? cols - 1 : xr;
  yb = center_y + resize_width / 2;
  yb = (yb >= rows) ? rows - 1 : yb;

  cropbox->x = static_cast<int>(xl);
  cropbox->y = static_cast<int>(yt);
  cropbox->width = static_cast<int>(xr - xl);
  cropbox->height = static_cast<int>(yb - yt);
}
void CropBox::Init(float crop_scale, float min_crop_size) {
  crop_scale_ = crop_scale;
  min_crop_size_ = min_crop_size;
}
CropBox::CropBox(float crop_scale, float min_crop_size) {
  Init(crop_scale, min_crop_size);
}
void CropBoxWholeImage::GetCropBox(const cv::Size &size,
                                   const std::vector<LightPtr> &lights,
                                   cv::Rect *cropbox) {
  for (size_t i = 0; i < lights.size(); ++i) {
    if (BoxIsValid(lights[i]->region.projection_roi, size)) {
      cropbox->x = cropbox->y = 0;
      cropbox->width = size.width;
      cropbox->height = size.height;
      return;
    }
  }
  ClearBox(cropbox);
}
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
