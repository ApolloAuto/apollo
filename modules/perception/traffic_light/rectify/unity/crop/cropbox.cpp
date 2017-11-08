//
// Created by gaohan02 on 16-8-1.
//

#include <xlog.h>
#include "module/perception/traffic_light/rectify/unity/crop/cropbox.h"
#include "module/perception/traffic_light/base/utils.h"

namespace adu {
namespace perception {
namespace traffic_light {

void
CropBox::get_crop_box(const cv::Size &size, const std::vector<LightPtr> &lights,
                      cv::Rect *cropbox) {
  int lights_num = lights.size();
  if (lights_num == 0) {
    AINFO << "No valid HD-map corrdinates info";
    clear_box(*cropbox);
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
    if (!box_is_valid(light.region.projection_roi, size)) {
      continue;
    }
    initialized = true;
    if (xl > light.region.projection_roi.x) {
      xl = light.region.projection_roi.x;
    }
    if (yt > light.region.projection_roi.y) {
      yt = light.region.projection_roi.y;
    }
    if (xr < light.region.projection_roi.x + light.region.projection_roi.width) {
      xr = light.region.projection_roi.x + light.region.projection_roi.width;
    }
    if (yb < light.region.projection_roi.y + light.region.projection_roi.height) {
      yb = light.region.projection_roi.y + light.region.projection_roi.height;
    }
  }
  if (!initialized) {
    clear_box(*cropbox);
    return;
  }
  // scale
  float center_x = (xr + xl) / 2;
  float center_y = (yb + yt) / 2;
  float resize_width = (xr - xl) * _crop_scale;
  float resize_height = (yb - yt) * _crop_scale;
  float resize = std::max(resize_width, resize_height);
  resize_width = resize_height = (resize < _min_crop_size) ? _min_crop_size : resize;

  float pad_t = (resize_height - (yb - yt)) / 2;
  float pad_l = (resize_width - (xr - xl)) / 2;
  float pad_b = pad_t;
  float pad_r = pad_l;
  // clamp
  xl = center_x - resize_width / 2;
  xl = (xl < 0) ? 0 : xl;
  yt = center_y - resize_height / 2;
  yt = (yt < 0) ? 0 : yt;
  xr = center_x + resize_width / 2;
  xr = (xr >= cols) ? cols - 1 : xr;
  yb = center_y + resize_width / 2;
  yb = (yb >= rows) ? rows - 1 : yb;

  cropbox->x = (int) xl;
  cropbox->y = (int) yt;
  cropbox->width = (int) (xr - xl);
  cropbox->height = (int) (yb - yt);
}
void CropBox::init(float crop_scale, float min_crop_size) {
  _crop_scale = crop_scale;
  _min_crop_size = min_crop_size;
}
CropBox::CropBox(float crop_scale, float min_crop_size) {
  init(crop_scale, min_crop_size);
}
void CropBoxWholeImage::get_crop_box(const cv::Size &size, const std::vector<LightPtr> &lights,
                                     cv::Rect *cropbox) {
  for (int i = 0; i < lights.size(); ++i) {
    if (box_is_valid(lights[i]->region.projection_roi, size)) {
      cropbox->x = cropbox->y = 0;
      cropbox->width = size.width;
      cropbox->height = size.height;
      return;
    }
  }
  clear_box(*cropbox);
}
}
}
}