/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/*
Copyright (C) 2006 Pedro Felzenszwalb
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/
/* image conversion */
#pragma once
#include <climits>
#include "modules/perception/lidar/segmentation/ncut/common/graph_felzenszwalb/image.h"
#include "modules/perception/lidar/segmentation/ncut/common/graph_felzenszwalb/imutil.h"
#include "modules/perception/lidar/segmentation/ncut/common/graph_felzenszwalb/misc.h"

namespace apollo {
namespace perception {
namespace lidar {
const double RED_WEIGHT = 0.299;
const double GREEN_WEIGHT = 0.587;
const double BLUE_WEIGHT = 0.114;
Image<uchar> *image_rgb2gray(Image<rgb> *input) {
  int width = input->width();
  int height = input->height();
  Image<uchar> *output = new Image<uchar>(width, height, false);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = (uchar)(imRef(input, x, y).r * RED_WEIGHT +
                                    imRef(input, x, y).g * GREEN_WEIGHT +
                                    imRef(input, x, y).b * BLUE_WEIGHT);
    }
  }
  return output;
}
Image<rgb> *image_gray2rgb(Image<uchar> *input) {
  int width = input->width();
  int height = input->height();
  Image<rgb> *output = new Image<rgb>(width, height, false);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y).r = imRef(input, x, y);
      imRef(output, x, y).g = imRef(input, x, y);
      imRef(output, x, y).b = imRef(input, x, y);
    }
  }
  return output;
}
Image<float> *image_uchar2float(Image<uchar> *input) {
  int width = input->width();
  int height = input->height();
  Image<float> *output = new Image<float>(width, height, false);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = imRef(input, x, y);
    }
  }
  return output;
}
Image<float> *image_int2float(Image<int> *input) {
  int width = input->width();
  int height = input->height();
  Image<float> *output = new Image<float>(width, height, false);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = imRef(input, x, y);
    }
  }
  return output;
}
Image<uchar> *image_float2uchar(Image<float> *input, float min, float max) {
  int width = input->width();
  int height = input->height();
  Image<uchar> *output = new Image<uchar>(width, height, false);
  if (max == min) {
    return output;
  }
  float scale = UCHAR_MAX / (max - min);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uchar val = (uchar)((imRef(input, x, y) - min) * scale);
      imRef(output, x, y) = bound(val, (uchar)0, (uchar)UCHAR_MAX);
    }
  }
  return output;
}
Image<uchar> *image_float2uchar(Image<float> *input) {
  float min, max;
  min_max(input, &min, &max);
  return image_float2uchar(input, min, max);
}
Image<uint32_t> *image_uchar2long(Image<uchar> *input) {
  int width = input->width();
  int height = input->height();
  Image<uint32_t> *output = new Image<uint32_t>(width, height, false);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(output, x, y) = imRef(input, x, y);
    }
  }
  return output;
}
Image<uchar> *image_long2uchar(Image<uint32_t> *input, uint32_t min,
                               uint32_t max) {
  int width = input->width();
  int height = input->height();
  Image<uchar> *output = new Image<uchar>(width, height, false);
  if (max == min) {
    return output;
  }
  float scale = UCHAR_MAX / static_cast<float>(max - min);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uchar val = (uchar)((imRef(input, x, y) - min) * scale);
      imRef(output, x, y) = bound(val, (uchar)0, (uchar)UCHAR_MAX);
    }
  }
  return output;
}
Image<uchar> *image_long2uchar(Image<uint32_t> *input) {
  uint32_t min, max;
  min_max(input, &min, &max);
  return image_long2uchar(input, min, max);
}
Image<uchar> *image_short2uchar(Image<uint16_t> *input, uint16_t min,
                                uint16_t max) {
  int width = input->width();
  int height = input->height();
  Image<uchar> *output = new Image<uchar>(width, height, false);
  if (max == min) {
    return output;
  }
  float scale = UCHAR_MAX / static_cast<float>(max - min);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      uchar val = (uchar)((imRef(input, x, y) - min) * scale);
      imRef(output, x, y) = bound(val, (uchar)0, (uchar)UCHAR_MAX);
    }
  }
  return output;
}
Image<uchar> *image_short2uchar(Image<uint16_t> *input) {
  uint16_t min, max;
  min_max(input, &min, &max);
  return image_short2uchar(input, min, max);
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
