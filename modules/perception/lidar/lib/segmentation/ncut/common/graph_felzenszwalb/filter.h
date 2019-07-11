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
/* simple filters */
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "modules/perception/lidar/segmentation/ncut/common/graph_felzenszwalb/convolve.h"
#include "modules/perception/lidar/segmentation/ncut/common/graph_felzenszwalb/image.h"
#include "modules/perception/lidar/segmentation/ncut/common/graph_felzenszwalb/imconv.h"
#include "modules/perception/lidar/segmentation/ncut/common/graph_felzenszwalb/misc.h"

namespace apollo {
namespace perception {
namespace lidar {
const double WIDTH = 4.0;
/* normalize mask so it integrates to one */
void normalize(std::vector<float> *mask_input) {
  std::vector<float> &mask = *mask_input;
  int len = mask.size();
  float sum = 0;
  for (int i = 1; i < len; i++) {
    sum += fabs(mask[i]);
  }
  sum = 2 * sum + fabs(mask[0]);
  for (int i = 0; i < len; i++) {
    mask[i] /= sum;
  }
}
/* make filters */
#define MAKE_FILTER(name, fun)                           \
  std::vector<float> make_##name(float sigma) {          \
    sigma = std::max(sigma, 0.01F);                      \
    int len = static_cast<int>(ceil(sigma * WIDTH)) + 1; \
    std::vector<float> mask(len);                        \
    for (int i = 0; i < len; i++) {                      \
      mask[i] = fun;                                     \
    }                                                    \
    return mask;                                         \
  }
MAKE_FILTER(fgauss, exp(-0.5 * square(i / sigma)));
/* convolve image with gaussian filter */
Image<float> *smooth(Image<float> *src, float sigma) {
  std::vector<float> mask = make_fgauss(sigma);
  normalize(mask);
  Image<float> *tmp = new Image<float>(src->height(), src->width(), false);
  Image<float> *dst = new Image<float>(src->width(), src->height(), false);
  convolve_even(src, tmp, mask);
  convolve_even(tmp, dst, mask);
  delete tmp;
  return dst;
}
/* convolve image with gaussian filter */
Image<float> *smooth(Image<uchar> *src, float sigma) {
  Image<float> *tmp = image_uchar2float(src);
  Image<float> *dst = smooth(tmp, sigma);
  delete tmp;
  return dst;
}
/* compute laplacian */
Image<float> *laplacian(Image<float> *src) {
  int width = src->width();
  int height = src->height();
  Image<float> *dst = new Image<float>(width, height);
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      float d2x =
          imRef(src, x - 1, y) + imRef(src, x + 1, y) - 2 * imRef(src, x, y);
      float d2y =
          imRef(src, x, y - 1) + imRef(src, x, y + 1) - 2 * imRef(src, x, y);
      imRef(dst, x, y) = d2x + d2y;
    }
  }
  return dst;
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
