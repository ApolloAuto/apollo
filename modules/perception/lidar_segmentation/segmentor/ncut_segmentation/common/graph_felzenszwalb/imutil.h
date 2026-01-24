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
/* some image utilities */
#pragma once

#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/graph_felzenszwalb/image.h"
#include "modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common/graph_felzenszwalb/misc.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @brief Compute minimum and maximum value in an image
 *
 * @tparam T
 * @param im
 * @param ret_min
 * @param ret_max
 */
template <class T>
void min_max(Image<T> *im, T *ret_min, T *ret_max) {
    int width = im->width();
    int height = im->height();
    T min = imRef(im, 0, 0);
    T max = imRef(im, 0, 0);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            T val = imRef(im, x, y);
            if (min > val) {
                min = val;
            }
            if (max < val) {
                max = val;
            }
        }
    }
    *ret_min = min;
    *ret_max = max;
}

/**
 * @brief Threshold image
 *
 * @tparam T
 * @param src
 * @param t
 * @return Image<uchar>*
 */
template <class T>
Image<uchar> *threshold(Image<T> *src, int t) {
    int width = src->width();
    int height = src->height();
    Image<uchar> *dst = new Image<uchar>(width, height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            imRef(dst, x, y) = (imRef(src, x, y) >= t);
        }
    }
    return dst;
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
