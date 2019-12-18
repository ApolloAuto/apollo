/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/perception/base/blob.h"
#include "modules/perception/base/image.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"

namespace apollo {
namespace perception {
namespace camera {

bool Equal(float x, float target, float eps = 1e-6f);
bool Equal(double x, double target, double eps = 1e-6);

// @brief whether rect1 is covered by rect2
template <typename T>
bool IsCovered(const base::Rect<T> &rect1, const base::Rect<T> &rect2,
               float thresh) {
  base::RectF inter = rect1 & rect2;
  return inter.Area() / rect1.Area() > thresh;
}
template <typename T>
bool IsCoveredHorizon(const base::Rect<T> &rect1, const base::Rect<T> &rect2,
                      float thresh) {
  base::RectF inter = rect1 & rect2;
  if (inter.Area() > 0) {
    return inter.width / rect1.width > thresh;
  }
  return false;
}
template <typename T>
bool IsCoveredVertical(const base::Rect<T> &rect1, const base::Rect<T> &rect2,
                       float thresh) {
  base::RectF inter = rect1 & rect2;
  if (inter.Area() > 0) {
    return inter.height / rect1.height > thresh;
  }
  return false;
}

template <typename T>
bool Contain(const std::vector<T> &array, const T &element) {
  for (const auto &item : array) {
    if (item == element) {
      return true;
    }
  }
  return false;
}

template <typename T>
bool OutOfValidRegion(const base::BBox2D<T> box, const T width, const T height,
                      const T border_size = 0) {
  if (box.xmin < border_size || box.ymin < border_size) {
    return true;
  }
  if (box.xmax + border_size > width || box.ymax + border_size > height) {
    return true;
  }
  return false;
}
template <typename T>
bool OutOfValidRegion(const base::Rect<T> rect, const T width, const T height,
                      const T border_size = 0) {
  base::BBox2D<T> box(rect);
  return OutOfValidRegion(box, width, height, border_size);
}

template <typename T>
void RefineBox(const base::Rect<T> &box_in, const T width, const T height,
               base::Rect<T> *box_out) {
  if (!box_out) {
    return;
  }
  *box_out = box_in;
  if (box_out->x < 0) {
    box_out->width += box_out->x;
    box_out->x = 0;
  }
  if (box_out->y < 0) {
    box_out->height += box_out->y;
    box_out->y = 0;
  }
  if (box_out->x >= width) {
    box_out->x = 0;
    box_out->width = 0;
  }
  if (box_out->y >= height) {
    box_out->y = 0;
    box_out->height = 0;
  }
  box_out->width = (box_out->x + box_out->width <= width) ? box_out->width
                                                          : width - box_out->x;
  box_out->height = (box_out->y + box_out->height <= height)
                        ? box_out->height
                        : height - box_out->y;
  if (box_out->width < 0) {
    box_out->width = 0;
  }
  if (box_out->height < 0) {
    box_out->height = 0;
  }
}

template <typename T>
void RefineBox(const base::BBox2D<T> &box_in, const T width, const T height,
               base::BBox2D<T> *box_out) {
  if (!box_out) {
    return;
  }
  base::Rect<T> rect;
  RefineBox(base::Rect<T>(box_in), width, height, &rect);
  *box_out = base::BBox2D<T>(rect);
}

bool LoadAnchors(const std::string &path, std::vector<float> *anchors);
bool LoadTypes(const std::string &path,
               std::vector<base::ObjectSubType> *types);
bool LoadExpand(const std::string &path, std::vector<float> *expands);

bool ResizeCPU(const base::Blob<uint8_t> &src_gpu,
               std::shared_ptr<base::Blob<float>> dst, int stepwidth,
               int start_axis);

std::string GetCyberWorkRoot();
void FillObjectPolygonFromBBox3D(base::Object *object_ptr);

template <typename T>
void CalculateMeanAndVariance(const std::vector<T> &data, T *mean,
                              T *variance) {
  if (!mean || !variance) {
    return;
  }
  if (data.empty()) {
    *mean = 0;
    *variance = 0;
    return;
  }
  T sum = std::accumulate(data.begin(), data.end(), static_cast<T>(0));
  *mean = sum / data.size();

  std::vector<T> diff(data.size());
  std::transform(data.begin(), data.end(), diff.begin(),
                 [mean](T x) { return x - *mean; });
  T sum_of_diff_sqrs = std::inner_product(diff.begin(), diff.end(),
                                          diff.begin(), static_cast<T>(0));
  *variance = sum_of_diff_sqrs / data.size();
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
