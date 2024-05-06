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
#pragma once

#include <algorithm>
#include <sstream>
#include <string>

#include "modules/perception/common/base/comparison_traits.h"
#include "modules/perception/common/base/point.h"

namespace apollo {
namespace perception {
namespace base {

template <typename T>
struct BBox2D;

template <typename T>
struct Rect {
  Rect() : x(0), y(0), width(0), height(0) {}

  Rect(const T &x_in, const T &y_in, const T &width_in, const T &height_in)
      : x(x_in), y(y_in), width(width_in), height(height_in) {}

  explicit Rect(const BBox2D<T> &bbox) {
    this->x = bbox.xmin;
    this->y = bbox.ymin;
    this->width = bbox.xmax - bbox.xmin;
    this->height = bbox.ymax - bbox.ymin;
  }

  Rect<T> &operator=(const BBox2D<T> &bbox) {
    this->x = bbox.xmin;
    this->y = bbox.ymin;
    this->width = bbox.xmax - bbox.xmin;
    this->height = bbox.ymax - bbox.ymin;
    return *this;
  }

  Point2D<T> Center() const {
    Point2D<T> p;
    p.x = this->x + this->width / 2;
    p.y = this->y + this->height / 2;
    return p;
  }

  void SetCenter(Point2D<T> p) {
    this->x = p.x - this->width / 2;
    this->y = p.y - this->height / 2;
  }

  T Area() const { return this->width * this->height; }

  std::string ToStr() const {
    std::stringstream ss;
    ss << "[ " << width << " x " << height << " ] from ( " << x << " , " << y
       << " )";
    return ss.str();
  }

  friend Rect<T> operator&(const Rect<T> &rect1, const Rect<T> &rect2) {
    T r1_xmin = rect1.x;
    T r1_xmax = rect1.x + rect1.width;
    T r1_ymin = rect1.y;
    T r1_ymax = rect1.y + rect1.height;
    T r2_xmin = rect2.x;
    T r2_xmax = rect2.x + rect2.width;
    T r2_ymin = rect2.y;
    T r2_ymax = rect2.y + rect2.height;
    if (r2_xmin <= r1_xmax && r2_xmax >= r1_xmin && r2_ymin <= r1_ymax &&
        r2_ymax >= r1_ymin) {
      T xmin = std::max(r1_xmin, r2_xmin);
      T ymin = std::max(r1_ymin, r2_ymin);
      T xmax = std::min(r1_xmax, r2_xmax);
      T ymax = std::min(r1_ymax, r2_ymax);
      return Rect<T>(xmin, ymin, xmax - xmin, ymax - ymin);
    } else {
      return Rect<T>(0, 0, 0, 0);
    }
  }

  friend Rect<T> operator|(const Rect<T> &rect1, const Rect<T> &rect2) {
    Rect<T> ret;
    ret.x = std::min(rect1.x, rect2.x);
    ret.y = std::min(rect1.y, rect2.y);
    ret.width = std::max(rect1.x + rect1.width, rect2.x + rect2.width) - ret.x;
    ret.height =
        std::max(rect1.y + rect1.height, rect2.y + rect2.height) - ret.y;

    return ret;
  }

  friend inline bool operator==(const Rect &rect1, const Rect &rect2) {
    return (Equal(rect1.x, rect2.x) && Equal(rect1.y, rect2.y) &&
            Equal(rect1.width, rect2.width) &&
            Equal(rect1.height, rect2.height));
  }

  friend inline bool operator!=(const Rect &rect1, const Rect &rect2) {
    return !(rect1 == rect2);
  }

  T x = 0;  // top-left
  T y = 0;  // top-left
  T width = 0;
  T height = 0;
};

template <typename T>
struct BBox2D {
  BBox2D() : xmin(0), ymin(0), xmax(0), ymax(0) {}

  BBox2D(const T &xmin_in, const T &ymin_in, const T &xmax_in, const T &ymax_in)
      : xmin(xmin_in), ymin(ymin_in), xmax(xmax_in), ymax(ymax_in) {}

  explicit BBox2D(const Rect<T> &rect) {
    this->xmin = rect.x;
    this->ymin = rect.y;
    this->xmax = rect.x + rect.width;
    this->ymax = rect.y + rect.height;
  }

  BBox2D<T> &operator=(const Rect<T> &rect) {
    this->xmin = rect.x;
    this->ymin = rect.y;
    this->xmax = rect.x + rect.width;
    this->ymax = rect.y + rect.height;
    return *this;
  }

  Point2D<T> Center() const {
    Point2D<T> p;
    p.x = this->xmin + (this->xmax - this->xmin) / 2;
    p.y = this->ymin + (this->ymax - this->ymin) / 2;
    return p;
  }

  T Area() const { return (xmax - xmin) * (ymax - ymin); }
  T xmin = 0;  // top-left
  T ymin = 0;  // top-left
  T xmax = 0;  // bottom-right
  T ymax = 0;  // bottom-right
};

typedef Rect<int> RectI;
typedef Rect<float> RectF;
typedef Rect<double> RectD;

typedef BBox2D<int> BBox2DI;
typedef BBox2D<float> BBox2DF;
typedef BBox2D<double> BBox2DD;

}  // namespace base
}  // namespace perception
}  // namespace apollo
