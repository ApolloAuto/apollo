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

#pragma once

#include <cmath>
namespace apollo {
namespace perception {
namespace lidar {
typedef unsigned char uchar;
typedef struct {
  uchar r;
  uchar g;
  uchar b;
} rgb;
inline bool operator==(const rgb &a, const rgb &b) {
  return ((a.r == b.r) && (a.g == b.g) && (a.b == b.b));
}
template <class T>
inline T abs(const T &x) {
  return (x > 0 ? x : -x);
}
template <class T>
inline int sign(const T &x) {
  return (x >= 0 ? 1 : -1);
}
template <class T>
inline T square(const T &x) {
  return x * x;
}
template <class T>
inline T bound(const T &x, const T &min, const T &max) {
  return (x < min ? min : (x > max ? max : x));
}
template <class T>
inline bool check_bound(const T &x, const T &min, const T &max) {
  return ((x < min) || (x > max));
}
inline int vlib_round(float x) { return static_cast<int>(x + 0.5F); }
inline int vlib_round(double x) { return static_cast<int>(x + 0.5); }
inline double gaussian(double val, double sigma) {
  return exp(-square(val / sigma) / 2) / (sqrt(2 * M_PI) * sigma);
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
