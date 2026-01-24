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
/**
 * @brief Definition of equal sign
 *
 * @param a
 * @param b
 * @return true
 * @return false
 */
inline bool operator==(const rgb &a, const rgb &b) {
    return ((a.r == b.r) && (a.g == b.g) && (a.b == b.b));
}
/**
 * @brief Get absolute value of x
 *
 * @tparam T
 * @param x
 * @return T
 */
template <class T>
inline T abs(const T &x) {
    return (x > 0 ? x : -x);
}
/**
 * @brief Get sign of value x
 *
 * @tparam T
 * @param x
 * @return int
 */
template <class T>
inline int sign(const T &x) {
    return (x >= 0 ? 1 : -1);
}
/**
 * @brief get square of value x
 *
 * @tparam T
 * @param x
 * @return T
 */
template <class T>
inline T square(const T &x) {
    return x * x;
}
/**
 * @brief Get bound of value x
 *
 * @tparam T
 * @param x
 * @param min
 * @param max
 * @return T
 */
template <class T>
inline T bound(const T &x, const T &min, const T &max) {
    return (x < min ? min : (x > max ? max : x));
}
/**
 * @brief Check bound of x
 *
 * @tparam T
 * @param x
 * @param min
 * @param max
 * @return true
 * @return false
 */
template <class T>
inline bool check_bound(const T &x, const T &min, const T &max) {
    return ((x < min) || (x > max));
}
/**
 * @brief Get round value of x
 *
 * @param x
 * @return int
 */
inline int vlib_round(float x) {
    return static_cast<int>(x + 0.5F);
}
/**
 * @brief Get round value of x
 *
 * @param x
 * @return int
 */
inline int vlib_round(double x) {
    return static_cast<int>(x + 0.5);
}
/**
 * @brief Get gaussian value of val based on sigma
 *
 * @param val
 * @param sigma
 * @return double
 */
inline double gaussian(double val, double sigma) {
    return exp(-square(val / sigma) / 2) / (sqrt(2 * M_PI) * sigma);
}
}  // namespace lidar
}  // namespace perception
}  // namespace apollo
