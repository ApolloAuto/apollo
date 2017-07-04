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

/**
 * @file
 * @brief Linear interpolation functions.
 */

#ifndef MODULES_COMMON_MATH_LINEAR_INTERPOLATION_H_
#define MODULES_COMMON_MATH_LINEAR_INTERPOLATION_H_

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace apollo {
namespace common {
namespace math {

/**
 * @brief Linear interpolation between two 1-D points.
 * @param x0 The coordinate of the first 1-D point.
 * @param t0 The interpolation parameter of the first 1-D point.
 * @param x1 The coordinate of the second 1-D point.
 * @param t1 The interpolation parameter of the second 1-D point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated 1-D point.
 * @return Interpolated 1-D point.
 */
double lerp(const double x0, const double t0, const double x1, const double t1,
            const double t);

/**
 * @brief Linear interpolation between two 2-D points.
 * @param x0 The x coordinate of the first 2-D point.
 * @param y0 The y coordinate of the first 2-D point.
 * @param t0 The interpolation parameter of the first 2-D point.
 * @param x1 The x coordinate of the second 2-D point.
 * @param y1 The y coordinate of the second 2-D point.
 * @param t1 The interpolation parameter of the second 2-D point.
 * @param t The interpolation parameter for interpolation.
 * @param x The x coordinate of the interpolated 2-D point.
 * @param y The y coordinate of the interpolated 2-D point.
 */
void lerp(const double x0, const double y0, const double t0, const double x1,
          const double y1, const double t1, const double t, double* x,
          double* y);

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif /* MODULES_COMMON_MATH_LINEAR_INTERPOLATION_H_ */
