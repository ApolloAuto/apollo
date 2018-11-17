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

/**
 * @file
 * @brief Nonlinear interpolation functions.
 */

#ifndef MODULES_COMMON_MATH_NONLINEAR_INTERPOLATION_H_
#define MODULES_COMMON_MATH_NONLINEAR_INTERPOLATION_H_

#include "modules/common/proto/pnc_point.pb.h"

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */

namespace apollo {
namespace common {
namespace math {

PathPoint SplineInterpolate(const PathPoint &p0, const PathPoint &p1,
                            const double s);

TrajectoryPoint SplineInterpolate(const TrajectoryPoint &tp0,
                                  const TrajectoryPoint &tp1,
                                  const double t);

}  // namespace math
}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_MATH_NONLINEAR_INTERPOLATION_H_
