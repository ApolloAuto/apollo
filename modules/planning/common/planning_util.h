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
 * @file planning_util.h
 */

#ifndef MODULES_PLANNING_UTIL_PLANNING_UTIL_H_
#define MODULES_PLANNING_UTIL_PLANNING_UTIL_H_

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace planning {
namespace util {

common::SLPoint interpolate(const common::SLPoint &start,
                            const common::SLPoint &end, const double weight);

common::PathPoint interpolate(const common::PathPoint &p0,
                              const common::PathPoint &p1, const double s);

common::PathPoint InterpolateUsingLinearApproximation(
    const common::PathPoint &p0, const common::PathPoint &p1, const double s);

common::TrajectoryPoint interpolate(const common::TrajectoryPoint &tp0,
                                    const common::TrajectoryPoint &tp1,
                                    const double t);

common::TrajectoryPoint InterpolateUsingLinearApproximation(
    const common::TrajectoryPoint &tp0, const common::TrajectoryPoint &tp1,
    const double t);

void DumpPlanningContext();

}  // namespace util
}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_UTIL_PLANNING_UTIL_H_
