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
 */

#ifndef MODULES_PLANNING_UTIL_PLANNING_UTIL_H_
#define MODULES_PLANNING_UTIL_PLANNING_UTIL_H_

#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/proto/planning_status.pb.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace planning {
namespace util {

/**
 * This function returns the run-time state of the planning module.
 * @Warnning: this function is not thread safe.
 */
PlanningStatus *GetPlanningStatus();

void DumpPlanningContext();

}  // namespace util
}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_UTIL_PLANNING_UTIL_H_
