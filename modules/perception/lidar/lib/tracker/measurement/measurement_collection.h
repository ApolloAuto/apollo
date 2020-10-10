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

#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

// @brief: measure anchor point velocity
// @params [in/out]: new object for current updating
// @params [in]: old object for last updating
// @params [in]: time interval from last updating
void MeasureAnchorPointVelocity(TrackedObjectPtr new_object,
                                const TrackedObjectConstPtr& old_object,
                                const double& time_diff);

// @brief: measure bbox center velocity
// @params [in/out]: new object for current updating
// @params [in]: old object for last updating
// @params [in]: time interval from last updating
void MeasureBboxCenterVelocity(TrackedObjectPtr new_object,
                               const TrackedObjectConstPtr& old_object,
                               const double& time_diff);

// @brief: measure bbox corner velocity
// @params [in/out]: new object for current updating
// @params [in]: old object for last updating
// @params [in]: time interval from last updating
void MeasureBboxCornerVelocity(TrackedObjectPtr new_object,
                               const TrackedObjectConstPtr& old_object,
                               const double& time_diff);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
