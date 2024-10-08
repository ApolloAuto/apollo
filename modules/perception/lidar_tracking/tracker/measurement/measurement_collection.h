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

#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @brief Measure anchor point velocity
 *
 * @param new_object for current updating
 * @param old_object for last updating
 * @param time_diff time interval from last updating
 */
void MeasureAnchorPointVelocity(TrackedObjectPtr new_object,
                                const TrackedObjectConstPtr& old_object,
                                const double& time_diff);

/**
 * @brief Measure bbox center velocity
 *
 * @param new_object for current updating
 * @param old_object for last updating
 * @param time_diff time interval from last updating
 */
void MeasureBboxCenterVelocity(TrackedObjectPtr new_object,
                               const TrackedObjectConstPtr& old_object,
                               const double& time_diff);

void MeasureBboxCenterHistoryVelocity(const MlfTrackDataConstPtr& track_data,
    TrackedObjectPtr new_object);

/**
 * @brief Measure bbox corner velocity
 *
 * @param new_object for current updating
 * @param old_object for last updating
 * @param time_diff time interval from last updating
 */
void MeasureBboxCornerVelocity(TrackedObjectPtr new_object,
                               const TrackedObjectConstPtr& old_object,
                               const double& time_diff);

void MeasureBboxCornerHistoryVelocity(const MlfTrackDataConstPtr& track_data,
    TrackedObjectPtr new_object);

void ComputeBboxCornerHistoryVelocity(TrackedObjectPtr new_object,
    const TrackedObjectConstPtr& old_object, const double& time_diff,
    TrackedObjectPtr curr_object);

/**
 * @brief Measure object-detection center and corner velocity
 *
 * @param new_object for current updating
 * @param old_object for last updating
 * @param time_diff time interval from last updating
 */
void MeasureObjectDetectionVelocity(TrackedObjectPtr new_object,
    const TrackedObjectConstPtr& old_object, const double& time_diff);

void MeasureObjectDetectionHistoryVelocity(
    const MlfTrackDataConstPtr& track_data,
    TrackedObjectPtr new_object);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
