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

#include "modules/common/math/vec2d.h"
#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @brief: compute euclidean distance for given track & object
 * @param last_object object for computing distance
 * @param track_predict predicted state of track for computing distance
 * @param new_object new detected object for computing distance
 * @param time_diff time interval from last matching
 * @return: euclidean distance
*/
float EuclideanDistance(const TrackedObjectConstPtr& last_object,
        const Eigen::VectorXf& track_predict,
        const TrackedObjectConstPtr& new_object,
        const double time_diff);

/**
 * @brief Compute location distance for given track & object
 *
 * @param last_object object for computing distance
 * @param track_predict predicted state of track for computing distance
 * @param new_object new detected object for computing distance
 * @param time_diff time interval from last matching
 * @return float distance
 */
float LocationDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff);

/**
 * @brief Compute direction distance for given track & object
 *
 * @param last_object object for computing distance
 * @param track_predict predicted state of track for computing distance
 * @param new_object new detected object for computing distance
 * @param time_diff time interval from last matching
 * @return float distance
 */
float DirectionDistance(const TrackedObjectConstPtr& last_object,
                        const Eigen::VectorXf& track_predict,
                        const TrackedObjectConstPtr& new_object,
                        const double time_diff);

/**
 * @brief Compute bbox size distance for given track & object
 *
 * @param last_object object for computing distance
 * @param track_predict predicted state of track for computing distance
 * @param new_object new detected object for computing distance
 * @param time_diff time interval from last matching
 * @return float distance
 */
float BboxSizeDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff);

/**
 * @brief Compute point num distance for given track & object
 *
 * @param last_object object for computing distance
 * @param track_predict predicted state of track for computing distance
 * @param new_object new detected object for computing distance
 * @param time_diff time interval from last matching
 * @return float distance
 */
float PointNumDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff);

/**
 * @brief Compute histogram distance for given track & object
 *
 * @param last_object object for computing distance
 * @param track_predict predicted state of track for computing distance
 * @param new_object new detected object for computing distance
 * @param time_diff time interval from last matching
 * @return float distance
 */
float HistogramDistance(const TrackedObjectConstPtr& last_object,
                        const Eigen::VectorXf& track_predict,
                        const TrackedObjectConstPtr& new_object,
                        const double time_diff);

/**
 * @brief Compute centroid shift distance for object and background match
 *
 * @param last_object object for computing distance
 * @param track_predict unused
 * @param cur_obj new detected object for computing distance
 * @param time_diff unused
 * @return float distance
 */
float CentroidShiftDistance(const TrackedObjectConstPtr& last_object,
                            const Eigen::VectorXf& track_predict,
                            const TrackedObjectConstPtr& cur_obj,
                            const double time_diff);

/**
 * @brief Compute bbox iou distance for object and background match
 *
 * @param last_object object for computing distance
 * @param track_predict unused
 * @param cur_obj new detected object for computing distance
 * @param time_diff unused
 * @param match_threshold match threshold
 * @return float distance
 */
float BboxIouDistance(const TrackedObjectConstPtr& last_object,
                      const Eigen::VectorXf& track_predict,
                      const TrackedObjectConstPtr& cur_obj,
                      const double time_diff, double match_threshold);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
