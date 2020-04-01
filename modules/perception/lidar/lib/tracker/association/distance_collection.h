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
#include "modules/perception/lidar/lib/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

// @brief: compute location distance for given track & object
// @params [in]: object for computing distance
// @params [in]: predicted state of track for computing distance
// @params [in]: new detected object for computing distance
// @params [in]: time interval from last matching
// @return: distance
float LocationDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff);

// @brief: compute direction distance for given track & object
// @params [in]: object for computing distance
// @params [in]: predicted state of track for computing distance
// @params [in]: new detected object for computing distance
// @params [in]: time interval from last matching
// @return distance
float DirectionDistance(const TrackedObjectConstPtr& last_object,
                        const Eigen::VectorXf& track_predict,
                        const TrackedObjectConstPtr& new_object,
                        const double time_diff);

// @brief: compute bbox size distance for given track & object
// @params [in]: object for computing distance
// @params [in]: predicted state of track for computing distance
// @params [in]: new detected object for computing distance
// @params [in]: time interval from last matching
// @return distance
float BboxSizeDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff);

// @brief: compute point num distance for given track & object
// @params [in]: object for computing distance
// @params [in]: predicted state of track for computing distance
// @params [in]: new detected object for computing distance
// @params [in]: time interval from last matching
// @return distance
float PointNumDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff);

// @brief: compute histogram distance for given track & object
// @params [in]: object for computing distance
// @params [in]: predicted state of track for computing distance
// @params [in]: new detected object for computing distance
// @params [in]: time interval from last matching
// @return distance
float HistogramDistance(const TrackedObjectConstPtr& last_object,
                        const Eigen::VectorXf& track_predict,
                        const TrackedObjectConstPtr& new_object,
                        const double time_diff);

// @brief: compute centroid shift distance for object and background match
// @params [in]: object for computing distance
// @params [in]: unused
// @params [in]: new detected object for computing distance
// @params [in]: unused
// @return distance
float CentroidShiftDistance(const TrackedObjectConstPtr& last_object,
                            const Eigen::VectorXf& track_predict,
                            const TrackedObjectConstPtr& cur_obj,
                            const double time_diff);

// @brief compute bbox iou distance for object and background match
// @params [in]: object for computing distance
// @params [in]: unused
// @params [in]: new detected object for computing distance
// @params [in]: unused
// @return distance
float BboxIouDistance(const TrackedObjectConstPtr& last_object,
                      const Eigen::VectorXf& track_predict,
                      const TrackedObjectConstPtr& cur_obj,
                      const double time_diff, double match_threshold);

// @brief lidar only: compute semantic map based distance
// @params [in]: track data contained predicted trajectory feature
// @params [in]: new detected object for computing distance
// @return distance
float SemanticMapDistance(const MlfTrackData& track_dat,
                          const TrackedObjectConstPtr& cur_obj);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
