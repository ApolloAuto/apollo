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

#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

class MlfMotionMeasurement {
 public:
  MlfMotionMeasurement() = default;
  ~MlfMotionMeasurement() = default;
  /**
   * @brief Wrapper of motion measurement functions
   *
   * @param track_data track data
   * @param new_object object for current updating
   */
  void ComputeMotionMeasurment(const MlfTrackDataConstPtr& track_data,
                               TrackedObjectPtr new_object);

 protected:
  /**
   * @brief Select measurement based on track history
   *
   * @param track_data track data
   * @param latest_object latest object in track
   * @param new_object new object for storing selection
   * @param condition some special condition for selection
   */
  void MeasurementSelection(const MlfTrackDataConstPtr& track_data,
        const TrackedObjectConstPtr& latest_object,
        TrackedObjectPtr new_object, bool condition = true);
  /**
   * @brief Estimate measurement quality
   *
   * @param latest_object latest object in track
   * @param new_object new object for storing quality
   */
  void MeasurementQualityEstimation(const TrackedObjectConstPtr& latest_object,
                                    TrackedObjectPtr new_object);

  void MeasurementRefine(const MlfTrackDataConstPtr& track_data,
                         const TrackedObjectConstPtr& latest_object,
                         TrackedObjectPtr new_object, const double& time_diff);

  Eigen::Vector2d ComputeExpectedVelocity(
      std::vector<TrackedObjectConstPtr> history_objects, double cur_time);

 private:
  const double EPSILON_TIME = 1e-3;  // or numeric_limits<double>::epsilon()
  const double DEFAULT_FPS = 0.1;
  DISALLOW_COPY_AND_ASSIGN(MlfMotionMeasurement);
};  // class MlfMotionMeasurement

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
