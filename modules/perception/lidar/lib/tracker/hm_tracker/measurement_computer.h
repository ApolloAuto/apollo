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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_MEASUREMENT_COMPUTER_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_MEASUREMENT_COMPUTER_H_

#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

class MeasurementComputer {
 public:
  MeasurementComputer() = default;
  ~MeasurementComputer() = default;

  // @brief compute measured velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] new_time_stamp: new object timestamp
  // @params[OUT] measured velocity updated in new_object
  void ComputeMeasurment(const TrackDataPtr &track_data,
                         TrackedObjectPtr new_object,
                         const double &new_time_stamp);
  // @brief compute measured anchor point velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @params[out] measured anchor point velocity updated in new object
  void ComputeMeasuredAnchorPointVelocity(TrackedObjectPtr new_object,
                                          const TrackedObjectPtr &old_object,
                                          const double &time_diff);
  // @brief compute measured bbox center velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @params[out] measured bbox center velocity updated in new object
  void ComputeMeasuredBboxCenterVelocity(TrackedObjectPtr new_object,
                                         const TrackedObjectPtr &old_object,
                                         const double &time_diff);
  // @brief compute measured bbox corner velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @params[out] measured bbox corner velocity and update new_object corner
  void ComputeMeasuredBboxCornerVelocity(TrackedObjectPtr new_object,
                                         const TrackedObjectPtr &old_object,
                                         const double &time_diff);
};

typedef std::shared_ptr<MeasurementComputer> MeasurementComputerPtr;
typedef std::shared_ptr<const MeasurementComputer> MeasurementComputerConstPtr;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_MEASUREMENT_COMPUTER_H_
