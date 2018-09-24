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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACKER_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACKER_H_

#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/common/track_pool_types.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/base_filter.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/measurement_computer.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/track_post_processor.h"

namespace apollo {
namespace perception {
namespace lidar {

struct TrackerOption {};

class Tracker {
 public:
  Tracker();

  ~Tracker() = default;

  virtual bool Init(const TrackerOption& option);

  virtual Eigen::VectorXd Predict(TrackDataPtr track_data, double timestamp);

  virtual void UpdateTrackDataWithObject(TrackDataPtr track_data,
                                         TrackedObjectPtr new_object,
                                         double timestamp);

  virtual void UpdateTrackDataWithoutObject(TrackDataPtr track_data,
                                            double timestamp);

  virtual std::string Name() const { return "Tracker"; };

 private:
  TrackedObjectPtr FakeTrackedObject(TrackDataPtr track_data, double timestamp);

  void UpdateNewTrack(TrackDataPtr track_data, TrackedObjectPtr new_object,
                      double timestamp);

  void UpdateForeground(TrackDataPtr track_data, TrackedObjectPtr new_object,
                        double timestamp);

  void UpdateBackground(TrackDataPtr track_data, TrackedObjectPtr new_object,
                        double timestamp);

  int GetNextTrackId() {
    if (current_track_id_ == INT_MAX) {
      current_track_id_ = 0;
    }
    return current_track_id_++;
  }

  bool tracker_prepared_;
  bool separate_fore_background_;
  int current_track_id_;

  MeasurementComputerPtr measure_computer_;
  BaseFilterPtr filter_;
  TrackPostProcessorPtr post_processor_;
};

typedef std::shared_ptr<Tracker> TrackerPtr;
typedef std::shared_ptr<const Tracker> TrackerConstPtr;
}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACKER_H_
