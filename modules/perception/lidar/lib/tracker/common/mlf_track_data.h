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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/lidar/lib/tracker/common/track_data.h"

namespace apollo {
namespace perception {
namespace lidar {

struct MlfPredict {
  Eigen::VectorXf state;
  base::PolygonDType polygon;
  base::PointDCloud cloud;
  double timestamp;

  void Reset() {
    state.setZero();
    polygon.clear();
    cloud.clear();
    timestamp = 0.0;
  }
};

class MlfTrackData : public TrackData {
 public:
  MlfTrackData() = default;
  ~MlfTrackData() = default;

  void Reset() override;

  void Reset(TrackedObjectPtr obj, int track_id);

  void PushTrackedObjectToTrack(TrackedObjectPtr obj);

  void PushTrackedObjectToCache(TrackedObjectPtr obj);

  bool ToObject(const Eigen::Vector3d& local_to_global_offset, double timestamp,
                base::ObjectPtr object) const;

  void RemoveStaleHistory(double timestamp);

  void PredictState(double timestamp) const;

  void GetAndCleanCachedObjectsInTimeInterval(
      std::vector<TrackedObjectPtr>* objects);

  std::pair<double, TrackedObjectPtr> GetLatestSensorObject(
      const std::string& sensor_name) {
    auto iter = sensor_history_objects_.find(sensor_name);
    if (iter != sensor_history_objects_.end()) {
      auto& history_objects = iter->second;
      if (history_objects.size() != 0) {
        return *history_objects.rbegin();
      }
    }
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }

  std::pair<double, TrackedObjectConstPtr> GetLatestSensorObject(
      const std::string& sensor_name) const {
    auto iter = sensor_history_objects_.find(sensor_name);
    if (iter != sensor_history_objects_.end()) {
      auto& history_objects = iter->second;
      if (history_objects.size() != 0) {
        return *history_objects.rbegin();
      }
    }
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }

 public:
  typedef std::map<double, TrackedObjectPtr> TimedObjects;
  std::map<std::string, TimedObjects> sensor_history_objects_;
  TimedObjects cached_objects_;

  // buffer for predict data
  mutable MlfPredict predict_;

  double duration_ = 0.0;
  double consecutive_invisible_time_ = 0.0;
  double latest_visible_time_ = 0.0;
  double latest_cached_time_ = 0.0;
  double first_tracked_time_ = 0.0;

  bool is_current_state_predicted_ = true;

  // @debug feature to be used for semantic mapping based tracking
  std::shared_ptr<apollo::prediction::Feature> feature_;

  static const double kMaxHistoryTime;
};

typedef std::shared_ptr<MlfTrackData> MlfTrackDataPtr;
typedef std::shared_ptr<const MlfTrackData> MlfTrackDataConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
