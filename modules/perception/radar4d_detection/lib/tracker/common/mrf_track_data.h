/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/radar4d_detection/lib/tracker/common/track_data.h"

namespace apollo {
namespace perception {
namespace radar4d {

struct MrfPredict {
  Eigen::VectorXf state;
  base::PolygonDType polygon;
  base::RadarPointDCloud cloud;
  double timestamp;
  /**
   * @brief Reset all data
   *
   */
  void Reset() {
    state.setZero();
    polygon.clear();
    cloud.clear();
    timestamp = 0.0;
  }
};

class MrfTrackData : public TrackData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MrfTrackData() = default;
  ~MrfTrackData() = default;

  /**
   * @brief Reset
   *
   */
  void Reset() override;
  /**
   * @brief Reset all data
   *
   * @param obj
   * @param track_id
   */
  void Reset(TrackedObjectPtr obj, int track_id);

  /**
   * @brief Push tracked object to track
   *
   * @param obj
   */
  void PushTrackedObjectToTrack(TrackedObjectPtr obj);

  /**
   * @brief Push tracked object to cache
   *
   * @param obj
   */
  void PushTrackedObjectToCache(TrackedObjectPtr obj);

  /**
   * @brief Convert tracked object to base object
   *
   * @param local_to_global_offset
   * @param timestamp
   * @param object
   * @return true
   * @return false
   */
  bool ToObject(const Eigen::Vector3d& local_to_global_offset, double timestamp,
                base::ObjectPtr object) const;

  /**
   * @brief Remove stale history data
   *
   * @param timestamp
   */
  void RemoveStaleHistory(double timestamp);

  /**
   * @brief Predict state
   *
   * @param timestamp
   */
  void PredictState(double timestamp) const;

  /**
   * @brief Get and clean cached objects in time interval
   *
   * @param objects
   */
  void GetAndCleanCachedObjectsInTimeInterval(
      std::vector<TrackedObjectPtr>* objects);

  /**
   * @brief Get latest sensor object
   *
   * @param sensor_name
   * @return std::pair<double, TrackedObjectPtr>
   */
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
  /**
   * @brief Get latest sensor object
   *
   * @param sensor_name
   * @return std::pair<double, TrackedObjectConstPtr>
   */
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
  mutable MrfPredict predict_;

  double duration_ = 0.0;
  double consecutive_invisible_time_ = 0.0;
  double latest_visible_time_ = 0.0;
  double latest_cached_time_ = 0.0;
  double first_tracked_time_ = 0.0;

  bool is_current_state_predicted_ = true;

  // @debug feature to be used for semantic mapping based tracking
  //  std::shared_ptr<apollo::prediction::Feature> feature_;

  static const double kMaxHistoryTime;
};

typedef std::shared_ptr<MrfTrackData> MrfTrackDataPtr;
typedef std::shared_ptr<const MrfTrackData> MrfTrackDataConstPtr;

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
