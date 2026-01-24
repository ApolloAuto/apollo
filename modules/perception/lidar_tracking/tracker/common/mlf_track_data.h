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

#include "modules/perception/common/base/object_types.h"
#include "modules/perception/lidar_tracking/tracker/common/track_data.h"

namespace apollo {
namespace perception {
namespace lidar {

struct MlfPredict {
  Eigen::VectorXf state;
  base::PolygonDType polygon;
  base::PointDCloud cloud;
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

class MlfTrackData : public TrackData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MlfTrackData() = default;
  ~MlfTrackData() = default;

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
                base::ObjectPtr object, bool update_time = true) const;

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

  void GetLatestKObjects(size_t k, std::vector<TrackedObjectPtr>* objects);

  void GetLatestKObjects(size_t k,
    std::vector<TrackedObjectConstPtr>* objects) const;

  void GetObjectsInInterval(double time,
      std::vector<TrackedObjectPtr>* objects);

  void GetObjectsInInterval(double time,
      std::vector<TrackedObjectConstPtr>* objects) const;

  void GetObjectsInIntervalByOrder(double time,
    std::vector<TrackedObjectConstPtr>* objects);

  void GetObjectsInIntervalByOrder(double time,
    std::vector<TrackedObjectConstPtr>* objects) const;

  void UpdateTrackableState(TrackedObjectPtr obj);

  bool IsForegroundTrack() const;

  bool IsBackgroundMovable(TrackedObjectPtr obj);

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
  bool is_front_critical_track_ = false;
  bool is_reserve_blind_cone_ = false;

  float foreground_track_prob_ = 0.0;

  static const double kMaxHistoryTime;
};

typedef std::shared_ptr<MlfTrackData> MlfTrackDataPtr;
typedef std::shared_ptr<const MlfTrackData> MlfTrackDataConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
