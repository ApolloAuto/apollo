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

#include <deque>
#include <map>
#include <memory>
#include <utility>

#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

enum class MotionState {
  STATIC = 0,
  SKEPTICAL_MOVE = 1,
  TRUSTED_MOVE = 2,
};

class TrackData {
 public:
  TrackData();
  TrackData(TrackedObjectPtr obj, int track_id);
  virtual ~TrackData();
  std::pair<double, TrackedObjectPtr> GetLatestObject() {
    if (history_objects_.size() != 0) {
      return *history_objects_.rbegin();
    }
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }
  std::pair<double, TrackedObjectConstPtr> GetLatestObject() const {
    if (history_objects_.size() != 0) {
      return *history_objects_.rbegin();
    }
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }
  std::pair<double, TrackedObjectPtr> GetOldestObject() {
    if (history_objects_.size() != 0) {
      return *history_objects_.begin();
    }
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }

  std::pair<double, TrackedObjectConstPtr> GetOldestObject() const {
    if (history_objects_.size() != 0) {
      return *history_objects_.begin();
    }
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }

  // when idx > 0, get object from oldest
  // when idx <= 0,  get object from latest
  // when abs_idx is large than history size, return farest object close to idx
  std::pair<double, TrackedObjectPtr> GetHistoryObject(int idx);

  std::pair<double, TrackedObjectConstPtr> GetHistoryObject(int idx) const;

  virtual void Reset();

  virtual void Reset(TrackedObjectPtr obj, double time, int track_id);

  virtual void PushTrackedObjectToTrack(TrackedObjectPtr obj, double time);

  int track_id_ = -1;
  int age_ = 0;
  int consecutive_invisible_count_ = 0;
  int total_visible_count_ = 0;
  static const int kMaxHistorySize;
  std::map<double, TrackedObjectPtr> history_objects_;
  int max_history_size_ = 40;
  // motion state related
  // used for judge object is static or not
  MotionState motion_state_ = MotionState::STATIC;
  size_t continuous_motion_frames_ = 0;
  size_t continuous_static_frames_ = 0;
  // if currenet frame is evaluated as in motion (implemented in post_process),
  // then the next pub_remain_frames should not be set as static,
  // in order to improve sensibility from static to motion
  size_t pub_remain_frames_ = 0;
  // if currenet frame is evaluated as static (implemented in post_process),
  // this flag is used to determine check velocity consistency or not,
  // in order to keep velocity consistency when measurement is unstable
  bool should_check_velocity_consistency_ = true;
  // the next two deques are used to calculate motion score
  std::deque<double> history_norm_variance_;
  std::deque<double> history_theta_variance_;
};

typedef std::shared_ptr<TrackData> TrackDataPtr;
typedef std::shared_ptr<const TrackData> TrackDataConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
