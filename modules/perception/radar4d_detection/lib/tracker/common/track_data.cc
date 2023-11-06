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
#include "modules/perception/radar4d_detection/lib/tracker/common/track_data.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace radar4d {
const int TrackData::kMaxHistorySize = 40;
TrackData::TrackData() { Reset(); }

TrackData::TrackData(TrackedObjectPtr obj, int track_id) {}

TrackData::~TrackData() {}

std::pair<double, TrackedObjectPtr> TrackData::GetHistoryObject(int idx) {
  if (history_objects_.empty()) {
    AWARN << "no object in track";
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }
  int max_idx = std::abs(idx) >= static_cast<int>(history_objects_.size())
                    ? static_cast<int>(history_objects_.size()) - 1
                    : abs(idx);
  // from oldest
  if (idx > 0) {
    std::map<double, TrackedObjectPtr>::iterator cur_obj =
        history_objects_.begin();
    for (int i = 0; i < max_idx; ++i) {
      ++cur_obj;
    }
    return *cur_obj;
  } else {
    std::map<double, TrackedObjectPtr>::reverse_iterator cur_obj =
        history_objects_.rbegin();
    for (int i = 0; i < max_idx; ++i) {
      ++cur_obj;
    }
    return *cur_obj;
  }
}

std::pair<double, TrackedObjectConstPtr> TrackData::GetHistoryObject(
    int idx) const {
  if (history_objects_.empty()) {
    AINFO << "no object in track";
    return std::pair<double, TrackedObjectPtr>(0.0, TrackedObjectPtr(nullptr));
  }
  int max_idx = static_cast<size_t>(abs(idx)) >= history_objects_.size()
                    ? static_cast<int>(history_objects_.size()) - 1
                    : abs(idx);
  // from oldest
  if (idx > 0) {
    std::map<double, TrackedObjectPtr>::const_iterator cur_obj =
        history_objects_.cbegin();
    for (int i = 0; i < max_idx; ++i) {
      ++cur_obj;
    }
    return *cur_obj;
  } else {
    std::map<double, TrackedObjectPtr>::const_reverse_iterator cur_obj =
        history_objects_.crbegin();
    for (int i = 0; i < max_idx; ++i) {
      ++cur_obj;
    }
    return *cur_obj;
  }
}

void TrackData::Reset() {
  track_id_ = -1;
  age_ = 0;
  consecutive_invisible_count_ = 0;
  total_visible_count_ = 0;
  max_history_size_ = 40;
  history_objects_.clear();
  motion_state_ = MotionState::STATIC;
  continuous_motion_frames_ = 0;
  continuous_static_frames_ = 0;
  pub_remain_frames_ = 0;
  should_check_velocity_consistency_ = true;
  history_norm_variance_.clear();
  history_theta_variance_.clear();
}

void TrackData::Reset(TrackedObjectPtr obj, double time, int track_id) {
  Reset();
  track_id_ = track_id;
  PushTrackedObjectToTrack(obj, time);
}

void TrackData::PushTrackedObjectToTrack(TrackedObjectPtr obj, double time) {
  if (history_objects_.find(time) == history_objects_.end()) {
    history_objects_.insert(std::make_pair(time, obj));
    age_++;
    obj->track_id = track_id_;
    obj->tracking_time = time - history_objects_.begin()->first;
    if (obj->is_fake) {
      ++consecutive_invisible_count_;
    } else {
      consecutive_invisible_count_ = 0;
      ++total_visible_count_;
    }
    if (history_objects_.size() > kMaxHistorySize) {
      history_objects_.erase(history_objects_.begin());
    }
  } else {
    AWARN << "push object time " << time
          << " already exist in track, ignore insert.";
  }
}

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
