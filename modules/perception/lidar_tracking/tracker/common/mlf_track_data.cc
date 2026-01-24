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

#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"

#include "cyber/common/log.h"
#include "modules/perception/lidar_tracking/tracker/common/track_pool_types.h"

namespace apollo {
namespace perception {
namespace lidar {

const double MlfTrackData::kMaxHistoryTime = 2.0;

void MlfTrackData::Reset() {
  TrackData::Reset();
  duration_ = 0.0;
  consecutive_invisible_time_ = 0.0;
  latest_visible_time_ = 0.0;
  latest_cached_time_ = 0.0;
  first_tracked_time_ = 0.0;
  is_current_state_predicted_ = true;
  is_front_critical_track_ = false;
  is_reserve_blind_cone_ = false;
  foreground_track_prob_ = 0.0;
  sensor_history_objects_.clear();
  cached_objects_.clear();
  predict_.Reset();
//  feature_.reset();
}

void MlfTrackData::Reset(TrackedObjectPtr obj, int track_id) {
  Reset();
  track_id_ = track_id;
  PushTrackedObjectToCache(obj);
}

void MlfTrackData::PushTrackedObjectToTrack(TrackedObjectPtr obj) {
  // double timestamp = obj->object_ptr->latest_tracked_time;
  double timestamp = obj->timestamp;
  if (history_objects_.find(timestamp) == history_objects_.end()) {
    auto pair = std::make_pair(timestamp, obj);
    history_objects_.insert(pair);
    sensor_history_objects_[obj->sensor_info.name].insert(pair);
    age_++;
    if (age_ == 1) {  // the first timestamp
      if (obj->is_fake) {
        AERROR << "obj is fake";
        return;
      }
      latest_visible_time_ = timestamp;
      first_tracked_time_ = timestamp;
    }
    obj->track_id = track_id_;
    obj->tracking_time = timestamp - first_tracked_time_;
    duration_ = obj->tracking_time;
    if (obj->is_fake) {
      ++consecutive_invisible_count_;
      consecutive_invisible_time_ = timestamp - latest_visible_time_;
    } else {
      consecutive_invisible_count_ = 0;
      consecutive_invisible_time_ = 0.0;
      ++total_visible_count_;
      latest_visible_time_ = timestamp;
    }
    RemoveStaleHistory(timestamp - kMaxHistoryTime);
  } else {
    AINFO << "Push object timestamp " << timestamp << " from sensor "
          << obj->sensor_info.name << " already exist in track, ignore push.";
  }
}

void MlfTrackData::PushTrackedObjectToCache(TrackedObjectPtr obj) {
  double timestamp = obj->object_ptr->latest_tracked_time;
  if (cached_objects_.find(timestamp) == cached_objects_.end()) {
    cached_objects_.insert(std::make_pair(timestamp, obj));
    latest_cached_time_ = timestamp;
  } else {
    AINFO << "Push object timestamp " << timestamp << " from sensor "
          << obj->sensor_info.name << " already exist in cache, ignore push.";
  }
}

bool MlfTrackData::ToObject(const Eigen::Vector3d& local_to_global_offset,
          double timestamp, base::ObjectPtr object, bool update_time) const {
  if (history_objects_.empty()) {
    return false;
  }
  auto latest_iter = history_objects_.rbegin();
  const double latest_time = latest_iter->first;
  const auto& latest_object = latest_iter->second;
  latest_object->ToObject(object);
  // predict object
  double time_diff = timestamp - latest_time;
  double offset_x = time_diff * object->velocity(0) + local_to_global_offset(0);
  double offset_y = time_diff * object->velocity(1) + local_to_global_offset(1);
  double offset_z = time_diff * object->velocity(2) + local_to_global_offset(2);
  // a). update polygon
  for (auto& pt : object->polygon) {
    pt.x += offset_x;
    pt.y += offset_y;
    pt.z += offset_z;
  }

  // b). update center
  object->center(0) += offset_x;
  object->center(1) += offset_y;
  object->center(2) += offset_z;
  // c). update anchor point
  object->anchor_point(0) += offset_x;
  object->anchor_point(1) += offset_y;
  object->anchor_point(2) += offset_z;
  if (update_time) {
      // d). update tracking timestamp
      object->tracking_time += time_diff;
      // e). update latest track timestamp
      object->latest_tracked_time += time_diff;
  }
  // f). update cloud world in lidar supplement
  for (auto& pt : object->lidar_supplement.cloud_world) {
    pt.x += offset_x;
    pt.y += offset_y;
    pt.z += offset_z;
  }
  return true;
}

void MlfTrackData::PredictState(double timestamp) const {
  if (history_objects_.empty()) {
    return;
  }
  auto latest_iter = history_objects_.rbegin();
  const double latest_time = latest_iter->first;
  const auto& latest_object = latest_iter->second;
  double time_diff = timestamp - latest_time;

  const Eigen::Vector3d& latest_anchor_point =
      latest_object->belief_anchor_point;
  const Eigen::Vector3d& latest_velocity = latest_object->output_velocity;

  predict_.state.resize(6);
  predict_.state(0) = static_cast<float>(latest_anchor_point(0) +
                                         latest_velocity(0) * time_diff);
  predict_.state(1) = static_cast<float>(latest_anchor_point(1) +
                                         latest_velocity(1) * time_diff);
  predict_.state(2) = static_cast<float>(latest_anchor_point(2) +
                                         latest_velocity(2) * time_diff);
  predict_.state(3) = static_cast<float>(latest_velocity(0));
  predict_.state(4) = static_cast<float>(latest_velocity(1));
  predict_.state(5) = static_cast<float>(latest_velocity(2));

  predict_.timestamp = timestamp;
  // TODO(.): predict cloud and polygon if needed in future.
}

void MlfTrackData::GetAndCleanCachedObjectsInTimeInterval(
    std::vector<TrackedObjectPtr>* objects) {
  objects->clear();
  auto iter = cached_objects_.begin();
  while (iter != cached_objects_.end()) {
    const auto& timestamp = iter->first;
    if (timestamp <= latest_visible_time_) {
      cached_objects_.erase(iter++);
    } else if (timestamp <= latest_cached_time_) {
      objects->push_back(iter->second);
      cached_objects_.erase(iter++);
    } else {
      break;
    }
  }
}

void RemoveStaleDataFromMap(double timestamp,
                            std::map<double, TrackedObjectPtr>* data) {
  auto iter = data->begin();
  while (iter != data->end()) {
    if (iter->first < timestamp) {
      data->erase(iter++);
    } else {
      break;
    }
  }
}

void MlfTrackData::RemoveStaleHistory(double timestamp) {
  RemoveStaleDataFromMap(timestamp, &history_objects_);
  for (auto& map : sensor_history_objects_) {
    RemoveStaleDataFromMap(timestamp, &map.second);
  }
}

void MlfTrackData::GetLatestKObjects(size_t k,
    std::vector<TrackedObjectPtr>* objects) {
    objects->clear();
    size_t i = 0;
    for (auto obj_it = history_objects_.rbegin();
      obj_it != history_objects_.rend(); ++obj_it) {
        if (i < k) {
            objects->push_back(obj_it->second);
            ++i;
        } else {
            break;
        }
    }
}

void MlfTrackData::GetLatestKObjects(size_t k,
    std::vector<TrackedObjectConstPtr>* objects) const {
    objects->clear();
    size_t i = 0;
    for (auto obj_it = history_objects_.rbegin();
      obj_it != history_objects_.rend(); ++obj_it) {
        if (i < k) {
            objects->push_back(obj_it->second);
            ++i;
        } else {
            break;
        }
    }
}

void MlfTrackData::GetObjectsInInterval(double time,
    std::vector<TrackedObjectPtr>* objects) {
  objects->clear();
  for (auto obj_it = history_objects_.rbegin();
      obj_it != history_objects_.rend(); ++obj_it) {
    if (obj_it->second->timestamp >= time) {
      objects->push_back(obj_it->second);
    } else {
      break;
    }
  }
}

void MlfTrackData::GetObjectsInInterval(double time,
    std::vector<TrackedObjectConstPtr>* objects) const {
  objects->clear();
  for (auto obj_it = history_objects_.rbegin();
      obj_it != history_objects_.rend(); ++obj_it) {
    if (obj_it->second->timestamp >= time) {
      objects->push_back(obj_it->second);
    } else {
      break;
    }
  }
}

void MlfTrackData::GetObjectsInIntervalByOrder(double time,
    std::vector<TrackedObjectConstPtr>* objects) {
    for (auto obj_it = history_objects_.begin();
      obj_it != history_objects_.end(); ++obj_it) {
        if (obj_it->second->timestamp >= time) {
            objects->push_back(obj_it->second);
        }
    }
}

void MlfTrackData::GetObjectsInIntervalByOrder(double time,
  std::vector<TrackedObjectConstPtr>* objects) const {
    for (auto obj_it = history_objects_.begin();
      obj_it != history_objects_.end(); ++obj_it) {
        if (obj_it->second->timestamp >= time) {
            objects->push_back(obj_it->second);
        }
    }
}

void MlfTrackData::UpdateTrackableState(TrackedObjectPtr obj) {
  if (age_ == 0) {
    foreground_track_prob_ =
      obj->object_ptr->lidar_supplement.is_clustered ? 0.0 : 1.0;
  } else {
    if (IsForegroundTrack() || IsBackgroundMovable(obj)) {
      obj->object_ptr->lidar_supplement.is_background = false;
      obj->is_background = false;
    }
    if (!obj->object_ptr->lidar_supplement.is_clustered) {
      foreground_track_prob_ = 1.0;
    } else {
      foreground_track_prob_ *= 0.9;
    }
  }
}

bool MlfTrackData::IsForegroundTrack() const {
  return (foreground_track_prob_ > 0.5);
}

bool MlfTrackData::IsBackgroundMovable(TrackedObjectPtr obj) {
  bool is_background_movable =
    (obj->object_ptr->lidar_supplement.dynamic_state ==
      base::MotionState::MOVING) ||
    (obj->object_ptr->lidar_supplement.semantic_type ==
      base::ObjectSemanticType::OBJECT);

  return is_background_movable;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
