/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/sequence_type_fuser/object_sequence.h"

#include <utility>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

bool ObjectSequence::AddTrackedFrameObjects(
    const std::vector<std::shared_ptr<Object>>& objects, double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& obj : objects) {
    int& track_id = obj->track_id;
    auto iter = sequence_.find(track_id);
    if (iter == sequence_.end()) {
      auto res = sequence_.insert(std::make_pair(
          track_id, std::map<int64_t, std::shared_ptr<Object>>()));
      if (!res.second) {
        AERROR << "Fail to insert track.";
        return false;
      }
      iter = res.first;
    }
    auto res =
        iter->second.insert(std::make_pair(DoubleToMapKey(timestamp), obj));
    if (!res.second) {
      AERROR << "Fail to insert object.";
      return false;
    }
  }
  RemoveStaleTracks(timestamp);
  current_ = timestamp;
  return true;
}

bool ObjectSequence::GetTrackInTemporalWindow(
    int track_id, std::map<int64_t, std::shared_ptr<Object>>* track,
    double window_time) {
  if (track == nullptr) {
    return false;
  }
  track->clear();
  std::lock_guard<std::mutex> lock(mutex_);
  double start_time = current_ - window_time;
  auto iter = sequence_.find(track_id);
  if (iter == sequence_.end()) {
    return false;
  }
  for (auto& tobj : iter->second) {
    if (MapKeyToDouble(tobj.first) >= start_time) {
      track->insert(tobj);
    }
  }
  return true;
}

void ObjectSequence::RemoveStaleTracks(double current_stamp) {
  for (auto outer_iter = sequence_.begin(); outer_iter != sequence_.end();) {
    CHECK(outer_iter->second.size() > 0) << "Find empty tracks.";
    auto& track = outer_iter->second;
    if (current_stamp - MapKeyToDouble(track.rbegin()->first) >
        s_max_time_out_) {
      sequence_.erase(outer_iter++);
      continue;
    }
    for (auto inner_iter = track.begin(); inner_iter != track.end();) {
      if (current_stamp - MapKeyToDouble(inner_iter->first) > s_max_time_out_) {
        track.erase(inner_iter++);
        continue;
      } else {
        break;
      }
    }
    if (track.size() == 0) {  // all element removed
      sequence_.erase(outer_iter++);
    } else {
      ++outer_iter;
    }
  }
}

}  // namespace perception
}  // namespace apollo
