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
#include "modules/perception/common/radar/common/object_sequence.h"

#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace radar4d {

using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;

bool ObjectSequence::AddTrackedFrameObjects(
    const std::vector<ObjectPtr>& objects, TimeStampKey timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& obj : objects) {
    TrackIdKey& track_id = obj->track_id;
    auto iter = sequence_.find(track_id);
    if (iter == sequence_.end()) {
      auto res = sequence_.insert(std::make_pair(track_id, TrackedObjects()));
      iter = res.first;
    }
    auto res = iter->second.insert(std::make_pair(timestamp, obj));
    if (!res.second) {
      AERROR << "Fail to insert object." << std::endl;
      return false;
    }
  }
  RemoveStaleTracks(timestamp);
  current_ = timestamp;
  return true;
}

bool ObjectSequence::GetTrackInTemporalWindow(TrackIdKey track_id,
                                              TrackedObjects* track,
                                              TimeStampKey window_time) {
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
    if (tobj.first >= start_time) {
      track->insert(tobj);
    }
  }
  return true;
}

void ObjectSequence::RemoveStaleTracks(TimeStampKey current_stamp) {
  for (auto outer_iter = sequence_.begin(); outer_iter != sequence_.end();) {
    if (outer_iter->second.empty()) {
      AERROR << "Found empty tracks";
      continue;
    }
    auto& track = outer_iter->second;

    if (current_stamp - track.rbegin()->first > kMaxTimeOut) {
      sequence_.erase(outer_iter++);
      continue;
    }
    for (auto inner_iter = track.begin(); inner_iter != track.end();) {
      if (current_stamp - inner_iter->first > kMaxTimeOut) {
        track.erase(inner_iter++);
        continue;
      } else {
        break;
      }
    }
    if (track.empty()) {  // all element removed
      sequence_.erase(outer_iter++);
    } else {
      ++outer_iter;
    }
  }
}

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
