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
#ifndef PERCEPTION_LIDAR_COMMON_OBJECT_SEQUENCE_H_
#define PERCEPTION_LIDAR_COMMON_OBJECT_SEQUENCE_H_
#include <map>
#include <mutex>
#include <vector>
#include "modules/perception/base/object.h"
#include "modules/perception/lidar/common/lidar_log.h"

namespace apollo {
namespace perception {
namespace lidar {

class ObjectSequence {
 public:
  typedef int TrackIdKey;
  typedef double TimeStampKey;
  typedef std::map<TimeStampKey, base::ObjectPtr> TrackedObjects;

 public:
  ObjectSequence() = default;
  ~ObjectSequence() = default;

  bool AddTrackedFrameObjects(const std::vector<base::ObjectPtr>& objects,
                              TimeStampKey timestamp);

  bool GetTrackInTemporalWindow(TrackIdKey track_id, TrackedObjects* track,
                                TimeStampKey window_time);

 protected:
  void RemoveStaleTracks(TimeStampKey current_stamp);

 protected:
  TimeStampKey current_;
  std::map<TrackIdKey, TrackedObjects> sequence_;
  std::mutex mutex_;
  static constexpr TimeStampKey kMaxTimeOut = 5.0;  // 5 second
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_COMMON_OBJECT_SEQUENCE_H_
