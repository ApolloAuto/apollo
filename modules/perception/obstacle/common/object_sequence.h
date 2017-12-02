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
#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_OBJECT_SEQUENCE_H
#define MODULES_PERCEPTION_OBSTACLE_COMMON_OBJECT_SEQUENCE_H
#include <map>
#include <mutex>
#include <vector>

#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

class ObjectSequence {
 public:
  // typedef int TrackIdKey;
  // typedef double TimeStampKey;
  typedef std::map<double, ObjectPtr> TrackedObjects;
  // typedef std::map<TimeStampKey, ObjectPtr> LastSightingObjects;
 public:
  ObjectSequence() = default;
  ~ObjectSequence() = default;

  bool AddTrackedFrameObjects(const std::vector<ObjectPtr>& objects,
                              double timestamp);

  bool GetTrackInTemporalWindow(int track_id, TrackedObjects* track,
                                double window_time);

  // bool get_objects_in_spatial_area(
  //         LastSightingObjects* objects,
  //         const Eigen::Vector3d& center,
  //         double radius);

 protected:
  void RemoveStaleTracks(double current_stamp);

 protected:
  double _current;
  std::map<int, TrackedObjects> _sequence;
  std::mutex _mutex;
  static constexpr double _s_max_time_out = 5;  // 5 second
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_OBJECT_SEQUENCE_H
