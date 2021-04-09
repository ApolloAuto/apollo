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

#ifndef MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_OBJECT_SEQUENCE_H_
#define MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_OBJECT_SEQUENCE_H_

#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "modules/perception/obstacle/base/object.h"

namespace apollo {
namespace perception {

class ObjectSequence {
 public:
  /**
   * @brief Construct
   */
  ObjectSequence() = default;

  /**
   * @brief Destruct
   */
  ~ObjectSequence() = default;

  /**
   * @brief Add objects of one single frame to the current sequence
   * @param objects The frame objects to be added to the storaged sequence
   * @param timestamp The frame timestamp
   * @return True if add successfully, false otherwise
   */
  bool AddTrackedFrameObjects(
      const std::vector<std::shared_ptr<Object>>& objects, double timestamp);

  /**
   * @brief Get tracked objects in a time window
   * @param track_id The track id of object sequence
   * @param track The output tracked objects
   * @param window_time The time interval
   * @return True if get track successfully, false otherwise
   */
  bool GetTrackInTemporalWindow(
      int track_id, std::map<int64_t, std::shared_ptr<Object>>* track,
      double window_time);

 protected:
  /**
   * @brief Remove too old tracks
   * @param current_stamp Current timestamp
   */
  void RemoveStaleTracks(double current_stamp);

 private:
  const double kEps = 1e-9;
  int64_t DoubleToMapKey(const double d) {
    return static_cast<int64_t>(d / kEps);
  }
  double MapKeyToDouble(const int64_t key) { return key * kEps; }

  double current_;
  std::map<int, std::map<int64_t, std::shared_ptr<Object>>> sequence_;
  std::mutex mutex_;
  static constexpr double s_max_time_out_ = 5.0;  // 5 seconds
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_COMMON_SEQUENCE_TYPE_FUSER_OBJECT_SEQUENCE_H_
