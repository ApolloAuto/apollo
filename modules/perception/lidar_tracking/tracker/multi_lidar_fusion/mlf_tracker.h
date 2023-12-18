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

#include <limits>
#include <string>
#include <vector>

#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar_tracking/tracker/common/tracked_object.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_base_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct MlfTrackerInitOptions : public BaseInitOptions {};

struct MlfTrackOptions {};

class MlfTracker {
 public:
  MlfTracker() = default;
  ~MlfTracker() {
    for (auto& filter : filters_) {
      delete filter;
    }
  }
  /**
   * @brief Init mlf tracker
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const MlfTrackerInitOptions options = MlfTrackerInitOptions());

  /**
   * @brief Initialize new track data and push new object to cache
   *
   * @param new_track_data new track data
   * @param new_object new object
   */
  void InitializeTrack(MlfTrackDataPtr new_track_data,
                       TrackedObjectPtr new_object);

  /**
   * @brief Update track data with object
   *
   * @param track_data history track data
   * @param new_object new object
   */
  void UpdateTrackDataWithObject(MlfTrackDataPtr track_data,
                                 TrackedObjectPtr new_object);

  /**
   * @brief Update track data without object
   *
   * @param timestamp
   * @param track_data history track data
   */
  void UpdateTrackDataWithoutObject(double timestamp,
                                    MlfTrackDataPtr track_data);

  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const { return "MlfTracker"; }

 protected:
  /**
   * @brief Get next track id
   *
   * @return int track id
   */
  int GetNextTrackId() {
    if (global_track_id_counter_ == std::numeric_limits<int>::max()) {
      global_track_id_counter_ = 0;
    }
    return global_track_id_counter_++;
  }

 protected:
  // a single whole state filter or separate state filters
  std::vector<MlfBaseFilter*> filters_;
  // global track id
  int global_track_id_counter_ = 0;
  // filter option
  MlfFilterOptions filter_options_;
};  // class MlfTracker

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
