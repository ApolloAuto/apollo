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
#pragma once

#include <limits>
#include <string>
#include <vector>

#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/radar4d_detection/lib/tracker/common/mrf_track_data.h"
#include "modules/perception/radar4d_detection/lib/tracker/common/tracked_object.h"
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_base_filter.h"

namespace apollo {
namespace perception {
namespace radar4d {

using apollo::perception::BaseInitOptions;

struct MrfTrackerInitOptions : public BaseInitOptions {};

struct MrfTrackOptions {};

class MrfTracker {
 public:
  MrfTracker() = default;
  ~MrfTracker() {
    for (auto& filter : filters_) {
      delete filter;
    }
  }
  /**
   * @brief Init mrf tracker
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const MrfTrackerInitOptions options = MrfTrackerInitOptions());

  /**
   * @brief Initialize new track data and push new object to cache
   *
   * @param new_track_data new track data
   * @param new_object new object
   */
  void InitializeTrack(MrfTrackDataPtr new_track_data,
                       TrackedObjectPtr new_object);

  /**
   * @brief Update track data with object
   *
   * @param track_data history track data
   * @param new_object new object
   */
  void UpdateTrackDataWithObject(MrfTrackDataPtr track_data,
                                 TrackedObjectPtr new_object);

  /**
   * @brief Update track data without object
   *
   * @param timestamp
   * @param track_data history track data
   */
  void UpdateTrackDataWithoutObject(double timestamp,
                                    MrfTrackDataPtr track_data);

  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const { return "MrfTracker"; }

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
  std::vector<MrfBaseFilter*> filters_;
  // global track id
  int global_track_id_counter_ = 0;
  // filter option
  MrfFilterOptions filter_options_;
};  // class MrfTracker

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
