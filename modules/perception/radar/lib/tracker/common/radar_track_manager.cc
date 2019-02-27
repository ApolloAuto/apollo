/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar/lib/tracker/common/radar_track_manager.h"

namespace apollo {
namespace perception {
namespace radar {

int RadarTrackManager::RemoveLostTracks() {
  size_t track_count = 0;
  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (!tracks_[i]->IsDead()) {
      if (i != track_count) {
        tracks_[track_count] = tracks_[i];
      }
      ++track_count;
    }
  }
  int removed_count = static_cast<int>(tracks_.size() - track_count);
  ADEBUG << "Remove " << removed_count << " tracks";
  tracks_.resize(track_count);
  return static_cast<int>(track_count);
}

void RadarTrackManager::ClearTracks() { tracks_.clear(); }

}  // namespace radar
}  // namespace perception
}  // namespace apollo
