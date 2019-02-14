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
#pragma once

#include <vector>

#include "cyber/common/macros.h"

#include "modules/perception/base/frame.h"
#include "modules/perception/radar/lib/tracker/common/radar_track.h"

namespace apollo {
namespace perception {
namespace radar {

class RadarTrackManager {
 public:
  RadarTrackManager() = default;
  ~RadarTrackManager() = default;

  inline std::vector<RadarTrackPtr> &mutable_tracks() { return tracks_; }
  inline const std::vector<RadarTrackPtr> &GetTracks() const { return tracks_; }

  void AddTrack(const RadarTrackPtr &track) { tracks_.push_back(track); }
  int RemoveLostTracks();
  void ClearTracks();

 protected:
  std::vector<RadarTrackPtr> tracks_;

 private:
  DISALLOW_COPY_AND_ASSIGN(RadarTrackManager);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
