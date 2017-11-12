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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track_manager.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

PbfTrackManager *PbfTrackManager::instance() {
  static PbfTrackManager track_manager;
  return &track_manager;
}

PbfTrackManager::PbfTrackManager() {

}

PbfTrackManager::~PbfTrackManager() {

}

int PbfTrackManager::RemoveLostTracks() {
  int track_count = 0;
  for (int i = 0; i < tracks_.size(); i++) {
    if (!tracks_[i]->IsDead()) {
      if (i != track_count) {
        tracks_[track_count] = tracks_[i];
      }
      track_count++;
    }
  }
  AINFO << "Remove " << (int) tracks_.size() - track_count << " tracks";
  tracks_.resize(track_count);

  return track_count;
}

} //namespace perception
} //namespace apollo