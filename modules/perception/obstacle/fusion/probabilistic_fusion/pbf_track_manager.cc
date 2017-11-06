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

PbfTrackManager* PbfTrackManager::instance() {
    static PbfTrackManager track_manager;
    return &track_manager;
}

PbfTrackManager::PbfTrackManager() {

}

PbfTrackManager::~PbfTrackManager() {

}

int PbfTrackManager::remove_lost_tracks() {
    int track_count = 0;
    for (int i = 0; i < _tracks.size(); i++) {
        if (!_tracks[i]->is_dead()) {
            if (i != track_count) {
                _tracks[track_count] = _tracks[i];
            }
            track_count++;            
        }
    }

    AINFO << "Remove " << (int)_tracks.size() - track_count << " tracks";
    _tracks.resize(track_count);

    int bk_track_count = 0;
    for (size_t i = 0; i < _background_tracks.size(); i++) {
        if (!_background_tracks[i]->is_dead()) {
            if (i != bk_track_count) {
                _background_tracks[bk_track_count] = _background_tracks[i];
            }
            bk_track_count++;
        }
    }
    AINFO << "Remove " << (int)_tracks.size() - track_count << " background tracks";
    _background_tracks.resize(bk_track_count);

    return track_count + bk_track_count;
}

} //namespace perception
} //namespace apollo