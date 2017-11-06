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
 
#ifndef ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_MANAGER_H
#define ADU_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_MANAGER_H
#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_background_track.h"

namespace apollo {
namespace perception {

class PbfTrackManager {
public:
    static PbfTrackManager* instance();
    ~PbfTrackManager();

    inline std::vector<PbfTrackPtr>& get_tracks() {
        return _tracks;
    }
    
    inline const std::vector<PbfTrackPtr>& get_tracks() const {
        return _tracks;
    }
    
    void add_track(const PbfTrackPtr& track){
        _tracks.push_back(track);
    }

    inline std::vector<PbfBackgroundTrackPtr>& get_background_tracks() {
        return _background_tracks;
    }
    
    inline const std::vector<PbfBackgroundTrackPtr>& get_background_tracks() const {
        return _background_tracks;
    }
    
    void add_background_track(const PbfBackgroundTrackPtr& track){
        _background_tracks.push_back(track);
    }

    int remove_lost_tracks();

private:
    DISALLOW_COPY_AND_ASSIGN(PbfTrackManager);
    PbfTrackManager();

protected:
    std::vector<PbfTrackPtr>  _tracks;
    std::vector<PbfBackgroundTrackPtr> _background_tracks;
};

} //namespace perception
} //namespace apollo

#endif