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

#include <algorithm>

#include "modules/common/log.h"

namespace apollo {
namespace perception {

PbfTrackManager::PbfTrackManager() {}

int PbfTrackManager::RemoveLostTracks() {
  size_t original_size = tracks_.size();
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [](const PbfTrackPtr& p) { return p->IsDead(); }),
      tracks_.end());
  ADEBUG << "Removed " << original_size - tracks_.size() << " tracks";
  return original_size - tracks_.size();
}

}  // namespace perception
}  // namespace apollo
