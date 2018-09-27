// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Yujiang Hui(huiyujiang@baidu.com)
// @file: radar_track_manager.cc
// @brief: radar tracker manager

#include "modules/perception/radar/lib/tracker/common/radar_track_manager.h"
#include <memory>
#include <utility>

namespace apollo {
namespace perception {
namespace radar {

int RadarTrackManager::RemoveLostTracks() {
  int track_count = 0;
  for (size_t i = 0; i < tracks_.size(); ++i) {
    if (!tracks_[i]->IsDead()) {
      if (i != track_count) {
        tracks_[track_count] = tracks_[i];
      }
      ++track_count;
    }
  }
  int removed_count = static_cast<int>(tracks_.size()) - track_count;
  AINFO << "Remove " << removed_count << " tracks";
  tracks_.resize(track_count);
  return track_count;
}

void RadarTrackManager::ClearTracks() { tracks_.clear(); }

}  // namespace radar
}  // namespace perception
}  // namespace apollo
