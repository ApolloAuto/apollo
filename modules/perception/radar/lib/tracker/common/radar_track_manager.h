// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Yujiang Hui(huiyujiang@baidu.com)
// @file: radar_track_manager.h
// @brief: radar track manager

#ifndef RADAR_LIB_TRACKER_COMMON_RADAR_TRACK_MANAGER_H_
#define RADAR_LIB_TRACKER_COMMON_RADAR_TRACK_MANAGER_H_

#include <utility>
#include <vector>
#include "modules/perception/base/frame.h"
#include "modules/perception/radar/lib/tracker/common/radar_track.h"

namespace apollo {
namespace perception {
namespace radar {

class RadarTrackManager {
 public:
  RadarTrackManager() = default;
  ~RadarTrackManager() = default;
  inline std::vector<RadarTrackPtr> &GetTracks() { return tracks_; }

  inline const std::vector<RadarTrackPtr> &GetTracks() const { return tracks_; }

  void AddTrack(const RadarTrackPtr &track) { tracks_.push_back(track); }
  int RemoveLostTracks();
  void ClearTracks();

 private:
  RadarTrackManager &operator=(const RadarTrackManager &) = delete;

 protected:
  std::vector<RadarTrackPtr> tracks_;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_TRACKER_COMMON_RADAR_TRACK_MANAGER_H_
