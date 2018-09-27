// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: hui yujiang (huiyujiang@baidu.com)
// @file: radar_tracker_radar_track_manager_test.cc
// @brief: radar track manager test

#include <gtest/gtest.h>
#include "cybertron/common/log.h"
#include "modules/perception/radar/lib/tracker/common/radar_track_manager.h"
#include "modules/perception/radar/lib/tracker/filter/adaptive_kalman_filter.h"

namespace apollo {
namespace perception {

namespace radar {
TEST(RadarTrackManagerTest, radar_track_manager_test) {
  base::ObjectPtr object(new base::Object);
  object->track_id = 100;
  object->center << 10.0, 20.0, 0.0;
  object->velocity << 3.0, 4.0, 0.0;
  double timestamp = 123456789.0;
  RadarTrackPtr radar_track(new RadarTrack(object, timestamp));
  RadarTrackPtr radar_track2(new RadarTrack(object, timestamp));

  RadarTrackManager* manager = new RadarTrackManager();
  EXPECT_NE(manager, nullptr);
  manager->AddTrack(radar_track);
  auto& radar_tracks = manager->GetTracks();
  EXPECT_EQ(radar_tracks.size(), 1);
  manager->AddTrack(radar_track2);
  manager->RemoveLostTracks();
  EXPECT_EQ(manager->GetTracks().size(), 2);
  radar_tracks[0]->SetDead();
  manager->RemoveLostTracks();
  EXPECT_EQ(manager->GetTracks().size(), 1);
  manager->ClearTracks();
  EXPECT_EQ(manager->GetTracks().size(), 0);
  delete manager;
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
