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

#include "cyber/common/log.h"
#include "gtest/gtest.h"
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
  auto& radar_tracks = manager->mutable_tracks();
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
