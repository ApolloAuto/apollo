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
#include "modules/perception/radar/lib/tracker/conti_ars_tracker/conti_ars_tracker.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace radar {

TEST(ContiArsTrackerTest, conti_ars_tracker_init_test) {
  std::unique_ptr<BaseTracker> tracker(new ContiArsTracker());
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/"
      "radar/conti_ars_tracker";
  EXPECT_TRUE(tracker->Init());
  EXPECT_EQ(tracker->Name(), "ContiArsTracker");
}

TEST(ContiArsTrackerTest, conti_ars_tracker_track_test) {
  std::unique_ptr<BaseTracker> tracker(new ContiArsTracker());
  FLAGS_work_root = "./radar_test_data/conti_ars_tracker";
  tracker->Init();
  base::Frame radar_frame;
  radar_frame.timestamp = 123456789.1;
  radar_frame.objects.resize(2);
  radar_frame.objects[0].reset(new base::Object);
  radar_frame.objects[0]->track_id = 100;
  radar_frame.objects[0]->center << 12.0, 15.0, 0.0;
  radar_frame.objects[0]->velocity << 3.0, 4.0, 0.0;
  radar_frame.objects[1].reset(new base::Object);
  radar_frame.objects[1]->track_id = 200;
  radar_frame.objects[1]->center << 50.3, 100.4, 0.0;
  radar_frame.objects[1]->velocity << 3.0, 4.0, 0.0;
  TrackerOptions options;
  base::FramePtr tracked_frame(new base::Frame);
  bool state = tracker->Track(radar_frame, options, tracked_frame);
  EXPECT_TRUE(state);
}

TEST(ContiArsTrackerTest, conti_ars_tracker_collect_test) {
  base::ObjectPtr object(new base::Object);
  object->track_id = 100;
  object->center << 10.0, 20.0, 0.0;
  object->velocity << 3.0, 4.0, 0.0;
  double timestamp = 123456789.0;
  RadarTrackPtr radar_track(new RadarTrack(object, timestamp));

  std::unique_ptr<ContiArsTracker> tracker(new ContiArsTracker());
  FLAGS_work_root = "./radar_test_data/conti_ars_tracker";
  tracker->Init();
  tracker->track_manager_->ClearTracks();
  tracker->track_manager_->AddTrack(radar_track);
  base::FramePtr tracked_frame(new base::Frame);
  tracker->CollectTrackedFrame(tracked_frame);
  EXPECT_EQ(tracked_frame->objects.size(), 0);
  RadarTrack::SetTrackedTimesThreshold(0);
  tracker->CollectTrackedFrame(tracked_frame);
  EXPECT_EQ(tracked_frame->objects.size(), 1);
}

TEST(ContiArsTrackerTest, conti_ars_tracker_unassigned_test) {
  base::ObjectPtr object(new base::Object);
  object->track_id = 100;
  object->center << 10.0, 20.0, 0.0;
  object->velocity << 3.0, 4.0, 0.0;
  double timestamp = 123456789.0;
  RadarTrackPtr radar_track(new RadarTrack(object, timestamp));

  std::vector<size_t> unassigned_tracks;
  unassigned_tracks.push_back(0);

  base::Frame radar_frame;
  radar_frame.timestamp =
      timestamp + ContiArsTracker::s_tracking_time_win_ + 1e-5;

  std::unique_ptr<ContiArsTracker> tracker(new ContiArsTracker());
  FLAGS_work_root = "./radar_test_data/conti_ars_tracker";
  tracker->Init();
  tracker->track_manager_->ClearTracks();
  tracker->track_manager_->AddTrack(radar_track);
  tracker->UpdateUnassignedTracks(radar_frame, unassigned_tracks);
  tracker->track_manager_->RemoveLostTracks();
  EXPECT_EQ(tracker->track_manager_->GetTracks().size(), 0);

  tracker->track_manager_->ClearTracks();
  RadarTrackPtr radar_track2(new RadarTrack(object, timestamp));
  tracker->track_manager_->AddTrack(radar_track2);
  radar_frame.timestamp =
      timestamp + ContiArsTracker::s_tracking_time_win_ - 1e-5;
  tracker->UpdateUnassignedTracks(radar_frame, unassigned_tracks);
  tracker->track_manager_->RemoveLostTracks();
  EXPECT_EQ(tracker->track_manager_->GetTracks().size(), 1);

  tracker->track_manager_->ClearTracks();
  RadarTrackPtr radar_track3(new RadarTrack(object, timestamp));
  radar_track3->SetObsRadarNullptr();
  tracker->track_manager_->AddTrack(radar_track3);
  radar_frame.timestamp =
      timestamp + ContiArsTracker::s_tracking_time_win_ - 1e-5;
  tracker->UpdateUnassignedTracks(radar_frame, unassigned_tracks);
  tracker->track_manager_->RemoveLostTracks();
  EXPECT_EQ(tracker->track_manager_->GetTracks().size(), 0);
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
