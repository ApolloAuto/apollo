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
#include "modules/perception/radar_detection/lib/tracker/matcher/hm_matcher.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/radar_detection/lib/tracker/common/radar_track.h"
#include "modules/perception/radar_detection/lib/tracker/common/radar_track_manager.h"

namespace apollo {
namespace perception {
namespace radar {

TEST(HMMatcherTest, hm_matcher_init_test) {
  BaseMatcher* matcher = new HMMatcher();
  EXPECT_NE(matcher, nullptr);
  FLAGS_work_root = "/apollo/modules/perception/testdata/radar/matcher";
  MatcherInitOptions init_options;
  EXPECT_TRUE(matcher->Init(init_options));
  delete matcher;
}

TEST(HMMatcherTest, hm_matcher_name_test) {
  BaseMatcher* matcher = new HMMatcher();
  EXPECT_EQ(matcher->Name(), "HMMatcher");
  delete matcher;
}

TEST(HMMatcherTest, hm_matcher_propterty_match_test) {
  HMMatcher* matcher = new HMMatcher();
  std::vector<RadarTrackPtr> radar_tracks;
  base::Frame radar_frame;
  std::vector<TrackObjectPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  unassigned_tracks.push_back(0);
  matcher->TrackObjectPropertyMatch(radar_tracks, radar_frame, &assignments,
                                    &unassigned_tracks, &unassigned_objects);
  EXPECT_EQ(unassigned_tracks.size(), 1);
  EXPECT_EQ(unassigned_objects.size(), 0);
  delete matcher;
}

TEST(HMMatcherTest, hm_matcher_test) {
  BaseMatcher* matcher = new HMMatcher();
  EXPECT_NE(matcher, nullptr);
  FLAGS_work_root = "/apollo/modules/perception/testdata/radar/matcher";
  MatcherInitOptions init_options;
  EXPECT_TRUE(matcher->Init(init_options));
  EXPECT_EQ(matcher->Name(), "HMMatcher");
  double match_distance = 2.5;
  BaseMatcher::SetMaxMatchDistance(match_distance);

  std::vector<RadarTrackPtr> radar_tracks;
  base::Frame radar_frame;
  std::vector<TrackObjectPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  matcher->IDMatch(radar_tracks, radar_frame, &assignments, &unassigned_tracks,
                   &unassigned_objects);
  EXPECT_EQ(assignments.size(), 0);
  EXPECT_EQ(unassigned_tracks.size(), 0);
  EXPECT_EQ(unassigned_objects.size(), 0);

  base::ObjectPtr object(new base::Object);
  object->track_id = 100;
  object->center << 10.0, 20.0, 0.0;
  object->velocity << 3.0, 4.0, 0.0;
  double timestamp = 123456789.0;
  RadarTrackPtr radar_track(new RadarTrack(object, timestamp));
  base::ObjectPtr object2(new base::Object);
  object2->track_id = 101;
  object2->center << 50.0, 100.0, 0.0;
  object2->velocity << 3.0, 4.0, 0.0;
  RadarTrackPtr radar_track2(new RadarTrack(object2, timestamp));
  base::ObjectPtr object3(new base::Object);
  object3->track_id = 102;
  object3->center << 20.0, 30.0, 0.0;
  object3->velocity << 3.0, 4.0, 0.0;
  RadarTrackPtr radar_track3(new RadarTrack(object3, timestamp));
  radar_tracks.push_back(radar_track);
  radar_tracks.push_back(radar_track2);
  radar_tracks.push_back(radar_track3);
  radar_frame.timestamp = 123456789.1;
  radar_frame.objects.resize(3);
  radar_frame.objects[0].reset(new base::Object);
  radar_frame.objects[0]->track_id = 100;
  radar_frame.objects[0]->center << 12.0, 15.0, 0.0;
  radar_frame.objects[0]->velocity << 3.0, 4.0, 0.0;
  radar_frame.objects[1].reset(new base::Object);
  radar_frame.objects[1]->track_id = 200;
  radar_frame.objects[1]->center << 50.3, 100.4, 0.0;
  radar_frame.objects[1]->velocity << 3.0, 4.0, 0.0;
  radar_frame.objects[2].reset(new base::Object);
  radar_frame.objects[2]->track_id = 102;
  radar_frame.objects[2]->center << 20.3, 30.4, 0.0;
  radar_frame.objects[2]->velocity << 3.0, 4.0, 0.0;

  TrackObjectMatcherOptions options;
  bool match_state =
      matcher->Match(radar_tracks, radar_frame, options, &assignments,
                     &unassigned_tracks, &unassigned_objects);
  EXPECT_EQ(assignments.size(), 2);
  EXPECT_EQ(unassigned_tracks.size(), 1);
  EXPECT_EQ(unassigned_objects.size(), 1);
  EXPECT_TRUE(match_state);
  delete matcher;
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
