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
#include "modules/perception/radar_detection/lib/tracker/common/radar_track.h"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/radar_detection/lib/tracker/filter/adaptive_kalman_filter.h"

namespace apollo {
namespace perception {

namespace radar {
TEST(RadarTrackTest, radar_track_test) {
  int tracked_times_threshold = 1;
  RadarTrack::SetTrackedTimesThreshold(tracked_times_threshold);
  std::string chosen_filter = "AdaptiveKalmanFilter";
  RadarTrack::SetChosenFilter(chosen_filter);
  bool use_filter = false;
  RadarTrack::SetUseFilter(use_filter);

  base::ObjectPtr object(new base::Object);
  object->track_id = 100;
  object->center << 10.0, 20.0, 0.0;
  object->velocity << 3.0, 4.0, 0.0;
  double timestamp = 123456789.0;
  RadarTrackPtr radar_track(new RadarTrack(object, timestamp));
  EXPECT_EQ(radar_track->GetObsId(), 0);
  EXPECT_FALSE(radar_track->ConfirmTrack());
  EXPECT_FALSE(radar_track->IsDead());
  radar_track->SetDead();
  EXPECT_TRUE(radar_track->IsDead());

  base::ObjectPtr object2(new base::Object);
  object2->track_id = 100;
  object2->center << 10.3, 20.4, 0.0;
  object2->velocity << 3.0, 4.0, 0.0;
  double timestamp2 = 123456789.1;
  radar_track->UpdataObsRadar(object2, timestamp2);
  EXPECT_TRUE(radar_track->ConfirmTrack());

  RadarTrackPtr radar_track2(new RadarTrack(object, timestamp));
  use_filter = true;
  RadarTrack::SetUseFilter(use_filter);
  base::ObjectPtr object3(new base::Object);
  object3->track_id = 100;
  object3->center << 10.3, 20.4, 0.0;
  object3->velocity << 3.0, 4.0, 0.0;
  double timestamp3 = 123456789.1;
  radar_track2->UpdataObsRadar(object3, timestamp3);
  EXPECT_LT(radar_track->GetTrackingTime() - 0.1, 1e-5);
  EXPECT_TRUE(radar_track2->ConfirmTrack());

  chosen_filter = "Default";
  RadarTrack::SetChosenFilter(chosen_filter);
  RadarTrackPtr radar_track3(new RadarTrack(object, timestamp));
}

TEST(RadarTrackTest, radar_track_function_test) {
  base::ObjectPtr object(new base::Object);
  object->track_id = 100;
  object->center << 10.0, 20.0, 0.0;
  object->velocity << 3.0, 4.0, 0.0;
  double timestamp = 123456789.0;
  RadarTrackPtr radar_track(new RadarTrack(object, timestamp));
  EXPECT_LT(std::fabs(radar_track->GetTrackingTime() - 0.0), 1e-5);
  EXPECT_LT(std::fabs(radar_track->GetTimestamp() - timestamp), 1e-5);
  radar_track->SetObsRadarNullptr();
  EXPECT_EQ(radar_track->GetObsRadar(), nullptr);
  EXPECT_EQ(radar_track->GetObs(), nullptr);
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
