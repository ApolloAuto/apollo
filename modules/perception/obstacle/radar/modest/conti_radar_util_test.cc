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
#include "modules/perception/obstacle/radar/modest/conti_radar_util.h"

#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {
TEST(ContiRadarUtilTest, conti_radar_util_test) {
  ContiRadarObs contiobs;
  const int delay_frames = 4;
  const double min_value = 1e-3;
  int tracking_times = delay_frames;
  double probexist = 0.9;
  double lo_vel_rms = 0.02;
  double la_vel_rms = 0.03;
  double lo_dist_rms = 0.2;
  double la_dist_rms = 0.3;
  ContiParams conti_params;
  conti_params.probexist_vehicle = probexist;
  conti_params.probexist_pedestrian = probexist;
  conti_params.probexist_bicycle = probexist;
  conti_params.probexist_unknown = probexist;
  conti_params.lo_vel_rms_vehicle = lo_vel_rms;
  conti_params.la_vel_rms_vehicle = la_vel_rms;
  conti_params.lo_dist_rms_vehicle = lo_dist_rms;
  conti_params.la_dist_rms_vehicle = la_dist_rms;
  conti_params.lo_vel_rms_pedestrian = lo_vel_rms;
  conti_params.la_vel_rms_pedestrian = la_vel_rms;
  conti_params.lo_dist_rms_pedestrian = lo_dist_rms;
  conti_params.la_dist_rms_pedestrian = la_dist_rms;
  conti_params.lo_vel_rms_bicycle = lo_vel_rms;
  conti_params.la_vel_rms_bicycle = la_vel_rms;
  conti_params.lo_dist_rms_bicycle = lo_dist_rms;
  conti_params.la_dist_rms_bicycle = la_dist_rms;
  conti_params.lo_vel_rms_unknown = lo_vel_rms;
  conti_params.la_vel_rms_unknown = la_vel_rms;
  conti_params.lo_dist_rms_unknown = lo_dist_rms;
  conti_params.la_dist_rms_unknown = la_dist_rms;
  contiobs.set_meas_state(static_cast<int>(ContiMeasState::CONTI_NEW));
  contiobs.set_probexist(probexist - min_value);
  contiobs.set_longitude_vel_rms(lo_vel_rms + min_value);
  contiobs.set_lateral_vel_rms(la_vel_rms - min_value);
  contiobs.set_longitude_dist_rms(lo_dist_rms - min_value);
  contiobs.set_lateral_dist_rms(la_dist_rms - min_value);
  bool state = false;
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_CAR));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_PEDESTRIAN));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_MOTOCYCLE));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_POINT));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_probexist(probexist + min_value);
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_CAR));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_PEDESTRIAN));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_MOTOCYCLE));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_POINT));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_longitude_vel_rms(lo_vel_rms - min_value);
  contiobs.set_lateral_vel_rms(la_vel_rms - min_value);
  contiobs.set_longitude_dist_rms(lo_dist_rms - min_value);
  contiobs.set_lateral_dist_rms(la_dist_rms - min_value);
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_CAR));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_PEDESTRIAN));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, false);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_MOTOCYCLE));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, false);
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_POINT));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);

  tracking_times = delay_frames * 2 + 1;
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_CAR));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, false);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_PEDESTRIAN));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, false);
  contiobs.set_obstacle_class(
      static_cast<int>(ContiObjectType::CONTI_MOTOCYCLE));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, false);
  contiobs.set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_POINT));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, false);

  contiobs.set_meas_state(static_cast<int>(ContiMeasState::CONTI_DELETED));
  state = ContiRadarUtil::IsFp(contiobs, conti_params, delay_frames,
                               tracking_times);
  EXPECT_EQ(state, true);
  AINFO << "Conti radar util test end!";
}

}  // namespace perception
}  // namespace apollo
