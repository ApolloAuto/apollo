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

#include "modules/perception/obstacle/radar/modest/conti_radar_id_expansion.h"

#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

TEST(ContiRadarIDExpansionTest, conti_radar_id_expansion_test) {
  ContiRadarIDExpansion id_expansion;
  ContiRadar raw_obstacles;
  ContiRadarObs *radar_obs = raw_obstacles.add_contiobs();
  radar_obs->set_obstacle_id(0);
  radar_obs->set_meas_state(static_cast<int>(ContiMeasState::CONTI_NEW));
  id_expansion.SetNeedRestart(true);
  id_expansion.UpdateTimestamp(0.0);
  id_expansion.ExpandIds(&raw_obstacles);
  EXPECT_EQ(radar_obs->obstacle_id(), 1);
  AINFO << radar_obs->obstacle_id();

  radar_obs->set_obstacle_id(0);
  id_expansion.UpdateTimestamp(0.07);
  radar_obs->set_meas_state(static_cast<int>(ContiMeasState::CONTI_MEASURED));
  id_expansion.ExpandIds(&raw_obstacles);
  EXPECT_EQ(radar_obs->obstacle_id(), 1);
  AINFO << radar_obs->obstacle_id();

  radar_obs->set_obstacle_id(1);
  id_expansion.UpdateTimestamp(0.14);
  id_expansion.ExpandIds(&raw_obstacles);
  EXPECT_EQ(radar_obs->obstacle_id(), 2);
  AINFO << radar_obs->obstacle_id();
}

TEST(ContiRadarIDExpansionSkipOutdatedObjectsTest, skip_outdated_objects_test) {
  ContiRadarIDExpansion id_expansion;
  ContiRadar raw_obstacles;
  auto *sensor_header = raw_obstacles.mutable_header();
  sensor_header->set_timestamp_sec(0.0);
  sensor_header->set_radar_timestamp(0.0 * 1e9);
  ContiRadarObs *radar_obs = raw_obstacles.add_contiobs();
  radar_obs->set_meas_state(static_cast<int>(ContiMeasState::CONTI_NEW));
  auto *header = radar_obs->mutable_header();
  header->set_timestamp_sec(0.0);
  header->set_radar_timestamp(0.0 * 1e9);
  id_expansion.SkipOutdatedObjects(&raw_obstacles);
  EXPECT_EQ(raw_obstacles.contiobs_size(), 1);

  sensor_header->set_timestamp_sec(0.7);
  sensor_header->set_radar_timestamp(0.7 * 1e9);
  ContiRadarObs *radar_obs2 = raw_obstacles.add_contiobs();
  radar_obs2->set_obstacle_id(0);
  radar_obs2->set_meas_state(static_cast<int>(ContiMeasState::CONTI_NEW));
  auto *header2 = radar_obs->mutable_header();
  header2->set_timestamp_sec(0.7);
  header2->set_radar_timestamp(0.7 * 1e9);
  id_expansion.SkipOutdatedObjects(&raw_obstacles);
  EXPECT_EQ(raw_obstacles.contiobs_size(), 1);
}

}  // namespace perception
}  // namespace apollo
