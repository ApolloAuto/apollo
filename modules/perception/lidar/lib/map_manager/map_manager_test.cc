/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar/lib/map_manager/map_manager.h"

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace lidar {

TEST(LidarLibMapManagerTest, lidar_map_manager_empty_test) {
  char cyber_path[100] = "CYBER_PATH=";
  putenv(cyber_path);
  char module_path[100] = "MODULE_PATH=";
  putenv(module_path);

  FLAGS_work_root = "/apollo/modules/perception/testdata/lidar/lib/map_manager";
  FLAGS_config_manager_path = "./empty_conf";
  lib::ConfigManager::Instance()->Reset();

  map::HDMapInput::Instance()->Reset();

  MapManager map_manager;
  EXPECT_TRUE(map_manager.Init());
}

TEST(LidarLibMapManagerTest, lidar_map_manager_test) {
  char cyber_path[100] = "CYBER_PATH=";
  putenv(cyber_path);
  char module_path[100] = "MODULE_PATH=";
  putenv(module_path);

  FLAGS_work_root = "/apollo/modules/perception/testdata/lidar/lib/map_manager";
  FLAGS_config_manager_path = "./conf";
  lib::ConfigManager::Instance()->Reset();

  map::HDMapInput::Instance()->Reset();

  MapManager map_manager;
  EXPECT_EQ(map_manager.Name(), "MapManager");
  MapManagerOptions option;
  EXPECT_FALSE(map_manager.Update(option, nullptr));
  LidarFrame frame;
  EXPECT_FALSE(map_manager.Update(option, &frame));
  EXPECT_TRUE(map_manager.Init());
  EXPECT_TRUE(map_manager.Update(option, &frame));

  Eigen::Matrix4d pose;
  pose << 0.85161966102636, -0.524117104574822, 0.00672297565424851,
      428800.6181463, 0.524128638716867, 0.851357499926331, -0.0218989069419588,
      4439224.020332, 0.00575393659561208, 0.0221732437986386,
      0.999737585292331, 36.327793326766, 0, 0, 0, 1;

  frame.lidar2world_pose.prerotate(pose.block<3, 3>(0, 0));
  frame.lidar2world_pose.pretranslate(pose.block<3, 1>(0, 3));
  EXPECT_TRUE(map_manager.Update(option, &frame));
  EXPECT_TRUE(frame.hdmap_struct->road_boundary.empty());

  frame.lidar2world_pose = pose;
  EXPECT_TRUE(map_manager.Init());
  EXPECT_TRUE(map_manager.Update(option, &frame));

  pose << 0.851662646615178, -0.524032057797173, 0.00781815337634697,
      428801.15767301, 0.524071437039706, 0.851410292805536,
      -0.0212044080070471, 4439224.3480147, 0.00445533393739291,
      0.0221562731670225, 0.999744592149723, 36.32589260724, 0, 0, 0, 1;
  frame.lidar2world_pose = pose;
  EXPECT_TRUE(map_manager.Update(option, &frame));

  pose << 0.851692358111521, -0.523982363558931, 0.00791167351803519,
      428801.70514839, 0.524024934753017, 0.851445251624461,
      -0.0209484124002575, 4439224.6815986, 0.00424024241714873,
      0.0219875170083176, 0.999749253270185, 36.326540271038, 0, 0, 0, 1;
  frame.lidar2world_pose = pose;
  EXPECT_TRUE(map_manager.Update(option, &frame));

  pose << 0.851676190668703, -0.524020172895821, 0.00710699716678622,
      428802.25940968, 0.524046407485889, 0.851438054749313,
      -0.0207023299129349, 4439225.0198116, 0.00479727126899779,
      0.0213560778450621, 0.999760423352933, 36.32548315711, 0, 0, 0, 1;
  frame.lidar2world_pose = pose;
  EXPECT_TRUE(map_manager.Update(option, &frame));

  map_manager.hdmap_input_ = nullptr;
  EXPECT_FALSE(map_manager.Update(option, &frame));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
