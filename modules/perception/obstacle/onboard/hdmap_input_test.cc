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

#include "modules/perception/obstacle/onboard/hdmap_input.h"

#include <vector>
#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

TEST(HDMapInputTest, test_Init) {
  auto* hdmap_input = HDMapInput::instance();
  EXPECT_TRUE(hdmap_input->Init());

  FLAGS_base_map_filename = "not_exit_dir";
  EXPECT_FALSE(hdmap_input->Init());
}

TEST(HDMapInputTest, test_GetROI) {
  HdmapStructPtr hdmap;
  auto* hdmap_input = HDMapInput::instance();
  pcl_util::PointD velodyne_pose_world = {587054.96336391149,
                                          4141606.3593586856, 0.0};
  EXPECT_FALSE(
      hdmap_input->GetROI(velodyne_pose_world, FLAGS_map_radius, &hdmap));
  FLAGS_map_dir = "modules/map/data/sunnyvale_loop";
  FLAGS_base_map_filename = "base_map.bin";
  EXPECT_TRUE(hdmap_input->Init());
  EXPECT_TRUE(
      hdmap_input->GetROI(velodyne_pose_world, FLAGS_map_radius, &hdmap));
  EXPECT_TRUE(hdmap != nullptr);
}

}  // namespace perception
}  // namespace apollo
