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
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"

#include "modules/perception/obstacle/onboard/hdmap_input.h"

namespace apollo {
namespace perception {

using std::vector;

class HDMapInputTest : public testing::Test {
 protected:
  HDMapInputTest() : hdmap_input_(NULL) {}
  virtual ~HDMapInputTest() {}
  virtual void SetUp() {
    hdmap_input_ = Singleton<HDMapInput>::Get();
    ASSERT_TRUE(hdmap_input_ != NULL);
  }

  HDMapInput* hdmap_input_;
};

TEST_F(HDMapInputTest, test_Init) {
  hdmap_input_->inited_ = true;
  EXPECT_TRUE(hdmap_input_->Init());

  hdmap_input_->inited_ = false;
  FLAGS_map_file_path = "not_exit_dir";
  EXPECT_FALSE(hdmap_input_->Init());
}

TEST_F(HDMapInputTest, test_GetROI) {
  HdmapStructPtr hdmap;
  pcl_util::PointD velodyne_pose_world = {587054.96336391149, 4141606.3593586856, 0.0};
  EXPECT_FALSE(hdmap_input_->GetROI(velodyne_pose_world, &hdmap));
  FLAGS_map_file_path = "modules/map/data/sunnyvale_loop.xml.txt";
  EXPECT_TRUE(hdmap_input_->Init());
  EXPECT_TRUE(hdmap_input_->GetROI(velodyne_pose_world, &hdmap));
  EXPECT_TRUE(hdmap != nullptr);
}

}  // namespace perception
}  // namespace apollo
