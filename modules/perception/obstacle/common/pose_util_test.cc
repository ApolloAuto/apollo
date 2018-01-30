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

#include "modules/perception/obstacle/common/pose_util.h"

#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

class PoseUtilTest : public testing::Test {
 protected:
  PoseUtilTest() {}
  virtual ~PoseUtilTest() {}
  void SetUp() {}
  void TearDown() {}
};

TEST_F(PoseUtilTest, ReadPoseFile) {
  std::string data_path = "modules/perception/data/hm_tracker_test/";
  std::string file_name = "QN68P2_12_1476265365_1476265665_2.pose";
  std::string test_file = data_path + file_name;
  Eigen::Matrix4d pose;
  int frame_id;
  double time_stamp;
  EXPECT_TRUE(ReadPoseFile(test_file, &pose, &frame_id, &time_stamp));
  EXPECT_EQ(frame_id, 11989);
  EXPECT_EQ(time_stamp, 588.419051);
  EXPECT_EQ(pose(0, 3), 33.330463);
  EXPECT_EQ(pose(1, 3), 45.010161);
  EXPECT_EQ(pose(2, 3), 40.746964);
}

}  // namespace perception
}  // namespace apollo
