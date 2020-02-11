/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/app/lidar_obstacle_segmentation.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/pcl_util.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetFileName;

TEST(LidarObstacleSegmentationTest, init_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");

  FLAGS_work_root = "/apollo/modules/perception/testdata/lidar/app";
  FLAGS_config_manager_path = "./conf/perception/lidar";
  LidarObstacleSegmentationInitOptions segmentation_init_options;
  LidarObstacleSegmentation segmentation;
  segmentation_init_options.enable_hdmap_input = false;
  ASSERT_TRUE(segmentation.Init(segmentation_init_options));
}

TEST(LidarObstacleSegmentationTest, process_test) {
  unsetenv("MODULE_PATH");
  unsetenv("CYBER_PATH");

  FLAGS_work_root = "/apollo/modules/perception/testdata/lidar/app";
  FLAGS_config_manager_path = "./conf/perception/lidar";
  LidarObstacleSegmentationInitOptions segmentation_init_options;
  LidarObstacleSegmentation segmentation;
  segmentation_init_options.enable_hdmap_input = false;
  segmentation_init_options.sensor_name = "velodyne64";
  ASSERT_TRUE(segmentation.Init(segmentation_init_options));

  std::string pcd_path =
      "/apollo/modules/perception/testdata/lidar/app/data/perception/lidar/files/";
  std::string pose_path =
      "/apollo/modules/perception/testdata/lidar/app/data/perception/lidar/poses/";
  std::vector<std::string> pcd_file_names;
  common::GetFileList(pcd_path, ".pcd", &pcd_file_names);
  std::string file_name;
  std::sort(pcd_file_names.begin(), pcd_file_names.end(),
  [](const std::string& lhs, const std::string& rhs) {
    if (lhs.length() < rhs.length()) {
      return true;
    } else if (lhs.length() == rhs.length()) {
      return lhs <= rhs;
    } else {
      return false;
    }
  });

  for (size_t i = 0; i < pcd_file_names.size(); ++i) {
    int frame_id = 0;
    double timestamp = 0.0;
    file_name = GetFileName(pcd_file_names[i]);
    std::shared_ptr<LidarFrame> frame(new LidarFrame);
    frame->cloud = base::PointFCloudPool::Instance().Get();
    EXPECT_TRUE(
        LoadPCLPCD(pcd_path + "/" + file_name + ".pcd", frame->cloud.get()));
    EXPECT_TRUE(common::ReadPoseFile(pose_path + "/" + file_name + ".pose",
                                     &frame->lidar2world_pose, &frame_id,
                                     &timestamp));
    frame->timestamp = timestamp;
    LidarObstacleSegmentationOptions segmentation_options;
    EXPECT_EQ(
        segmentation.Process(segmentation_options, frame.get()).error_code,
        LidarErrorCode::Succeed);
    }
}

} // namespace lidar
} // namespace perception
} // namespace apollo