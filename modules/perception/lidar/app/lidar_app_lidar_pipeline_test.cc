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
#include "gtest/gtest.h"

#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/app/lidar_obstacle_segmentation.h"
#include "modules/perception/lidar/app/lidar_obstacle_tracking.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
// #include "modules/perception/lidar/common/pcl_util.h"

namespace apollo {
namespace perception {
namespace lib {
// DECLARE_string(work_root);
}

namespace lidar {

class LidarAppPipelineTest : public testing::Test {
 protected:
  void SetUp() {
    char cyber_path[100] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[100] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_work_root = "/apollo/modules/perception/testdata/lidar/app";
  }

  void TearDown() {}

  LidarObstacleSegmentation segmentation_;
  LidarObstacleTracking tracking_;
};  // class DecisionForestClassifierTest

#ifdef PERCEPTION_LIDAR_USE_COMMON_MESSAGE
void ToMessage(const base::PointFCloud& cloud,
               adu::common::sensor::PointCloud* message) {
  message->set_measurement_time(0.0);
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& pt = cloud[i];
    message->add_point();
    message->mutable_point(i)->set_x(pt.x);
    message->mutable_point(i)->set_y(pt.y);
    message->mutable_point(i)->set_z(pt.z);
    message->mutable_point(i)->set_intensity(pt.intensity);
    message->mutable_point(i)->set_stamp(
        static_cast<uint64_t>(cloud.points_timestamp(i) * 1e9));
  }
}
#endif

/*
TEST_F(LidarAppPipelineTest, lidar_app_pipeline_test) {
  std::string pcd_path =
      "/apollo/modules/perception/testdata/lidar/app/data/perception/lidar/files/";
  std::string pose_path =
      "modules/perception/testdata/lidar/app/data/perception/lidar/poses/";
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
  LidarObstacleSegmentationInitOptions segmentation_init_options;
  LidarObstacleTrackingInitOptions tracking_init_options;
  EXPECT_TRUE(segmentation_.Init(segmentation_init_options));
  EXPECT_TRUE(tracking_.Init(tracking_init_options));
  for (size_t i = 0; i < pcd_file_names.size(); ++i) {
    int frame_id = 0;
    double timestamp = 0.0;
    lib::FileUtil::GetFileName(pcd_file_names[i], &file_name);
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
        segmentation_.Process(segmentation_options, frame.get()).error_code,
        LidarErrorCode::Succeed);
    LidarObstacleTrackingOptions tracking_options;
    EXPECT_EQ(tracking_.Process(tracking_options, frame.get()).error_code,
              LidarErrorCode::Succeed);
  }
}

#ifdef PERCEPTION_LIDAR_USE_COMMON_MESSAGE
TEST_F(LidarAppPipelineTest, lidar_app_pipeline_test2) {
  std::string pcd_path = "./data/perception/lidar/files/";
  std::string pose_path = "./data/perception/lidar/poses/";
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
  LidarObstacleSegmentationInitOptions segmentation_init_options;
  LidarObstacleTrackingInitOptions tracking_init_options;
  EXPECT_TRUE(segmentation_.Init(segmentation_init_options));
  EXPECT_TRUE(tracking_.Init(tracking_init_options));
  for (size_t i = 0; i < pcd_file_names.size(); ++i) {
    int frame_id = 0;
    double timestamp = 0.0;
    lib::FileUtil::GetFileName(pcd_file_names[i], &file_name);
    std::shared_ptr<LidarFrame> frame(new LidarFrame);
    frame->cloud = base::PointFCloudPool::Instance().Get();
    EXPECT_TRUE(
        LoadPCLPCD(pcd_path + "/" + file_name + ".pcd", frame->cloud.get()));
    EXPECT_TRUE(common::ReadPoseFile(pose_path + "/" + file_name + ".pose",
                                     &frame->lidar2world_pose, &frame_id,
                                     &timestamp));
    std::shared_ptr<adu::common::sensor::PointCloud> message(
        new adu::common::sensor::PointCloud);
    ToMessage(*frame->cloud, message.get());
    frame->timestamp = timestamp;
    LidarObstacleSegmentationOptions segmentation_options;
    EXPECT_EQ(segmentation_.Process(segmentation_options, message, frame.get())
                  .error_code,
              LidarErrorCode::Succeed);
    LidarObstacleTrackingOptions tracking_options;
    EXPECT_EQ(tracking_.Process(tracking_options, frame.get()).error_code,
              LidarErrorCode::Succeed);
  }
}
#endif
*/

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
