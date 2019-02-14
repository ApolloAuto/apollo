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

#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"

DECLARE_string(work_root);

namespace apollo {
namespace perception {
namespace lidar {

class PointCloudPreprocessorTest : public testing::Test {
 protected:
  void SetUp() {
    char cyber_path[100] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[100] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_work_root =
        "/apollo/modules/perception/testdata/"
        "lidar/lib/pointcloud_preprocessor";
  }
  void TearDown() {}

 protected:
  PointCloudPreprocessor preprocessor;
};

void MockPointcloud(base::PointFCloud* cloud) {
  cloud->resize(10);
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud->at(i).x = 5.f * i;
    cloud->at(i).y = 5.f * i;
    cloud->at(i).z = 0.f;
  }
  // 1. three nan points
  cloud->at(0).x = std::numeric_limits<float>::quiet_NaN();
  cloud->at(1).y = std::numeric_limits<float>::quiet_NaN();
  cloud->at(2).z = std::numeric_limits<float>::quiet_NaN();
  // 2. three inf points
  cloud->at(3).x = 10000.f;
  cloud->at(4).y = 10000.f;
  cloud->at(5).z = 10000.f;
  // 3. one box points
  cloud->at(6).x = cloud->at(6).y = 0.f;
  // 4. one large z points
  cloud->at(7).z = 10.f;
  // 5. two normal points
}
#ifdef PERCEPTION_LIDAR_USE_COMMON_MESSAGE
void MockMessage(adu::common::sensor::PointCloud* message) {
  message->set_measurement_time(0.0);
  for (size_t i = 0; i < 10; ++i) {
    message->add_point();
    message->mutable_point(i)->set_x(5.f * i);
    message->mutable_point(i)->set_y(5.f * i);
    message->mutable_point(i)->set_z(0.f);
  }
  // 1. three nan points
  message->mutable_point(0)->set_x(std::numeric_limits<float>::quiet_NaN());
  message->mutable_point(1)->set_y(std::numeric_limits<float>::quiet_NaN());
  message->mutable_point(2)->set_z(std::numeric_limits<float>::quiet_NaN());
  // 2. three inf points
  message->mutable_point(3)->set_x(10000.f);
  message->mutable_point(4)->set_y(10000.f);
  message->mutable_point(5)->set_z(10000.f);
  // 3. one box points
  message->mutable_point(6)->set_x(0.f);
  message->mutable_point(6)->set_y(0.f);
  // 4. one large z points
  message->mutable_point(7)->set_z(10.f);
  // 5. two normal points
}
#endif

TEST_F(PointCloudPreprocessorTest, basic_test) {
  EXPECT_EQ(preprocessor.Name(), "PointCloudPreprocessor");
  EXPECT_TRUE(preprocessor.Init());
  PointCloudPreprocessorOptions option;
  {
    LidarFrame frame;
    EXPECT_FALSE(preprocessor.Preprocess(option, nullptr));
    EXPECT_FALSE(preprocessor.Preprocess(option, &frame));
    frame.cloud = base::PointFCloudPool::Instance().Get();
    frame.lidar2world_pose = Eigen::Affine3d::Identity();
    option.sensor2novatel_extrinsics = Eigen::Affine3d::Identity();
    MockPointcloud(frame.cloud.get());
    EXPECT_EQ(frame.cloud->size(), 10);
    EXPECT_TRUE(preprocessor.Preprocess(option, &frame));
    EXPECT_EQ(frame.cloud->size(), 2);
    EXPECT_EQ(frame.world_cloud->size(), 2);
    for (size_t i = 0; i < frame.cloud->size(); ++i) {
      auto& pt = frame.cloud->at(i);
      auto& world_pt = frame.world_cloud->at(i);
      EXPECT_EQ(pt.x, world_pt.x);
      EXPECT_EQ(pt.y, world_pt.y);
      EXPECT_EQ(pt.z, world_pt.z);
      EXPECT_EQ(pt.intensity, world_pt.intensity);
      EXPECT_EQ(frame.cloud->points_beam_id()[i],
                frame.world_cloud->points_beam_id()[i]);
    }
  }
#ifdef PERCEPTION_LIDAR_USE_COMMON_MESSAGE
  {
    std::shared_ptr<adu::common::sensor::PointCloud> message(
        new adu::common::sensor::PointCloud);
    LidarFrame frame;
    EXPECT_FALSE(preprocessor.Preprocess(option, message, nullptr));
    MockMessage(message.get());
    EXPECT_EQ(message->point_size(), 10);
    frame.lidar2world_pose = Eigen::Affine3d::Identity();
    EXPECT_TRUE(preprocessor.Preprocess(option, message, &frame));
    EXPECT_EQ(frame.cloud->size(), 2);
    EXPECT_EQ(frame.world_cloud->size(), 2);
  }
#endif
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
