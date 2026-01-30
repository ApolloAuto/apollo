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
#include "modules/perception/multi_sensor_fusion/base/sensor_data_manager.h"

#include "gtest/gtest.h"

#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(SensorDataManagerTest, test) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/fusion/base";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "fusion/base/params";

  SensorDataManager* sensor_data_manager = SensorDataManager::Instance();
  base::SensorInfo sensor_info_1;
  sensor_info_1.name = "velodyne64";
  sensor_info_1.type = base::SensorType::VELODYNE_64;

  double timestamp = 7012;
  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  base::ObjectPtr base_object_1(new base::Object());
  base::FramePtr base_frame_1(new base::Frame());
  base_frame_1->sensor_info = sensor_info_1;
  base_frame_1->timestamp = timestamp;
  base_frame_1->sensor2world_pose = sensor2world_pose;
  base_frame_1->objects.emplace_back(base_object_1);

  base::ObjectPtr base_object_2(new base::Object());
  base::FramePtr base_frame_2(new base::Frame());
  base_frame_2->sensor_info = sensor_info_1;
  timestamp = 9012;
  base_frame_2->timestamp = timestamp;
  base_frame_2->sensor2world_pose = sensor2world_pose;
  base_frame_2->objects.emplace_back(base_object_2);

  sensor_data_manager->AddSensorMeasurements(base_frame_1);
  EXPECT_EQ(sensor_data_manager->sensors_.size(), 1);
  sensor_data_manager->AddSensorMeasurements(base_frame_2);
  EXPECT_EQ(sensor_data_manager->sensors_.size(), 1);

  base::SensorInfo sensor_info_2;
  sensor_info_2.name = "camera_smartereye";
  sensor_info_2.type = base::SensorType::STEREO_CAMERA;

  base::ObjectPtr base_object_3(new base::Object());
  base::FramePtr base_frame_3(new base::Frame());
  base_frame_3->sensor_info = sensor_info_2;
  timestamp = 9014;
  base_frame_3->timestamp = timestamp;
  base_frame_3->sensor2world_pose = sensor2world_pose;
  base_frame_3->objects.emplace_back(base_object_3);

  sensor_data_manager->AddSensorMeasurements(base_frame_3);
  EXPECT_EQ(sensor_data_manager->sensors_.size(), 2);

  base::SensorInfo error_info;
  error_info.name = "error";
  base::FramePtr error_frame(new base::Frame());
  error_frame->sensor_info = error_info;
  sensor_data_manager->AddSensorMeasurements(error_frame);
  EXPECT_EQ(sensor_data_manager->sensors_.size(), 2);

  double query_timestamp = 9015;
  std::vector<SensorFramePtr> frame_vec;
  sensor_data_manager->GetLatestSensorFrames(query_timestamp, "velodyne64",
                                             nullptr);
  EXPECT_EQ(frame_vec.size(), 0);
  sensor_data_manager->GetLatestSensorFrames(query_timestamp, "ultrasonic",
                                             &frame_vec);
  EXPECT_EQ(frame_vec.size(), 0);
  sensor_data_manager->GetLatestSensorFrames(query_timestamp, "velodyne64",
                                             &frame_vec);
  EXPECT_EQ(frame_vec.size(), 2);

  frame_vec.clear();
  query_timestamp = 8000;
  sensor_data_manager->sensors_["velodyne64"]->SetLatestQueryTimestamp(0.0);
  sensor_data_manager->GetLatestFrames(query_timestamp, nullptr);
  EXPECT_EQ(frame_vec.size(), 0);
  sensor_data_manager->GetLatestFrames(query_timestamp, &frame_vec);
  EXPECT_EQ(frame_vec.size(), 1);
  query_timestamp = 9015;
  sensor_data_manager->GetLatestFrames(query_timestamp, &frame_vec);
  EXPECT_EQ(frame_vec.size(), 2);
  query_timestamp = 9018;
  sensor_data_manager->GetLatestFrames(query_timestamp, &frame_vec);
  EXPECT_EQ(frame_vec.size(), 0);

  Eigen::Affine3d pose;
  query_timestamp = 7012;
  EXPECT_FALSE(
      sensor_data_manager->GetPose("velodyne64", query_timestamp, nullptr));
  EXPECT_FALSE(
      sensor_data_manager->GetPose("ultrasonic", query_timestamp, &pose));
  EXPECT_TRUE(
      sensor_data_manager->GetPose("velodyne64", query_timestamp, &pose));
  EXPECT_EQ((pose.matrix() - sensor2world_pose.matrix()).trace(), 0.0);

  EXPECT_EQ(sensor_data_manager->GetCameraIntrinsic("velodyne64"), nullptr);
  EXPECT_NE(sensor_data_manager->GetCameraIntrinsic("camera_smartereye"),
            nullptr);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
