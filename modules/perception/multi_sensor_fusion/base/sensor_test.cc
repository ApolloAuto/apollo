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
#include "modules/perception/multi_sensor_fusion/base/sensor.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(SensorTest, test) {
  base::SensorInfo sensor_info;
  sensor_info.name = "test";
  sensor_info.type = base::SensorType::VELODYNE_64;
  SensorPtr sensor_ptr(new Sensor(sensor_info));

  Sensor::SetMaxCachedFrameNumber(2);

  double timestamp = 7012;
  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  base::ObjectPtr base_object(new base::Object());
  base::FramePtr base_frame(new base::Frame());
  base_frame->timestamp = timestamp;
  base_frame->sensor2world_pose = sensor2world_pose;
  base_frame->objects.emplace_back(base_object);

  double timestamp_2 = 9012;
  Eigen::Affine3d sensor2world_pose_2 = Eigen::Affine3d::Identity();
  base::ObjectPtr base_object_2(new base::Object());
  base::FramePtr base_frame_2(new base::Frame());
  base_frame_2->timestamp = timestamp_2;
  base_frame_2->sensor2world_pose = sensor2world_pose_2;
  base_frame_2->objects.emplace_back(base_object_2);

  sensor_ptr->AddFrame(base_frame);
  EXPECT_EQ(sensor_ptr->frames_.size(), 1);
  sensor_ptr->AddFrame(base_frame_2);
  EXPECT_EQ(sensor_ptr->frames_.size(), 2);
  sensor_ptr->AddFrame(base_frame);
  EXPECT_EQ(sensor_ptr->frames_.size(), 2);

  std::vector<SensorFramePtr> frame_vec;
  double query_timestamp = 7013;
  sensor_ptr->QueryLatestFrames(query_timestamp, &frame_vec);
  EXPECT_EQ(frame_vec.size(), 1);
  query_timestamp = 9013;
  sensor_ptr->QueryLatestFrames(query_timestamp, &frame_vec);
  EXPECT_EQ(frame_vec.size(), 1);

  sensor_ptr->latest_query_timestamp_ = 0;
  sensor_ptr->QueryLatestFrames(query_timestamp, &frame_vec);
  EXPECT_EQ(frame_vec.size(), 2);

  sensor_ptr->latest_query_timestamp_ = 0;
  query_timestamp = 7013;
  EXPECT_NE(sensor_ptr->QueryLatestFrame(query_timestamp), nullptr);
  query_timestamp = 9013;
  EXPECT_NE(sensor_ptr->QueryLatestFrame(query_timestamp), nullptr);
  query_timestamp = 7013;
  EXPECT_EQ(sensor_ptr->QueryLatestFrame(query_timestamp), nullptr);

  Eigen::Affine3d pose;
  query_timestamp = 7012;
  EXPECT_TRUE(sensor_ptr->GetPose(query_timestamp, &pose));
  EXPECT_EQ((pose.matrix() - sensor2world_pose.matrix()).trace(), 0.0);
  query_timestamp = 2012;
  EXPECT_FALSE(sensor_ptr->GetPose(query_timestamp, &pose));

  EXPECT_EQ(sensor_ptr->GetSensorId(), "test");
  EXPECT_EQ(sensor_ptr->GetSensorType(), base::SensorType::VELODYNE_64);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
