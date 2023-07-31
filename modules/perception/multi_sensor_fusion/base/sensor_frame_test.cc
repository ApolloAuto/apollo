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
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"

#include "gtest/gtest.h"

#include "modules/perception/multi_sensor_fusion/base/sensor.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(SensorFrameTest, test) {
  base::SensorInfo sensor_info;
  sensor_info.name = "test";
  sensor_info.type = base::SensorType::VELODYNE_64;
  SensorPtr sensor_ptr(new Sensor(sensor_info));

  double timestamp = 7012;
  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  base::ObjectPtr base_object(new base::Object());
  base::ObjectPtr base_object_2(new base::Object());
  base_object_2->lidar_supplement.is_background = true;
  base::FramePtr base_frame(new base::Frame());
  base_frame->timestamp = timestamp;
  base_frame->sensor2world_pose = sensor2world_pose;
  base_frame->sensor_info = sensor_info;
  base_frame->objects.emplace_back(base_object);
  base_frame->objects.emplace_back(base_object_2);

  SensorFramePtr frame_ptr(new SensorFrame());
  frame_ptr->Initialize(base_frame, sensor_ptr);
  EXPECT_EQ(frame_ptr->GetForegroundObjects().size(), 1);
  EXPECT_EQ(frame_ptr->GetBackgroundObjects().size(), 1);

  Eigen::Affine3d pose;
  EXPECT_DOUBLE_EQ(frame_ptr->GetTimestamp(), 7012);
  EXPECT_TRUE(frame_ptr->GetPose(&pose));
  EXPECT_EQ((pose.matrix() - sensor2world_pose.matrix()).trace(), 0.0);
  EXPECT_EQ(frame_ptr->GetSensorId(), "test");
  EXPECT_EQ(frame_ptr->GetSensorType(), base::SensorType::VELODYNE_64);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
