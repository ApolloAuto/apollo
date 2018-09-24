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
#include <gtest/gtest.h>

#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/fusion/base/sensor_object.h"

namespace apollo {
namespace perception {
namespace common {
DECLARE_string(obs_sensor_meta_path);
DECLARE_string(obs_sensor_intrinsic_path);
}

namespace lib {
DECLARE_string(work_root);
}

namespace fusion {

TEST(SensorObjectTest, test) {
  lib::FLAGS_work_root = "./fusion_test_data/base";
  common::FLAGS_obs_sensor_meta_path = "./data/sensor_meta.pt";
  common::FLAGS_obs_sensor_intrinsic_path = "./fusion_test_data/base/params";
  base::SensorInfo sensor_info;
  sensor_info.name = "test";
  sensor_info.type = base::SensorType::VELODYNE_64;
  SensorPtr sensor_ptr(new Sensor(sensor_info));

  double timestamp = 7012;
  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  base::ObjectPtr base_object(new base::Object());
  base::FramePtr base_frame(new base::Frame());
  base_frame->timestamp = timestamp;
  base_frame->sensor2world_pose = sensor2world_pose;
  base_frame->objects.emplace_back(base_object);
  SensorFramePtr frame_ptr(new SensorFrame());
  frame_ptr->Initialize(base_frame, sensor_ptr);

  SensorObjectPtr object(new SensorObject(base_object, frame_ptr));

  EXPECT_TRUE(IsLidar(object));
  EXPECT_FALSE(IsRadar(object));
  EXPECT_FALSE(IsCamera(object));

  Eigen::Affine3d pose;
  EXPECT_DOUBLE_EQ(object->GetTimestamp(), 7012);
  EXPECT_TRUE(object->GetRelatedFramePose(&pose));
  EXPECT_EQ((pose.matrix() - sensor2world_pose.matrix()).trace(), 0.0);
  EXPECT_EQ(object->GetSensorId(), "test");
  EXPECT_TRUE(object->GetBaseObject() != nullptr);

  frame_ptr.reset();
  EXPECT_DOUBLE_EQ(object->GetTimestamp(), 0);
  EXPECT_FALSE(object->GetRelatedFramePose(&pose));
  EXPECT_EQ(object->GetSensorId(), "");

  FusedObjectPtr fused_object(new FusedObject());
  EXPECT_TRUE(fused_object->GetBaseObject() != nullptr);
  EXPECT_DOUBLE_EQ(fused_object->GetTimestamp(), 0);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
