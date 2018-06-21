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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.h"

#include "gtest/gtest.h"

#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/fusion/interface/base_fusion.h"

namespace apollo {
namespace perception {

TEST(ProbabilisticFusionTest, probabilistic_fusion_test) {
  FLAGS_work_root = "modules/perception";
  FLAGS_config_manager_path = "./conf/config_manager.config";
  AINFO << "start probabilistic_fusion_test\n";
  ProbabilisticFusion *probabilistic_fusion = new ProbabilisticFusion();
  EXPECT_TRUE(probabilistic_fusion->Init());
  AINFO << "After fusion init";
  std::vector<SensorObjects> sensor_objects;
  std::vector<std::shared_ptr<Object>> fused_objects;
  sensor_objects.resize(1);
  sensor_objects[0].sensor_type = SensorType::VELODYNE_64;
  sensor_objects[0].seq_num = 0;
  sensor_objects[0].timestamp = 0.0;
  sensor_objects[0].sensor2world_pose = Eigen::Matrix4d::Identity();
  EXPECT_TRUE(probabilistic_fusion->Fuse(sensor_objects, &fused_objects));
  double timestamp = 0.0;
  std::shared_ptr<Object> moc_obj(new Object());
  Eigen::Vector3d position(20, 0, 0);
  Eigen::Vector3d dir(1, 0, 0);
  Eigen::Vector3d velocity(10, 0, 0);
  ObjectType type = ObjectType::VEHICLE;
  moc_obj->center = position;
  moc_obj->anchor_point = position;
  moc_obj->length = 4;
  moc_obj->width = 2;
  moc_obj->height = 2;
  moc_obj->velocity = velocity;
  moc_obj->track_id = 1;
  moc_obj->type = type;
  moc_obj->polygon.resize(1);
  moc_obj->polygon.points[0].x = position(0);
  moc_obj->polygon.points[0].y = position(1);
  moc_obj->polygon.points[0].z = position(2);

  std::vector<SensorObjects> sensor_objects2;
  sensor_objects2.resize(1);
  sensor_objects2[0].sensor_type = SensorType::RADAR;
  sensor_objects2[0].seq_num = 0;
  sensor_objects2[0].sensor2world_pose = Eigen::Matrix4d::Identity();
  std::shared_ptr<Object> obj(new Object());
  std::shared_ptr<Object> radar_obj(new Object());
  obj->clone(*moc_obj);
  for (int i = 0; i < 10; i++) {
    position = position + velocity * 0.05;
    timestamp += 0.05;
    radar_obj->center = position;
    radar_obj->anchor_point = position;
    radar_obj->velocity = velocity;
    radar_obj->track_id = 1;
    radar_obj->polygon.resize(1);
    radar_obj->polygon.points[0].x = position(0);
    radar_obj->polygon.points[0].y = position(1);
    radar_obj->polygon.points[0].z = position(2);
    sensor_objects2[0].timestamp = timestamp;
    sensor_objects2[0].objects.resize(1);
    sensor_objects2[0].objects[0] = radar_obj;
    EXPECT_TRUE(probabilistic_fusion->Fuse(sensor_objects2, &fused_objects));
    obj->clone(*moc_obj);
    position = position + velocity * 0.05;
    timestamp += 0.05;
    sensor_objects[0].timestamp = timestamp;
    obj->center = position;
    obj->anchor_point = position;
    obj->polygon.points[0].x = position(0);
    obj->polygon.points[0].y = position(1);
    obj->polygon.points[0].z = position(2);
    sensor_objects[0].objects.resize(1);
    sensor_objects[0].objects[0] = obj;
    EXPECT_TRUE(probabilistic_fusion->Fuse(sensor_objects, &fused_objects));
    EXPECT_EQ(fused_objects.size(), 1);
    EXPECT_DOUBLE_EQ(fused_objects[0]->length, obj->length);
    EXPECT_DOUBLE_EQ(fused_objects[0]->width, obj->width);
    EXPECT_DOUBLE_EQ(fused_objects[0]->height, obj->height);
    EXPECT_TRUE((fused_objects[0]->center - obj->center).norm() < 1.0e-2);
    EXPECT_TRUE((fused_objects[0]->velocity - obj->velocity).norm() < 1.0e-2);
  }
  AINFO << "end probabilistic_fusion_test\n";
}

}  // namespace perception
}  // namespace apollo
