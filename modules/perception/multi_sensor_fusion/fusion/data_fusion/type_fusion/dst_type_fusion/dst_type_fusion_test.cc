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
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"

#include <boost/format.hpp>

#include "gtest/gtest.h"

#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_data_manager.h"
#include "modules/perception/multi_sensor_fusion/common/camera_util.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(DstTypeFusionTest, test_update_with_measurement) {
  FLAGS_work_root =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  EXPECT_TRUE(algorithm::SensorManager::Instance()->Init());

  std::cout << "start init dst app\n";
  TypeFusionInitOptions options;
  bool flag = DstTypeFusion::Init(options);
  EXPECT_TRUE(flag);
  // create a track
  base::ObjectPtr base_lidar_object_ptr(new base::Object());
  base_lidar_object_ptr->type = base::ObjectType::VEHICLE;
  base_lidar_object_ptr->type_probs.resize(
      static_cast<size_t>(base::ObjectType::MAX_OBJECT_TYPE), 0.0f);
  base_lidar_object_ptr->type_probs = std::vector<float>(
      static_cast<size_t>(base::ObjectType::MAX_OBJECT_TYPE), 0.1f);
  base_lidar_object_ptr
      ->type_probs[static_cast<size_t>(base::ObjectType::VEHICLE)] = 0.6f;

  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->sensor_info.name = "velodyne_64";
  lidar_frame_ptr->sensor_info.type = base::SensorType::VELODYNE_64;

  SensorPtr lidar_sensor_ptr(new Sensor(lidar_frame_ptr->sensor_info));
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);

  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(lidar_frame_ptr, lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(base_lidar_object_ptr, lidar_sensor_frame_ptr));
  TrackPtr track(new Track());
  EXPECT_TRUE(DstTypeFusion::Init(options));

  EXPECT_TRUE(track->Initialize(lidar_sensor_object_ptr));

  DstTypeFusion type_fusion(track);
  CHECK_EQ(static_cast<size_t>(track->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));

  // create a camera measurement
  base::ObjectPtr base_camera_object_ptr(new base::Object());
  base_camera_object_ptr->type = base::ObjectType::PEDESTRIAN;
  base_camera_object_ptr->type_probs.resize(
      static_cast<size_t>(base::ObjectType::MAX_OBJECT_TYPE));
  base_camera_object_ptr->type_probs = std::vector<float>(
      static_cast<size_t>(base::ObjectType::MAX_OBJECT_TYPE), 0.1f);
  base_camera_object_ptr
      ->type_probs[static_cast<size_t>(base::ObjectType::PEDESTRIAN)] = 0.4f;

  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->sensor_info.type = base::SensorType::MONOCULAR_CAMERA;

  SensorPtr camera_sensor_ptr(new Sensor(camera_frame_ptr->sensor_info));
  camera_sensor_ptr->AddFrame(camera_frame_ptr);

  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(camera_frame_ptr, camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(base_camera_object_ptr, camera_sensor_frame_ptr));

  // camera not supported
  camera_frame_ptr->sensor_info.name = "camera_test";
  type_fusion.UpdateWithMeasurement(camera_sensor_object_ptr, 0.0);
  CHECK_EQ(static_cast<size_t>(track->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));

  camera_frame_ptr->sensor_info.name = "camera_smartereye";
  SensorFramePtr camera_sensor_frame_normal(new SensorFrame());
  SensorPtr camera_sensor_ptr1(new Sensor(camera_frame_ptr->sensor_info));
  camera_sensor_ptr1->AddFrame(camera_frame_ptr);
  camera_sensor_frame_normal->Initialize(camera_frame_ptr, camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr1(
      new SensorObject(base_camera_object_ptr, camera_sensor_frame_normal));
  // update with measurment
  type_fusion.UpdateWithMeasurement(camera_sensor_object_ptr1, 0.0);
  // std::cout<< "fused dst: " << type_fusion.PrintFusedDst();
  CHECK_EQ(static_cast<size_t>(track->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::PEDESTRIAN));
  // update more times
  type_fusion.UpdateWithMeasurement(camera_sensor_object_ptr1, 0.0);
  type_fusion.UpdateWithMeasurement(camera_sensor_object_ptr1, 0.0);
  CHECK_EQ(static_cast<size_t>(track->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::PEDESTRIAN));
}

TEST(DstTypeFusionTest, test_update_without_measurement) {
  FLAGS_work_root =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  algorithm::SensorManager *sensor_manager =
      algorithm::SensorManager::Instance();
  EXPECT_NE(sensor_manager, nullptr);

  EXPECT_TRUE(sensor_manager->Init());

  std::cout << "start init dst app\n";
    TypeFusionInitOptions options;
  bool flag = DstTypeFusion::Init(options);
  EXPECT_TRUE(flag);
  base::SensorInfo radar_front_info;
  EXPECT_TRUE(sensor_manager->GetSensorInfo("radar_front", &radar_front_info));
  SensorPtr radar_front_sensor(new Sensor(radar_front_info));
  // radar front frame
  base::ObjectPtr base_obj(new base::Object());
  base::FramePtr base_frame(new base::Frame());
  base_obj->center = Eigen::Vector3d(2.3, 2.3, 0.0);
  base_obj->anchor_point = base_obj->center;
  base_obj->velocity = Eigen::Vector3f(5, 0, 0);
  base_obj->direction = Eigen::Vector3f(1, 0, 0);
  base_obj->radar_supplement.range = 10;
  base_obj->radar_supplement.angle = 10;
  base_frame->objects.emplace_back(base_obj);
  base_frame->sensor_info = radar_front_info;
  SensorFramePtr radar_frame(new SensorFrame());
  radar_frame->Initialize(base_frame, radar_front_sensor);

  // create a lidar track
  base::ObjectPtr base_lidar_object_ptr(new base::Object());
  base_lidar_object_ptr->type = base::ObjectType::VEHICLE;
  base_lidar_object_ptr->type_probs.resize(
      static_cast<size_t>(base::ObjectType::MAX_OBJECT_TYPE), 0.0f);
  base_lidar_object_ptr->type_probs = std::vector<float>(
      static_cast<size_t>(base::ObjectType::MAX_OBJECT_TYPE), 0.1f);
  base_lidar_object_ptr
      ->type_probs[static_cast<size_t>(base::ObjectType::VEHICLE)] = 0.6f;
  base_lidar_object_ptr->center = Eigen::Vector3d(0, 0, 10);

  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->sensor_info.name = "velodyne_64";
  lidar_frame_ptr->sensor_info.type = base::SensorType::VELODYNE_64;

  SensorPtr lidar_sensor_ptr(new Sensor(lidar_frame_ptr->sensor_info));
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);

  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(lidar_frame_ptr, lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(base_lidar_object_ptr, lidar_sensor_frame_ptr));
  TrackPtr track1(new Track());
  EXPECT_TRUE(track1->Initialize(lidar_sensor_object_ptr));

  EXPECT_TRUE(DstTypeFusion::Init(options));
  DstTypeFusion type_fusion1(track1);
  CHECK_EQ(static_cast<size_t>(track1->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));

  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->sensor_info.name = "camera_smartereye";
  camera_frame_ptr->sensor_info.type = base::SensorType::MONOCULAR_CAMERA;
  camera_frame_ptr->timestamp = 151192277.124567989;
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  camera_frame_ptr->sensor2world_pose = pose;

  SensorPtr camera_sensor_ptr(new Sensor(camera_frame_ptr->sensor_info));
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorDataManager::Instance()->AddSensorMeasurements(camera_frame_ptr);

  // update without measurment
  std::string sensor_id = "camera_smartereye";
  // std::cout<< "fused dst: " << type_fusion1.PrintFusedDst();
  type_fusion1.UpdateWithoutMeasurement("velodyne_64", 151192277.124567989,
                                        151192277.124567989, 0.9);
  CHECK_EQ(static_cast<size_t>(track1->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));
  type_fusion1.UpdateWithoutMeasurement(sensor_id, 151192277.124567989,
                                        151192277.124567989, 0.9);
  CHECK_EQ(static_cast<size_t>(track1->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));
  type_fusion1.UpdateWithoutMeasurement("camera_test", 151192277.124567989,
                                        151192277.124567989, 0.9);
  CHECK_EQ(static_cast<size_t>(track1->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));

  // update more times
  type_fusion1.UpdateWithoutMeasurement(sensor_id, 151192277.124567989,
                                        151192277.124567989, 0.0);
  type_fusion1.UpdateWithoutMeasurement(sensor_id, 151192277.124567989,
                                        151192277.124567989, 0.0);
  type_fusion1.UpdateWithoutMeasurement(sensor_id, 0, 0, 0.0);
  CHECK_EQ(static_cast<size_t>(track1->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));

  TrackPtr track2(new Track());
  track2->Initialize(radar_frame->GetForegroundObjects()[0]);
  DstTypeFusion type_fusion_2(track2);
  type_fusion_2.UpdateWithoutMeasurement(sensor_id, 151192277.124567989,
                                         151192277.124567989, 0.0);
  CHECK_EQ(static_cast<size_t>(track1->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));

  base::ObjectPtr base_camera_object_ptr(new base::Object());

  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(camera_frame_ptr, camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(base_camera_object_ptr, camera_sensor_frame_ptr));
  TrackPtr track3(new Track());
  track3->Initialize(camera_sensor_object_ptr);
  DstTypeFusion type_fusion_3(track3);
  type_fusion_3.UpdateWithoutMeasurement(sensor_id, 151192277.124567989,
                                         151192277.124567989, 0.0);
  CHECK_EQ(static_cast<size_t>(track3->GetFusedObject()->GetBaseObject()->type),
           static_cast<size_t>(base::ObjectType::UNKNOWN));
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
