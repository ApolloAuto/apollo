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

#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"

#include <boost/format.hpp>

#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_data_manager.h"
#include "modules/perception/multi_sensor_fusion/common/camera_util.h"

namespace apollo {
namespace perception {
namespace fusion {

/* TODO(all): Initialize() not compiling. to be fixed
TEST(DstExistFusionTest, test_update_with_measurement) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "fusion/dst_existence_fusion";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/dst_existence_fusion/params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  EXPECT_TRUE(common::SensorManager::Instance()->Init());

  std::cout << "start init dst app\n";
  bool flag = DstExistenceFusion::Init();
  EXPECT_TRUE(flag);

  // create a lidar track
  base::ObjectPtr base_lidar_object_ptr(new base::Object());

  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->sensor_info.name = "velodyne64";
  lidar_frame_ptr->sensor_info.type = base::SensorType::VELODYNE_64;

  SensorPtr lidar_sensor_ptr(new Sensor(lidar_frame_ptr->sensor_info));
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);

  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(lidar_frame_ptr, lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(base_lidar_object_ptr, lidar_sensor_frame_ptr));
  TrackPtr track(new Track());
  EXPECT_TRUE(track->Initialize(lidar_sensor_object_ptr));

  DstExistenceFusion existence_fusion(track);

  // create a camera measurement
  base::ObjectPtr base_camera_object_ptr(new base::Object());
  // base_camera_object_ptr->center = Eigen::Vector3d(0, 0, 10);

  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->sensor_info.name = "camera_smartereye";
  camera_frame_ptr->sensor_info.type = base::SensorType::STEREO_CAMERA;
  camera_frame_ptr->timestamp = 151192277.124567989;
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  camera_frame_ptr->sensor2world_pose = pose;

  SensorPtr camera_sensor_ptr(new Sensor(camera_frame_ptr->sensor_info));
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorDataManager::Instance()->AddSensorMeasurements(camera_frame_ptr);

  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(camera_frame_ptr, camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(base_camera_object_ptr, camera_sensor_frame_ptr));

  // update with measurment
  existence_fusion.UpdateWithMeasurement(camera_sensor_object_ptr,
                                         151192277.124567989, 0.9);
  EXPECT_NEAR(existence_fusion.GetToicScore(), 0.5, 1e-6);
  EXPECT_NEAR(existence_fusion.GetExistenceProbability(), 0.74, 1e-6);

  // create a radar track
  base::ObjectPtr base_radar_object_ptr(new base::Object());
  base_radar_object_ptr->center = Eigen::Vector3d(0, 0, 10);

  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->sensor_info.name = "radar";
  radar_frame_ptr->sensor_info.type = base::SensorType::LONG_RANGE_RADAR;

  SensorPtr radar_sensor_ptr(new Sensor(radar_frame_ptr->sensor_info));
  radar_sensor_ptr->AddFrame(radar_frame_ptr);

  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(radar_frame_ptr, radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(base_radar_object_ptr, radar_sensor_frame_ptr));
  TrackPtr track_1(new Track());
  EXPECT_TRUE(track_1->Initialize(radar_sensor_object_ptr));

  DstExistenceFusion existence_fusion_1(track_1);
  existence_fusion_1.UpdateWithMeasurement(camera_sensor_object_ptr,
                                           151192277.124567989, 0);
  EXPECT_NEAR(existence_fusion_1.GetToicScore(), 0.71, 1e-2);
  EXPECT_NEAR(existence_fusion_1.GetExistenceProbability(), 0.74, 1e-6);

  // radar will not update toic score
  existence_fusion_1.UpdateWithMeasurement(radar_sensor_object_ptr,
                                           151192277.124567989, 0);
  EXPECT_NEAR(existence_fusion_1.GetToicScore(), 0.71, 1e-2);
  EXPECT_NEAR(existence_fusion_1.GetExistenceProbability(), 0.74, 1e-3);
}
*/

/* TODO(all): Initialize() not compiling. to be fixed
TEST(DstExistFusionTest, test_update_without_measurement) {
  FLAGS_work_root = "./";
  FLAGS_obs_sensor_intrinsic_path = "./params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  EXPECT_TRUE(common::SensorManager::Instance()->Init());

  std::cout << "start init dst app\n";
  bool flag = DstExistenceFusion::Init();
  EXPECT_TRUE(flag);

  // create a lidar track
  base::ObjectPtr base_lidar_object_ptr(new base::Object());
  base_lidar_object_ptr->center = Eigen::Vector3d(0, 0, 10);

  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->sensor_info.name = "velodyne64";
  lidar_frame_ptr->sensor_info.type = base::SensorType::VELODYNE_64;

  SensorPtr lidar_sensor_ptr(new Sensor(lidar_frame_ptr->sensor_info));
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);

  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(lidar_frame_ptr, lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(base_lidar_object_ptr, lidar_sensor_frame_ptr));
  TrackPtr track(new Track());
  EXPECT_TRUE(track->Initialize(lidar_sensor_object_ptr));

  DstExistenceFusion existence_fusion(track);

  // create a camera measurement
  base::ObjectPtr base_camera_object_ptr(new base::Object());

  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->sensor_info.name = "camera_smartereye";
  camera_frame_ptr->sensor_info.type = base::SensorType::STEREO_CAMERA;
  camera_frame_ptr->timestamp = 151192277.124567989;
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  camera_frame_ptr->sensor2world_pose = pose;

  SensorPtr camera_sensor_ptr(new Sensor(camera_frame_ptr->sensor_info));
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorDataManager::Instance()->AddSensorMeasurements(camera_frame_ptr);

  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(camera_frame_ptr, camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(base_camera_object_ptr, camera_sensor_frame_ptr));

  // update without measurment
  std::string camera_sensor_id = camera_frame_ptr->sensor_info.name;
  std::string lidar_sensor_id = lidar_frame_ptr->sensor_info.name;
  existence_fusion.UpdateWithoutMeasurement(
      lidar_sensor_id, 151192277.124567989, 151192277.124567989, 0.9);
  EXPECT_NEAR(existence_fusion.GetExistenceProbability(), 0.455, 1e-6);
  existence_fusion.UpdateWithoutMeasurement(
      camera_sensor_id, 151192277.124567989, 151192277.124567989, 0.9);
  EXPECT_NEAR(existence_fusion.GetToicScore(), 0.5, 1e-6);
  EXPECT_NEAR(existence_fusion.GetExistenceProbability(), 0.455, 1e-6);

  // create a radar track
  base::ObjectPtr base_radar_object_ptr(new base::Object());
  base_radar_object_ptr->center = Eigen::Vector3d(0, 0, 10);

  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->sensor_info.name = "radar";
  radar_frame_ptr->sensor_info.type = base::SensorType::LONG_RANGE_RADAR;

  SensorPtr radar_sensor_ptr(new Sensor(radar_frame_ptr->sensor_info));
  radar_sensor_ptr->AddFrame(radar_frame_ptr);

  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(radar_frame_ptr, radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(base_radar_object_ptr, radar_sensor_frame_ptr));
  TrackPtr track_1(new Track());
  EXPECT_TRUE(track_1->Initialize(radar_sensor_object_ptr));

  DstExistenceFusion existence_fusion_1(track_1);
  existence_fusion_1.UpdateWithoutMeasurement(
      camera_sensor_id, 151192277.124567989, 151192277.124567989, 1.0);
  EXPECT_NEAR(existence_fusion_1.GetToicScore(), 0.5, 1e-2);
  EXPECT_NEAR(existence_fusion_1.GetExistenceProbability(), 0.5, 1e-3);
  existence_fusion_1.UpdateWithoutMeasurement("radar", 151192277.124567989,
                                              151192277.124567989, 0.0);

  // create a camera track
  TrackPtr track_2(new Track());
  EXPECT_TRUE(track_2->Initialize(camera_sensor_object_ptr));
  DstExistenceFusion existence_fusion_2(track_2);
  existence_fusion_2.UpdateWithoutMeasurement(
      camera_sensor_id, 151192277.124567989, 151192277.124567989, 1.0);
  EXPECT_NEAR(existence_fusion_1.GetToicScore(), 0.5, 1e-2);
}
*/

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
