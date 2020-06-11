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

#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/fusion/base/sensor_object.h"
#include "modules/perception/fusion/base/track.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(TrackTest, test) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/fusion/base/";
  FLAGS_obs_sensor_meta_path = "./data/sensor_meta.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/"
      "fusion/base/params";
  base::SensorInfo vlp64_info;
  vlp64_info.name = "velodyne64";
  vlp64_info.type = base::SensorType::VELODYNE_64;
  SensorPtr vlp64_ptr(new Sensor(vlp64_info));

  base::SensorInfo radar_info;
  radar_info.name = "radar";
  radar_info.type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_ptr(new Sensor(radar_info));

  base::SensorInfo radar_rear_info;
  radar_rear_info.name = "radar_rear";
  radar_rear_info.type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_rear_ptr(new Sensor(radar_rear_info));

  base::SensorInfo camera_info;
  camera_info.name = "camera_smartereye";
  camera_info.type = base::SensorType::STEREO_CAMERA;
  SensorPtr camera_ptr(new Sensor(camera_info));

  Sensor::SetMaxCachedFrameNumber(2);

  Track::s_max_camera_invisible_period_ = 0.2;
  Track::s_max_lidar_invisible_period_ = 0.2;
  Track::s_max_radar_invisible_period_ = 0.2;

  double timestamp = 123456789.1;
  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  base::ObjectPtr base_obj_1(new base::Object());
  base_obj_1->latest_tracked_time = timestamp;
  base::FramePtr base_frame_1(new base::Frame());
  base_frame_1->timestamp = timestamp;
  base_frame_1->sensor2world_pose = sensor2world_pose;
  base_frame_1->objects.emplace_back(base_obj_1);
  base_frame_1->sensor_info = vlp64_info;
  SensorFramePtr frame_1(new SensorFrame());
  frame_1->Initialize(base_frame_1, vlp64_ptr);
  EXPECT_EQ(frame_1->GetForegroundObjects().size(), 1);

  base::ObjectPtr base_obj_2(new base::Object());
  base::FramePtr base_frame_2(new base::Frame());
  base_obj_2->latest_tracked_time = timestamp + 0.05;
  base_obj_2->center = Eigen::Vector3d(2.2, 2.2, 0.0);
  base_frame_2->timestamp = timestamp + 0.05;
  base_frame_2->sensor2world_pose = sensor2world_pose;
  base_frame_2->objects.emplace_back(base_obj_2);
  base_frame_2->sensor_info = radar_info;
  SensorFramePtr frame_2(new SensorFrame());
  frame_2->Initialize(base_frame_2, radar_ptr);
  EXPECT_EQ(frame_2->GetForegroundObjects().size(), 1);

  base::ObjectPtr base_obj_3(new base::Object());
  base::FramePtr base_frame_3(new base::Frame());
  base_obj_3->latest_tracked_time = timestamp + 0.07;
  base_obj_3->center = Eigen::Vector3d(2.5, 2.5, 0.0);
  base_frame_3->timestamp = timestamp + 0.07;
  base_frame_3->sensor2world_pose = sensor2world_pose;
  base_frame_3->objects.emplace_back(base_obj_3);
  base_frame_3->sensor_info = radar_rear_info;
  SensorFramePtr frame_3(new SensorFrame());
  frame_3->Initialize(base_frame_3, radar_rear_ptr);
  EXPECT_EQ(frame_3->GetForegroundObjects().size(), 1);

  base::ObjectPtr base_obj_4(new base::Object());
  base::FramePtr base_frame_4(new base::Frame());
  base_obj_4->latest_tracked_time = timestamp + 0.08;
  base_obj_4->center = Eigen::Vector3d(2.6, 2.6, 0.0);
  base_frame_4->timestamp = timestamp + 0.08;
  base_frame_4->sensor2world_pose = sensor2world_pose;
  base_frame_4->objects.emplace_back(base_obj_4);
  base_frame_4->sensor_info = camera_info;
  SensorFramePtr frame_4(new SensorFrame());
  frame_4->Initialize(base_frame_4, camera_ptr);
  EXPECT_EQ(frame_4->GetForegroundObjects().size(), 1);

  base::ObjectPtr base_obj_5(new base::Object());
  base::FramePtr base_frame_5(new base::Frame());
  base_obj_5->latest_tracked_time = timestamp + 0.1;
  base_obj_5->center = Eigen::Vector3d(2.9, 2.9, 0.0);
  base_frame_5->timestamp = timestamp + 0.1;
  base_frame_5->sensor2world_pose = sensor2world_pose;
  base_frame_5->objects.emplace_back(base_obj_5);
  base_frame_5->sensor_info = vlp64_info;
  SensorFramePtr frame_5(new SensorFrame());
  frame_5->Initialize(base_frame_5, vlp64_ptr);
  EXPECT_EQ(frame_5->GetForegroundObjects().size(), 1);

  base::ObjectPtr base_obj_6(new base::Object());
  base::FramePtr base_frame_6(new base::Frame());
  base_obj_6->latest_tracked_time = timestamp + 0.15;
  base_obj_6->center = Eigen::Vector3d(3.2, 3.2, 0.0);
  base_frame_6->timestamp = timestamp + 0.15;
  base_frame_6->sensor2world_pose = sensor2world_pose;
  base_frame_6->objects.emplace_back(base_obj_6);
  base_frame_6->sensor_info = radar_info;
  SensorFramePtr frame_6(new SensorFrame());
  frame_6->Initialize(base_frame_6, radar_ptr);
  EXPECT_EQ(frame_6->GetForegroundObjects().size(), 1);

  Track track;
  EXPECT_NE(track.GetFusedObject(), nullptr);
  EXPECT_EQ(track.GetTrackId(), -1);

  track.Initialize(frame_1->GetForegroundObjects()[0], false);
  EXPECT_NE(track.GetLatestLidarObject(), nullptr);
  EXPECT_EQ(track.GetTrackId(), 1);
  EXPECT_NE(track.GetSensorObject(vlp64_info.name), nullptr);

  track.UpdateWithSensorObject(frame_2->GetForegroundObjects()[0]);
  EXPECT_NE(track.GetLatestRadarObject(), nullptr);
  EXPECT_NE(track.GetSensorObject(radar_info.name), nullptr);

  track.UpdateWithSensorObject(frame_3->GetForegroundObjects()[0]);
  EXPECT_EQ(track.GetRadarObjects().size(), 2);
  SensorObjectConstPtr radar_obj = track.GetLatestRadarObject();
  EXPECT_NE(radar_obj, nullptr);
  EXPECT_NE(track.GetSensorObject(radar_rear_info.name), nullptr);

  track.UpdateWithSensorObject(frame_4->GetForegroundObjects()[0]);
  EXPECT_EQ(track.GetCameraObjects().size(), 1);
  EXPECT_NE(track.GetSensorObject(camera_info.name), nullptr);

  track.UpdateWithSensorObject(frame_5->GetForegroundObjects()[0]);
  EXPECT_EQ(track.GetLidarObjects().size(), 1);
  EXPECT_NE(track.GetSensorObject(vlp64_info.name), nullptr);

  track.UpdateWithSensorObject(frame_6->GetForegroundObjects()[0]);
  EXPECT_EQ(track.GetRadarObjects().size(), 2);

  EXPECT_TRUE(track.IsLidarVisible());
  EXPECT_TRUE(track.IsRadarVisible());
  EXPECT_TRUE(track.IsCameraVisible());

  EXPECT_EQ(track.GetSensorObject("incorrect_id"), nullptr);

  EXPECT_LT(
      fabs(track.GetTrackingPeriod() - frame_6->GetTimestamp() + timestamp),
      1e-6);

  track.UpdateWithoutSensorObject(vlp64_info.name, timestamp + 0.2);
  EXPECT_FALSE(track.IsVisible(vlp64_info.name));
  EXPECT_TRUE(track.IsVisible(camera_info.name));
  EXPECT_TRUE(track.IsVisible(radar_info.name));

  track.UpdateWithoutSensorObject(camera_info.name, timestamp + 0.25);
  EXPECT_FALSE(track.IsVisible(camera_info.name));

  track.UpdateWithoutSensorObject(radar_info.name, timestamp + 0.26);
  EXPECT_FALSE(track.IsVisible(radar_info.name));

  track.UpdateWithoutSensorObject(radar_info.name, timestamp + 0.36);
  EXPECT_EQ(track.GetRadarObjects().size(), 0);

  EXPECT_FALSE(track.IsVisible(vlp64_info.name));
  EXPECT_FALSE(track.IsLidarVisible());
  EXPECT_FALSE(track.IsRadarVisible());
  EXPECT_FALSE(track.IsCameraVisible());

  std::cout << track.GetLidarObjects().size() << " "
            << track.GetRadarObjects().size() << " "
            << track.GetCameraObjects().size() << "\n";
  EXPECT_FALSE(track.IsAlive());

  Track::s_track_idx_ = UINT_MAX;
  size_t new_id = Track::GenerateNewTrackId();
  new_id = Track::GenerateNewTrackId();
  EXPECT_EQ(new_id, 1);

  track.Reset();
  EXPECT_EQ(track.GetTrackId(), 0);
  EXPECT_NE(track.GetFusedObject(), nullptr);
  EXPECT_TRUE(track.GetLidarObjects().empty());
  EXPECT_TRUE(track.GetRadarObjects().empty());
  EXPECT_TRUE(track.GetCameraObjects().empty());
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
