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

#include "modules/perception/multi_sensor_fusion/fusion/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"

#include "gtest/gtest.h"

#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/multi_sensor_fusion/base/sensor.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_object.h"
#include "modules/perception/multi_sensor_fusion/base/track.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(PbfGatekeeperTest, test) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "fusion/pbf_gatekeeper";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path = "/apollo/modules/perception/testdata/"
      "fusion/pbf_gatekeeper/params";

  PbfGatekeeper gate_keeper;
  EXPECT_EQ(gate_keeper.Name(), "PbfGatekeeper");
  EXPECT_TRUE(gate_keeper.Init());

  // case 1. lidar publish
  // case 2. radar publish
  // case 3. camera publish
  // case 4. invisible in all sensors, unable
  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  base::SensorInfo lidar_info;
  base::SensorInfo camera_obstacle_info;
  base::SensorInfo camera_narrow_info;
  base::SensorInfo radar_front_info;
  base::SensorInfo radar_rear_info;
  EXPECT_TRUE(sensor_manager->GetSensorInfo("velodyne64", &lidar_info));
  EXPECT_TRUE(sensor_manager->GetSensorInfo("radar_front", &radar_front_info));
  EXPECT_TRUE(
      sensor_manager->GetSensorInfo("front_6mm", &camera_obstacle_info));
  EXPECT_TRUE(
      sensor_manager->GetSensorInfo("front_12mm", &camera_narrow_info));
  EXPECT_TRUE(sensor_manager->GetSensorInfo("radar_rear", &radar_rear_info));

  SensorPtr lidar_sensor(new Sensor(lidar_info));
  SensorPtr radar_front_sensor(new Sensor(radar_front_info));
  SensorPtr radar_rear_sensor(new Sensor(radar_rear_info));
  SensorPtr camera_obstacle_sensor(new Sensor(camera_obstacle_info));
  SensorPtr camera_narrow_sensor(new Sensor(camera_narrow_info));

  Sensor::SetMaxCachedFrameNumber(2);

  // lidar frame
  double timestamp = 1526966680.88;
  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  base::ObjectPtr base_obj_1(new base::Object());
  base_obj_1->latest_tracked_time = timestamp;
  base::FramePtr base_frame_1(new base::Frame());
  base_frame_1->timestamp = timestamp;
  base_frame_1->sensor2world_pose = sensor2world_pose;
  base_frame_1->objects.emplace_back(base_obj_1);
  base_frame_1->sensor_info = lidar_info;
  SensorFramePtr frame_1(new SensorFrame());
  frame_1->Initialize(base_frame_1, lidar_sensor);

  // camera obstacle frame
  base::ObjectPtr base_obj_2(new base::Object());
  base::FramePtr base_frame_2(new base::Frame());
  base_obj_2->latest_tracked_time = timestamp + 0.05;
  base_obj_2->center = Eigen::Vector3d(2.2, 2.2, 0.0);
  base_frame_2->timestamp = timestamp + 0.05;
  base_frame_2->sensor2world_pose = sensor2world_pose;
  base_frame_2->objects.emplace_back(base_obj_2);
  base_frame_2->sensor_info = camera_obstacle_info;
  SensorFramePtr frame_2(new SensorFrame());
  frame_2->Initialize(base_frame_2, camera_obstacle_sensor);

  // camera narrow frame
  base::ObjectPtr base_obj_3(new base::Object());
  base::FramePtr base_frame_3(new base::Frame());
  base_obj_3->latest_tracked_time = timestamp + 0.15;
  base_obj_3->center = Eigen::Vector3d(2.2, 2.2, 0.0);
  base_obj_3->camera_supplement.local_center = Eigen::Vector3f(0, 0, 40);
  base_frame_3->timestamp = timestamp + 0.15;
  base_frame_3->sensor2world_pose = sensor2world_pose;
  base_frame_3->objects.emplace_back(base_obj_3);
  base_frame_3->sensor_info = camera_narrow_info;
  SensorFramePtr frame_3(new SensorFrame());
  frame_3->Initialize(base_frame_3, camera_narrow_sensor);

  // radar front frame
  base::ObjectPtr base_obj_4(new base::Object());
  base::FramePtr base_frame_4(new base::Frame());
  base_obj_4->latest_tracked_time = timestamp + 0.25;
  base_obj_4->center = Eigen::Vector3d(2.3, 2.3, 0.0);
  base_obj_4->anchor_point = base_obj_4->center;
  base_obj_4->velocity = Eigen::Vector3f(5, 0, 0);
  base_obj_4->direction = Eigen::Vector3f(1, 0, 0);
  base_obj_4->radar_supplement.range = 10;
  base_obj_4->radar_supplement.angle = 10;
  base_frame_4->timestamp = timestamp + 0.25;
  base_frame_4->sensor2world_pose = sensor2world_pose;
  base_frame_4->objects.emplace_back(base_obj_4);
  base_frame_4->sensor_info = radar_front_info;
  SensorFramePtr frame_4(new SensorFrame());
  frame_4->Initialize(base_frame_4, radar_front_sensor);

  // radar rear frame
  base::ObjectPtr base_obj_5(new base::Object());
  base::FramePtr base_frame_5(new base::Frame());
  base_obj_5->latest_tracked_time = timestamp + 0.55;
  base_obj_5->center = Eigen::Vector3d(2.4, 2.4, 0.0);
  base_obj_5->velocity = Eigen::Vector3f(8, 0, 0);
  base_obj_5->direction = Eigen::Vector3f(1, 0, 0);
  base_obj_5->radar_supplement.range = 20;
  base_obj_5->radar_supplement.angle = 10;
  base_frame_5->timestamp = timestamp + 0.55;
  base_frame_5->sensor2world_pose = sensor2world_pose;
  base_frame_5->objects.emplace_back(base_obj_5);
  base_frame_5->sensor_info = radar_rear_info;
  SensorFramePtr frame_5(new SensorFrame());
  frame_5->Initialize(base_frame_5, radar_rear_sensor);

  // lidar publish
  TrackPtr track1(new Track());
  track1->Initialize(frame_1->GetForegroundObjects()[0]);
  gate_keeper.params_.publish_if_has_lidar = false;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track1));
  gate_keeper.params_.publish_if_has_lidar = true;
  EXPECT_TRUE(gate_keeper.AbleToPublish(track1));

  track1->tracked_times_ = 0;
  gate_keeper.params_.use_track_time_pub_strategy = true;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track1));
  SensorObjectPtr sensor_object(new SensorObject(base_obj_1, frame_1));
  track1->UpdateWithSensorObject(sensor_object);
  EXPECT_TRUE(gate_keeper.AbleToPublish(track1));

  // invisible in any sensor
  gate_keeper.params_.use_track_time_pub_strategy = false;
  track1->UpdateWithoutSensorObject(lidar_info.name, timestamp + 0.55);
  EXPECT_FALSE(gate_keeper.AbleToPublish(track1));

  // radar publish
  TrackPtr track2(new Track());
  // front radar
  track2->Initialize(frame_4->GetForegroundObjects()[0]);
  track2->SetToicProb(0.82);
  gate_keeper.params_.publish_if_has_radar = false;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track2));
  gate_keeper.params_.publish_if_has_radar = true;
  gate_keeper.params_.min_radar_confident_distance = 5;
  gate_keeper.params_.max_radar_confident_angle = 5;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track2));
  gate_keeper.params_.max_radar_confident_angle = 20;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track2));
  EXPECT_FALSE(gate_keeper.RadarAbleToPublish(track2, true));
  gate_keeper.params_.toic_threshold = 0.9;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track2));
  gate_keeper.params_.toic_threshold = 0.8;
  gate_keeper.params_.min_radar_confident_distance = 50;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track2));
  // rear radar
  track2->Initialize(frame_5->GetForegroundObjects()[0]);
  gate_keeper.params_.min_radar_confident_distance = 5;
  EXPECT_TRUE(gate_keeper.AbleToPublish(track2));
  base_obj_5->velocity = Eigen::Vector3f(1, 0, 0);
  EXPECT_FALSE(gate_keeper.AbleToPublish(track2));
  base_obj_5->velocity = Eigen::Vector3f(10, 0, 0);
  gate_keeper.params_.min_radar_confident_distance = 50;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track2));

  // camera publish
  TrackPtr track3(new Track());
  // camera obstacle
  track3->Initialize(frame_2->GetForegroundObjects()[0]);
  EXPECT_FALSE(gate_keeper.AbleToPublish(track3));
  gate_keeper.params_.publish_if_has_camera = true;
  gate_keeper.params_.use_camera_3d = true;
  gate_keeper.params_.min_camera_publish_distance = 50;
  base_obj_2->camera_supplement.local_center = Eigen::Vector3f(0, 0, 40);
  EXPECT_FALSE(gate_keeper.AbleToPublish(track3));
  base_obj_2->camera_supplement.local_center = Eigen::Vector3f(0, 0, 60);
  EXPECT_FALSE(gate_keeper.AbleToPublish(track3));
  track3->SetExistenceProb(0.6);
  EXPECT_FALSE(gate_keeper.AbleToPublish(track3));
  track3->SetExistenceProb(0.8);
  EXPECT_TRUE(gate_keeper.AbleToPublish(track3));
  // camera narrow
  track3->Initialize(frame_3->GetForegroundObjects()[0]);
  gate_keeper.params_.min_camera_publish_distance = 10;
  gate_keeper.params_.use_camera_3d = false;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track3));
  gate_keeper.params_.use_camera_3d = true;
  EXPECT_TRUE(gate_keeper.AbleToPublish(track3));
  gate_keeper.params_.publish_if_has_camera = false;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track3));
  gate_keeper.params_.publish_if_has_camera = true;
  EXPECT_FALSE(gate_keeper.CameraAbleToPublish(track3, true));
  gate_keeper.params_.min_camera_publish_distance = 50;
  EXPECT_FALSE(gate_keeper.AbleToPublish(track3));
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
