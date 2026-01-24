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
#include "modules/perception/multi_sensor_fusion/fusion/fusion_system/probabilistic_fusion/probabilistic_fusion.h"

#include "gtest/gtest.h"

#include "modules/perception/common/base/sensor_meta.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/tracker/pbf_tracker/pbf_tracker.h"

namespace apollo {
namespace perception {
namespace fusion {

using apollo::perception::base::SensorInfo;

TEST(ProbabliticFusionTest, test_init) {
  FLAGS_work_root =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/data/params";
  SensorDataManager* sensor_manager = SensorDataManager::Instance();
  sensor_manager->Reset();
  sensor_manager->Init();
  FusionInitOptions init_options;
  ProbabilisticFusion pf;
  init_options.config_path = "perception/multi_sensor_fusion/data";
  init_options.config_file = "probabilistic_fusion.pb.txt";
  EXPECT_TRUE(pf.Init(init_options));
  EXPECT_EQ(pf.Name(), "ProbabilisticFusion");

  // sensor info
  std::shared_ptr<SensorInfo> sensor_info(new base::SensorInfo);
  sensor_info->type = base::SensorType::VELODYNE_64;
  sensor_info->name = "velodyne64";
  // radar_sensor_info
  std::shared_ptr<SensorInfo> radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  // camera_sensor_info
  std::shared_ptr<SensorInfo> camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";

  base::FramePtr sensor_frame_ptr(new base::Frame);
  sensor_frame_ptr->sensor_info = *sensor_info;
  sensor_frame_ptr->timestamp = 151192257.124567989;
  sensor_frame_ptr->sensor2world_pose = Eigen::Matrix4d::Identity();
  base::FrameConstPtr const_sensor_frame_ptr = sensor_frame_ptr;

  base::FramePtr sensor_frame_ptr2(new base::Frame);
  sensor_frame_ptr2->sensor_info = *radar_sensor_info;
  sensor_frame_ptr2->timestamp = 151192257.124567989;
  sensor_frame_ptr2->sensor2world_pose = Eigen::Matrix4d::Identity();
  base::FrameConstPtr const_sensor_frame_ptr2 = sensor_frame_ptr2;

  base::FramePtr sensor_frame_ptr3(new base::Frame);
  sensor_frame_ptr3->sensor_info = *camera_sensor_info;
  sensor_frame_ptr3->timestamp = 151192257.124567989;
  sensor_frame_ptr3->sensor2world_pose = Eigen::Matrix4d::Identity();
  base::FrameConstPtr const_sensor_frame_ptr3 = sensor_frame_ptr3;

  // test fuse coverage
  std::vector<base::ObjectPtr> fused_objects;
  pf.params_.use_lidar = false;
  bool state = pf.Fuse(const_sensor_frame_ptr, &fused_objects);
  EXPECT_TRUE(state);
  pf.params_.use_lidar = true;
  pf.params_.use_radar = false;
  state = pf.Fuse(const_sensor_frame_ptr2, &fused_objects);
  EXPECT_TRUE(state);
  pf.params_.use_radar = true;
  pf.params_.use_camera = false;
  state = pf.Fuse(const_sensor_frame_ptr3, &fused_objects);
  EXPECT_TRUE(state);
  pf.params_.use_camera = true;
  state = pf.Fuse(const_sensor_frame_ptr2, &fused_objects);
  EXPECT_TRUE(state);
  state = pf.Fuse(const_sensor_frame_ptr, &fused_objects);
  EXPECT_TRUE(state);
}

TEST(ProbabliticFusionTest, test_update) {
  FLAGS_work_root =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/data/params";
  SensorDataManager* sensor_manager = SensorDataManager::Instance();

  sensor_manager->Reset();
  sensor_manager->Init();
  FusionInitOptions init_options;
  ProbabilisticFusion pf;
  init_options.config_path = "perception/multi_sensor_fusion/data";
  init_options.config_file = "probabilistic_fusion.pb.txt";
  EXPECT_TRUE(pf.Init(init_options));
  EXPECT_EQ(pf.Name(), "ProbabilisticFusion");

  // sensor info
  std::shared_ptr<SensorInfo> sensor_info(new base::SensorInfo);
  sensor_info->type = base::SensorType::VELODYNE_64;
  sensor_info->name = "velodyne64";

  base::ObjectPtr base_track_lidar(new base::Object);
  base_track_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_track_lidar->track_id = 0;
  base_track_lidar->polygon.resize(3);
  for (size_t i = 0; i < 3; i++) {
    base_track_lidar->polygon[i].x = 10;
    base_track_lidar->polygon[i].y = 0;
    base_track_lidar->polygon[i].z = 0;
  }
  base_track_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base::ObjectPtr base_bg_track_lidar(new base::Object);
  *base_bg_track_lidar = *base_track_lidar;
  base_bg_track_lidar->track_id = 1;
  base_bg_track_lidar->lidar_supplement.on_use = true;
  base_bg_track_lidar->lidar_supplement.is_background = true;

  base::FramePtr lidar_track_frame(new base::Frame);
  lidar_track_frame->sensor_info = *sensor_info;
  lidar_track_frame->timestamp = 151192277.124567989;
  lidar_track_frame->sensor2world_pose = Eigen::Matrix4d::Identity();
  lidar_track_frame->objects.push_back(base_track_lidar);
  lidar_track_frame->objects.push_back(base_bg_track_lidar);

  std::vector<base::ObjectPtr> fused_objects;
  bool state = pf.Fuse(lidar_track_frame, &fused_objects);
  EXPECT_TRUE(state);
  EXPECT_EQ(pf.trackers_.size(), 1);
}

TEST(ProbabilisticFusionTest, test_collect_sensor_measurement) {
  FLAGS_work_root =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/data/params";
  SensorDataManager* sensor_manager = SensorDataManager::Instance();
  sensor_manager->Reset();
  sensor_manager->Init();
  FusionInitOptions init_options;
  ProbabilisticFusion pf;
  init_options.config_path = "perception/multi_sensor_fusion/data";
  init_options.config_file = "probabilistic_fusion.pb.txt";
  EXPECT_TRUE(pf.Init(init_options));
  EXPECT_EQ(pf.Name(), "ProbabilisticFusion");

  // sensor info
  std::shared_ptr<SensorInfo> sensor_info(new base::SensorInfo);
  sensor_info->type = base::SensorType::VELODYNE_64;
  sensor_info->name = "velodyne64";
  // radar_sensor_info
  std::shared_ptr<SensorInfo> radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  // camera_sensor_info
  std::shared_ptr<SensorInfo> camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";

  base::ObjectPtr base_track_lidar(new base::Object);
  base_track_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_track_lidar->track_id = 0;
  base_track_lidar->polygon.resize(4);
  for (size_t i = 0; i < 4; i++) {
    base_track_lidar->polygon[i].x = static_cast<double>(8 + 4 * (i % 2));
    base_track_lidar->polygon[i].y =
        static_cast<double>(-1 + 2 * ((i + 1) % 2));
    base_track_lidar->polygon[i].z = 0.0;
  }
  base_track_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_track_lidar->type = base::ObjectType::VEHICLE;
  base_track_lidar->size = Eigen::Vector3f(4, 2, 2);
  base_track_lidar->latest_tracked_time = 151192277.2024567989;

  base::FramePtr sensor_frame_ptr(new base::Frame);
  sensor_frame_ptr->objects.push_back(base_track_lidar);
  sensor_frame_ptr->sensor_info = *sensor_info;
  sensor_frame_ptr->timestamp = 151192277.2024567989;
  sensor_frame_ptr->sensor2world_pose = Eigen::Matrix4d::Identity();
  base::FrameConstPtr const_sensor_frame_ptr = sensor_frame_ptr;

  base::FramePtr lidar_frame_ptr2(new base::Frame);
  base::ObjectPtr base_track_lidar2(new base::Object);
  *base_track_lidar2 = *base_track_lidar;
  base_track_lidar2->latest_tracked_time += 0.1;
  lidar_frame_ptr2->objects.push_back(base_track_lidar2);
  lidar_frame_ptr2->sensor_info = *sensor_info;
  lidar_frame_ptr2->timestamp = base_track_lidar2->latest_tracked_time;
  lidar_frame_ptr2->sensor2world_pose = Eigen::Matrix4d::Identity();
  base::FrameConstPtr const_lidar_frame_ptr2 = lidar_frame_ptr2;

  base::ObjectPtr base_track_radar(new base::Object);
  base_track_radar->center = Eigen::Vector3d(10, 0, 0);
  base_track_radar->track_id = 0;
  base_track_radar->polygon.resize(4);
  for (size_t i = 0; i < 4; i++) {
    base_track_radar->polygon[i].x = static_cast<double>(8 + 4 * (i % 2));
    base_track_radar->polygon[i].y =
        static_cast<double>(-1 + 2 * ((i + 1) % 2));
    base_track_radar->polygon[i].z = 0.0;
  }
  base_track_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_track_radar->type = base::ObjectType::VEHICLE;
  base_track_radar->size = Eigen::Vector3f(4, 2, 2);
  base_track_radar->latest_tracked_time = 151192277.224567989;
  base_track_radar->radar_supplement.range = 10;
  base_track_radar->radar_supplement.on_use = true;
  base::FramePtr sensor_frame_ptr2(new base::Frame);
  sensor_frame_ptr2->objects.push_back(base_track_radar);
  sensor_frame_ptr2->sensor_info = *radar_sensor_info;
  sensor_frame_ptr2->timestamp = 151192277.224567989;
  sensor_frame_ptr2->sensor2world_pose = Eigen::Matrix4d::Identity();
  base::FrameConstPtr const_sensor_frame_ptr2 = sensor_frame_ptr2;

  base::ObjectPtr base_track_camera(new base::Object);
  base_track_camera->center = Eigen::Vector3d(60, 0, 0);
  base_track_camera->track_id = 0;
  base_track_camera->polygon.resize(4);
  for (size_t i = 0; i < 4; i++) {
    base_track_radar->polygon[i].x = static_cast<double>(58 + 4 * (i % 2));
    base_track_radar->polygon[i].y =
        static_cast<double>(-1 + 2 * ((i + 1) % 2));
    base_track_radar->polygon[i].z = 0.0;
  }
  base_track_camera->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_track_camera->type = base::ObjectType::VEHICLE;
  base_track_camera->size = Eigen::Vector3f(4, 2, 2);
  base_track_camera->latest_tracked_time = 151192277.244567989;
  base::FramePtr sensor_frame_ptr3(new base::Frame);
  sensor_frame_ptr3->sensor_info = *camera_sensor_info;
  sensor_frame_ptr3->timestamp = 151192277.244567989;
  sensor_frame_ptr3->objects.push_back(base_track_camera);
  sensor_frame_ptr3->sensor2world_pose = Eigen::Matrix4d::Identity();
  base::FrameConstPtr const_sensor_frame_ptr3 = sensor_frame_ptr3;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
