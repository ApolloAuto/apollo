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

#include "modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"

#include "gtest/gtest.h"

#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {
const double SHAPE_FUSION_PI = 3.1415926;

using SensorInfoPtr = std::shared_ptr<perception::base::SensorInfo>;
using FramePtr = std::shared_ptr<perception::base::Frame>;

TEST(PbfShapeFusion, lidar_track) {
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/fusion/pbf_shape_fusion";
  FLAGS_obs_sensor_meta_path = "./data/sensor_meta.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/pbf_shape_fusion/params";
  Eigen::Matrix4d pose;
  pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  SensorInfoPtr lidar_sensor_info(new base::SensorInfo);
  lidar_sensor_info->type = base::SensorType::VELODYNE_64;
  lidar_sensor_info->name = "VELODYNE_64";
  SensorPtr lidar_sensor(new Sensor(*lidar_sensor_info));

  SensorInfoPtr radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  SensorPtr radar_sensor(new Sensor(*radar_sensor_info));

  SensorInfoPtr camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";
  SensorPtr camera_sensor(new Sensor(*camera_sensor_info));

  // lidar track
  base::ObjectPtr base_track_lidar(new base::Object);
  base_track_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_track_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_track_lidar->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_track_lidar->direction = Eigen::Vector3f(0.707f, 0.707f, 0.0);
  base_track_lidar->size = Eigen::Vector3f(3.0f, 2.0f, 1.0f);
  std::vector<base::ObjectPtr> lidar_track_objects;
  lidar_track_objects.push_back(base_track_lidar);

  FramePtr lidar_track_frame(new base::Frame);
  lidar_track_frame->sensor_info = *lidar_sensor_info;
  lidar_track_frame->timestamp = 151192277.124567989;
  lidar_track_frame->sensor2world_pose = pose;
  lidar_track_frame->objects = lidar_track_objects;

  SensorFramePtr lidar_sensor_frame(new SensorFrame);
  lidar_sensor_frame->Initialize(lidar_track_frame, lidar_sensor);
  SensorObjectPtr lidar_obj(
      new SensorObject(base_track_lidar, lidar_sensor_frame));
  TrackPtr lidar_track(new Track());
  lidar_track->Initialize(lidar_obj, false);
  // lidar measurment
  base::ObjectPtr base_object_lidar(new base::Object);
  base_object_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->theta = 0.0;
  base_object_lidar->direction = Eigen::Vector3f(1.0f, 0, 0);
  base_object_lidar->size = Eigen::Vector3f(5.0f, 2.0f, 1.8f);
  std::vector<base::ObjectPtr> lidar_objects;
  lidar_objects.push_back(base_object_lidar);

  FramePtr lidar_frame(new base::Frame);
  lidar_frame->sensor_info = *lidar_sensor_info;
  lidar_frame->timestamp = 151192277.124567989;
  lidar_frame->sensor2world_pose = pose;
  lidar_frame->objects = lidar_objects;

  SensorFramePtr lidar_sensor_frame_2(new SensorFrame);
  lidar_sensor_frame_2->Initialize(lidar_frame, lidar_sensor);
  SensorObjectPtr lidar_measurement(
      new SensorObject(base_object_lidar, lidar_sensor_frame_2));
  // radar measurment
  base::ObjectPtr base_object_radar(new base::Object);
  base_object_radar->center = Eigen::Vector3d(10, 0, 0);
  base_object_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_radar->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_object_radar->direction = Eigen::Vector3f(0.707f, 0.707f, 0);
  base_object_radar->size = Eigen::Vector3f(2.0f, 1.0f, 3.0f);
  std::vector<base::ObjectPtr> radar_objects;
  radar_objects.push_back(base_object_radar);

  FramePtr radar_frame(new base::Frame);
  radar_frame->sensor_info = *radar_sensor_info;
  radar_frame->timestamp = 151192277.124567989;
  radar_frame->sensor2world_pose = pose;
  radar_frame->objects = radar_objects;

  SensorFramePtr radar_sensor_frame(new SensorFrame);
  radar_sensor_frame->Initialize(radar_frame, radar_sensor);
  SensorObjectPtr radar_measurement(
      new SensorObject(base_object_radar, radar_sensor_frame));
  // camera measurment
  base::ObjectPtr base_object_camera(new base::Object);
  base_object_camera->center = Eigen::Vector3d(10, 0, 0);
  base_object_camera->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_camera->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_object_camera->direction = Eigen::Vector3f(0.707f, 0.707f, 0);
  base_object_camera->size = Eigen::Vector3f(3.5f, 2.5f, 1.5f);
  std::vector<base::ObjectPtr> camera_objects;
  camera_objects.push_back(base_object_camera);

  FramePtr camera_frame(new base::Frame);
  camera_frame->sensor_info = *camera_sensor_info;
  camera_frame->timestamp = 151192277.124567989;
  camera_frame->sensor2world_pose = pose;
  camera_frame->objects = camera_objects;

  SensorFramePtr camera_sensor_frame(new SensorFrame);
  camera_sensor_frame->Initialize(camera_frame, camera_sensor);
  SensorObjectPtr camera_measurement(
      new SensorObject(base_object_camera, camera_sensor_frame));

  PbfShapeFusion shape_fusion(lidar_track);
  shape_fusion.UpdateWithMeasurement(lidar_measurement, 100.0);
  Eigen::Vector3f size(5.0f, 2.0f, 1.8f);
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta, 0.0);

  shape_fusion.UpdateWithMeasurement(radar_measurement, 100.0);
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta, 0.0);

  shape_fusion.UpdateWithMeasurement(camera_measurement, 100.0);
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta, 0.0);
}

TEST(PbfShapeFusion, radar_track) {
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/fusion/pbf_shape_fusion";
  FLAGS_obs_sensor_meta_path = "./conf/sensor_meta.config";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/pbf_shape_fusion/params";
  Eigen::Matrix4d pose;
  pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  SensorInfoPtr lidar_sensor_info(new base::SensorInfo);
  lidar_sensor_info->type = base::SensorType::VELODYNE_64;
  lidar_sensor_info->name = "VELODYNE_64";
  SensorPtr lidar_sensor(new Sensor(*lidar_sensor_info));

  SensorInfoPtr radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  SensorPtr radar_sensor(new Sensor(*radar_sensor_info));

  SensorInfoPtr camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";
  SensorPtr camera_sensor(new Sensor(*camera_sensor_info));

  SensorInfoPtr unknown_sensor_info(new base::SensorInfo);
  unknown_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  unknown_sensor_info->name = "UNKNOWN_SENSOR_TYPE";
  SensorPtr unknown_sensor(new Sensor(*unknown_sensor_info));

  // radar track
  base::ObjectPtr base_track_radar(new base::Object);
  base_track_radar->center = Eigen::Vector3d(10, 0, 0);
  base_track_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_track_radar->theta = 0.0;
  base_track_radar->direction = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  base_track_radar->size = Eigen::Vector3f(3.2f, 2.2f, 1.2f);
  std::vector<base::ObjectPtr> radar_track_objects;
  radar_track_objects.push_back(base_track_radar);

  FramePtr radar_track_frame(new base::Frame);
  radar_track_frame->sensor_info = *radar_sensor_info;
  radar_track_frame->timestamp = 151192277.124567989;
  radar_track_frame->sensor2world_pose = pose;
  radar_track_frame->objects = radar_track_objects;

  SensorFramePtr radar_sensor_frame(new SensorFrame);
  radar_sensor_frame->Initialize(radar_track_frame, radar_sensor);
  SensorObjectPtr radar_obj(
      new SensorObject(base_track_radar, radar_sensor_frame));
  TrackPtr radar_track(new Track());
  radar_track->Initialize(radar_obj, false);

  // lidar measurment
  base::ObjectPtr base_object_lidar(new base::Object);
  base_object_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->theta = 0.0;
  base_object_lidar->direction = Eigen::Vector3f(1.0f, 0, 0);
  base_object_lidar->size = Eigen::Vector3f(5.0f, 2.0f, 1.8f);
  std::vector<base::ObjectPtr> lidar_objects;
  lidar_objects.push_back(base_object_lidar);

  FramePtr lidar_frame(new base::Frame);
  lidar_frame->sensor_info = *lidar_sensor_info;
  lidar_frame->timestamp = 151192277.124567989;
  lidar_frame->sensor2world_pose = pose;
  lidar_frame->objects = lidar_objects;

  SensorFramePtr lidar_sensor_frame(new SensorFrame);
  lidar_sensor_frame->Initialize(lidar_frame, lidar_sensor);
  SensorObjectPtr lidar_measurement(
      new SensorObject(base_object_lidar, lidar_sensor_frame));
  // radar measurment
  base::ObjectPtr base_object_radar(new base::Object);
  base_object_radar->center = Eigen::Vector3d(10, 0, 0);
  base_object_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_radar->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_object_radar->direction = Eigen::Vector3f(0.707f, 0.707f, 0);
  base_object_radar->size = Eigen::Vector3f(2.0f, 1.0f, 3.0f);
  std::vector<base::ObjectPtr> radar_objects;
  radar_objects.push_back(base_object_radar);

  FramePtr radar_frame(new base::Frame);
  radar_frame->sensor_info = *radar_sensor_info;
  radar_frame->timestamp = 151192277.124567989;
  radar_frame->sensor2world_pose = pose;
  radar_frame->objects = radar_objects;

  SensorFramePtr radar_sensor_frame_2(new SensorFrame);
  radar_sensor_frame_2->Initialize(radar_frame, radar_sensor);
  SensorObjectPtr radar_measurement(
      new SensorObject(base_object_radar, radar_sensor_frame_2));
  // camera measurment
  base::ObjectPtr base_object_camera(new base::Object);
  base_object_camera->center = Eigen::Vector3d(10, 0, 0);
  base_object_camera->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_camera->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_object_camera->direction = Eigen::Vector3f(0.707f, 0.707f, 0);
  base_object_camera->size = Eigen::Vector3f(3.5f, 2.5f, 1.5f);
  std::vector<base::ObjectPtr> camera_objects;
  camera_objects.push_back(base_object_camera);

  FramePtr camera_frame(new base::Frame);
  camera_frame->sensor_info = *camera_sensor_info;
  camera_frame->timestamp = 151192277.124567989;
  camera_frame->sensor2world_pose = pose;
  camera_frame->objects = camera_objects;

  SensorFramePtr camera_sensor_frame(new SensorFrame);
  camera_sensor_frame->Initialize(camera_frame, camera_sensor);
  SensorObjectPtr camera_measurement(
      new SensorObject(base_object_camera, camera_sensor_frame));
  // unknown measurment
  base::ObjectPtr base_object_unknown(new base::Object);
  base_object_unknown->center = Eigen::Vector3d(10, 0, 0);
  base_object_unknown->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_unknown->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_object_unknown->direction = Eigen::Vector3f(0.707f, 0.707f, 0);
  base_object_unknown->size = Eigen::Vector3f(3.5f, 2.5f, 1.5f);
  std::vector<base::ObjectPtr> unknown_objects;
  unknown_objects.push_back(base_object_unknown);

  FramePtr unknown_frame(new base::Frame);
  unknown_frame->sensor_info = *unknown_sensor_info;
  unknown_frame->timestamp = 151192277.124567989;
  unknown_frame->sensor2world_pose = pose;
  unknown_frame->objects = unknown_objects;

  SensorFramePtr unknown_sensor_frame(new SensorFrame);
  unknown_sensor_frame->Initialize(unknown_frame, unknown_sensor);
  SensorObjectPtr unknown_measurement(
      new SensorObject(base_object_unknown, unknown_sensor_frame));

  PbfShapeFusion shape_fusion(radar_track);
  shape_fusion.UpdateWithMeasurement(lidar_measurement, 100.0);
  Eigen::Vector3f size(5.0f, 2.0f, 1.8f);
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta, 0.0);

  shape_fusion.UpdateWithMeasurement(radar_measurement, 100.0);
  size << 2.0f, 1.0f, 3.0f;
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta,
      SHAPE_FUSION_PI / 4);

  shape_fusion.UpdateWithMeasurement(camera_measurement, 100.0);
  size << 3.5f, 2.5f, 1.5f;
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta,
      SHAPE_FUSION_PI / 4);
  shape_fusion.UpdateWithMeasurement(unknown_measurement, 100.0);
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta,
      SHAPE_FUSION_PI / 4);

  EXPECT_TRUE(shape_fusion.Init());
  shape_fusion.UpdateWithoutMeasurement("VELODYNE_64", 100.0, 200.0);
  EXPECT_EQ(shape_fusion.Name(), "PbfShapeFusion");
}

TEST(PbfShapeFusion, camera_track) {
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/fusion/pbf_shape_fusion";
  FLAGS_obs_sensor_meta_path = "./conf/sensor_meta.config";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/pbf_shape_fusion/params";
  Eigen::Matrix4d pose;
  pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  SensorInfoPtr lidar_sensor_info(new base::SensorInfo);
  lidar_sensor_info->type = base::SensorType::VELODYNE_64;
  lidar_sensor_info->name = "VELODYNE_64";
  SensorPtr lidar_sensor(new Sensor(*lidar_sensor_info));

  SensorInfoPtr radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  SensorPtr radar_sensor(new Sensor(*radar_sensor_info));

  SensorInfoPtr camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";
  SensorPtr camera_sensor(new Sensor(*camera_sensor_info));

  // camera track
  base::ObjectPtr base_track_camera(new base::Object);
  base_track_camera->center = Eigen::Vector3d(10.0, 0, 0);
  base_track_camera->anchor_point = Eigen::Vector3d(10.0, 0, 0);
  base_track_camera->theta = static_cast<float>(SHAPE_FUSION_PI / 2.0);
  base_track_camera->direction = Eigen::Vector3f(0.0, 1.0, 0.0);
  base_track_camera->size = Eigen::Vector3f(3.1f, 2.1f, 1.1f);
  std::vector<base::ObjectPtr> camera_track_objects;
  camera_track_objects.push_back(base_track_camera);

  FramePtr camera_track_frame(new base::Frame);
  camera_track_frame->sensor_info = *camera_sensor_info;
  camera_track_frame->timestamp = 151192277.124567989;
  camera_track_frame->sensor2world_pose = pose;
  camera_track_frame->objects = camera_track_objects;

  SensorFramePtr camera_sensor_frame(new SensorFrame);
  camera_sensor_frame->Initialize(camera_track_frame, camera_sensor);
  SensorObjectPtr camera_obj(
      new SensorObject(base_track_camera, camera_sensor_frame));
  TrackPtr camera_track(new Track());
  camera_track->Initialize(camera_obj, false);

  // lidar measurment
  base::ObjectPtr base_object_lidar(new base::Object);
  base_object_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->theta = 0.0;
  base_object_lidar->direction = Eigen::Vector3f(1, 0, 0);
  base_object_lidar->size = Eigen::Vector3f(5.0f, 2.0f, 1.8f);
  std::vector<base::ObjectPtr> lidar_objects;
  lidar_objects.push_back(base_object_lidar);

  FramePtr lidar_frame(new base::Frame);
  lidar_frame->sensor_info = *lidar_sensor_info;
  lidar_frame->timestamp = 151192277.124567989;
  lidar_frame->sensor2world_pose = pose;
  lidar_frame->objects = lidar_objects;

  SensorFramePtr lidar_sensor_frame(new SensorFrame);
  lidar_sensor_frame->Initialize(lidar_frame, lidar_sensor);
  SensorObjectPtr lidar_measurement(
      new SensorObject(base_object_lidar, lidar_sensor_frame));
  // radar measurment
  base::ObjectPtr base_object_radar(new base::Object);
  base_object_radar->center = Eigen::Vector3d(10, 0, 0);
  base_object_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_radar->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_object_radar->direction = Eigen::Vector3f(0.707f, 0.707f, 0);
  base_object_radar->size = Eigen::Vector3f(2.0, 1.0, 3.0);
  std::vector<base::ObjectPtr> radar_objects;
  radar_objects.push_back(base_object_radar);

  FramePtr radar_frame(new base::Frame);
  radar_frame->sensor_info = *radar_sensor_info;
  radar_frame->timestamp = 151192277.124567989;
  radar_frame->sensor2world_pose = pose;
  radar_frame->objects = radar_objects;

  SensorFramePtr radar_sensor_frame(new SensorFrame);
  radar_sensor_frame->Initialize(radar_frame, radar_sensor);
  SensorObjectPtr radar_measurement(
      new SensorObject(base_object_radar, radar_sensor_frame));
  FramePtr radar_frame_2(new base::Frame);
  radar_frame_2->sensor_info = *radar_sensor_info;
  radar_frame_2->timestamp = 151192278.124567989;
  radar_frame_2->sensor2world_pose = pose;
  radar_frame_2->objects = radar_objects;

  SensorFramePtr radar_sensor_frame_2(new SensorFrame);
  radar_sensor_frame_2->Initialize(radar_frame_2, radar_sensor);
  SensorObjectPtr radar_measurement_2(
      new SensorObject(base_object_radar, radar_sensor_frame_2));
  // camera measurment
  base::ObjectPtr base_object_camera(new base::Object);
  base_object_camera->center = Eigen::Vector3d(10, 0, 0);
  base_object_camera->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_camera->theta = static_cast<float>(SHAPE_FUSION_PI / 4.0);
  base_object_camera->direction = Eigen::Vector3f(0.707f, 0.707f, 0.0f);
  base_object_camera->size = Eigen::Vector3f(3.5f, 2.5f, 1.5f);
  std::vector<base::ObjectPtr> camera_objects;
  camera_objects.push_back(base_object_camera);

  FramePtr camera_frame(new base::Frame);
  camera_frame->sensor_info = *camera_sensor_info;
  camera_frame->timestamp = 151192277.124567989;
  camera_frame->sensor2world_pose = pose;
  camera_frame->objects = camera_objects;

  SensorFramePtr camera_sensor_frame_2(new SensorFrame);
  camera_sensor_frame_2->Initialize(camera_frame, camera_sensor);
  SensorObjectPtr camera_measurement(
      new SensorObject(base_object_camera, camera_sensor_frame_2));

  PbfShapeFusion shape_fusion(camera_track);
  shape_fusion.UpdateWithMeasurement(lidar_measurement, 100.0);
  Eigen::Vector3f size(5.0f, 2.0f, 1.8f);
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta, 0.0);

  shape_fusion.UpdateWithMeasurement(radar_measurement, 100.0);
  size << 3.1f, 2.1f, 1.1f;
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta,
      SHAPE_FUSION_PI / 2);

  shape_fusion.UpdateWithMeasurement(camera_measurement, 100.0);
  size << 3.5f, 2.5f, 1.5f;
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta,
      SHAPE_FUSION_PI / 4);

  shape_fusion.UpdateWithMeasurement(radar_measurement_2, 100.0);
  EXPECT_LT(
      (size - shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->size)
          .norm(),
      1e-6);
  EXPECT_FLOAT_EQ(
      shape_fusion.GetTrack()->GetFusedObject()->GetBaseObject()->theta,
      SHAPE_FUSION_PI / 4);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
