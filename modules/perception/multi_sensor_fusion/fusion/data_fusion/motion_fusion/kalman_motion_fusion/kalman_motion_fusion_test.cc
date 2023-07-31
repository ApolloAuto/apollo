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
#include "modules/perception/multi_sensor_fusion/fusion/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/multi_sensor_fusion/base/sensor.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_object.h"
#include "modules/perception/multi_sensor_fusion/base/track.h"

namespace apollo {
namespace perception {
namespace fusion {
const double SHAPE_FUSION_PI = 3.1415926;

/* TODO(all): not compiling. to be fixed
TEST(MotionFusionTest, lidar_test) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "fusion/kalman_motion_fusion";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/kalman_motion_fusion/params";
  Eigen::Matrix4d pose;
  pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  std::shared_ptr<base::SensorInfo> lidar_sensor_info(new base::SensorInfo);
  lidar_sensor_info->type = base::SensorType::VELODYNE_64;
  lidar_sensor_info->name = "VELODYNE_64";
  SensorPtr lidar_sensor(new Sensor(*lidar_sensor_info));
  std::shared_ptr<base::SensorInfo> radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  std::shared_ptr<Sensor> radar_sensor(new Sensor(*radar_sensor_info));
  std::shared_ptr<base::SensorInfo> camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";
  std::shared_ptr<Sensor> camera_sensor(new Sensor(*camera_sensor_info));
  // lidar track
  base::ObjectPtr base_track_lidar(new base::Object);
  base_track_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_track_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_track_lidar->direction = Eigen::Vector3f(0.707, 0.707, 0.0);
  base_track_lidar->velocity = Eigen::Vector3f(3.0, 2.0, 1.0);
  std::vector<base::ObjectPtr> lidar_track_objects;
  lidar_track_objects.push_back(base_track_lidar);
  base::FramePtr lidar_track_frame(new base::Frame);
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
  base_object_lidar->center = Eigen::Vector3d(10.5, 0, 0);
  base_object_lidar->anchor_point = Eigen::Vector3d(10.5, 0, 0);
  // base_object_lidar->theta = 0.0;
  base_object_lidar->direction = Eigen::Vector3f(1, 0, 0);
  base_object_lidar->velocity = Eigen::Vector3f(3.0, 2.0, 1.0);
  std::vector<base::ObjectPtr> lidar_objects;
  lidar_objects.push_back(base_object_lidar);
  base::FramePtr lidar_frame(new base::Frame);
  lidar_frame->sensor_info = *lidar_sensor_info;
  lidar_frame->timestamp = 151192277.124567989;
  lidar_frame->sensor2world_pose = pose;
  lidar_frame->objects = lidar_objects;
  SensorFramePtr lidar_sensor_frame_2(new SensorFrame);
  lidar_sensor_frame_2->Initialize(lidar_frame, lidar_sensor);
  SensorObjectConstPtr lidar_measurement(
      new const SensorObject(base_object_lidar, lidar_sensor_frame_2));
  // radar measurement
  base::ObjectPtr base_object_radar(new base::Object);
  base_object_radar->center = Eigen::Vector3d(10.5, 0, 0);
  base_object_radar->anchor_point = Eigen::Vector3d(10.5, 0, 0);
  // base_object_radar->theta = SHAPE_FUSION_PI / 4;
  base_object_radar->direction = Eigen::Vector3f(0.707, 0.707, 0);
  base_object_radar->velocity = Eigen::Vector3f(2.0, 1.0, 3.0);
  std::vector<base::ObjectPtr> radar_objects;
  radar_objects.push_back(base_object_radar);
  base::FramePtr radar_frame(new base::Frame);
  radar_frame->sensor_info = *radar_sensor_info;
  radar_frame->timestamp = 151192277.124567989;
  radar_frame->sensor2world_pose = pose;
  radar_frame->objects = radar_objects;
  SensorFramePtr radar_sensor_frame(new SensorFrame);
  radar_sensor_frame->Initialize(radar_frame, radar_sensor);
  SensorObjectConstPtr radar_measurement(
      new const SensorObject(base_object_radar, radar_sensor_frame));
  // radar measurement 2
  base::ObjectPtr base_object_radar_2(new base::Object);
  base_object_radar_2->center = Eigen::Vector3d(10.5, 0, 0);
  base_object_radar_2->anchor_point = Eigen::Vector3d(10.5, 0, 0);
  // base_object_radar_2->theta = SHAPE_FUSION_PI / 4;
  base_object_radar_2->direction = Eigen::Vector3f(0.707, 0.707, 0);
  base_object_radar_2->velocity = Eigen::Vector3f(3.3, 2.2, 1.1);
  std::vector<base::ObjectPtr> radar_objects_2;
  radar_objects_2.push_back(base_object_radar_2);
  base::FramePtr radar_frame_2(new base::Frame);
  radar_frame_2->sensor_info = *radar_sensor_info;
  radar_frame_2->timestamp = 151192277.124567989;
  radar_frame_2->sensor2world_pose = pose;
  radar_frame_2->objects = radar_objects_2;
  SensorFramePtr radar_sensor_frame_2(new SensorFrame);
  radar_sensor_frame_2->Initialize(radar_frame_2, radar_sensor);
  SensorObjectConstPtr radar_measurement_2(
      new const SensorObject(base_object_radar_2, radar_sensor_frame_2));
  // camera measurment
  base::ObjectPtr base_object_camera(new base::Object);
  base_object_camera->center = Eigen::Vector3d(10.5, 0, 0);
  base_object_camera->anchor_point = Eigen::Vector3d(10.5, 0, 0);
  base_object_camera->theta = SHAPE_FUSION_PI / 4;
  base_object_camera->direction = Eigen::Vector3f(0.707, 0.707, 0);
  base_object_camera->velocity = Eigen::Vector3f(3.5, 2.5, 1.5);
  std::vector<base::ObjectPtr> camera_objects;
  camera_objects.push_back(base_object_camera);
  base::FramePtr camera_frame(new base::Frame);
  camera_frame->sensor_info = *camera_sensor_info;
  camera_frame->timestamp = 151192277.124567989;
  camera_frame->sensor2world_pose = pose;
  camera_frame->objects = camera_objects;
  SensorFramePtr camera_sensor_frame(new SensorFrame);
  camera_sensor_frame->Initialize(camera_frame, camera_sensor);
  SensorObjectConstPtr camera_measurement(
      new const SensorObject(base_object_camera, camera_sensor_frame));
  // motion fusion
  KalmanMotionFusion kal(lidar_track);
  bool state = kal.Init();
  EXPECT_TRUE(state);
  EXPECT_FLOAT_EQ(kal.fused_anchor_point_(0), 10.0);
  EXPECT_FLOAT_EQ(kal.fused_anchor_point_(1), 0.0);
  EXPECT_FLOAT_EQ(kal.fused_anchor_point_(2), 0.0);
  EXPECT_FLOAT_EQ(kal.fused_velocity_(0), 3.0);
  EXPECT_FLOAT_EQ(kal.fused_velocity_(1), 2.0);
  EXPECT_FLOAT_EQ(kal.fused_velocity_(2), 1.0);

  Eigen::MatrixXd r_mat;
  r_mat.setIdentity(6, 6);
  kal.RewardRMatrix(base::SensorType::VELODYNE_64, true, &r_mat);
  EXPECT_FLOAT_EQ(r_mat(0, 0), 0.01);
  r_mat.setIdentity(6, 6);
  kal.RewardRMatrix(base::SensorType::VELODYNE_64, false, &r_mat);
  EXPECT_FLOAT_EQ(r_mat(2, 2), 1000.0);

  r_mat.setIdentity(6, 6);
  kal.RewardRMatrix(base::SensorType::SHORT_RANGE_RADAR, false, &r_mat);
  EXPECT_FLOAT_EQ(r_mat(5, 5), 0.5);
  kal.UpdateWithoutMeasurement("sensor_id", 151192277.124567989,
                               151192277.124567989);
  KalmanMotionFusion kal3(lidar_track);
  kal3.Init();
  kal3.UpdateWithMeasurement(lidar_measurement, 151192277.124567989);
  EXPECT_FLOAT_EQ(kal3.fused_anchor_point_(0), 10.5);
  EXPECT_FLOAT_EQ(kal3.fused_anchor_point_(1), 0.0);
  EXPECT_FLOAT_EQ(kal3.fused_anchor_point_(2), 0.0);
  EXPECT_FLOAT_EQ(kal3.fused_velocity_(0), 3.0);
  EXPECT_FLOAT_EQ(kal3.fused_velocity_(1), 2.0);
  EXPECT_FLOAT_EQ(kal3.fused_velocity_(2), 1.0);
  kal3.UpdateWithMeasurement(radar_measurement, 151192277.124567989);
  KalmanMotionFusion kal2(lidar_track);
  kal2.Init();
  kal2.UpdateWithMeasurement(camera_measurement, 151192277.124567989);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(0), 10.5);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(1), 0.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(2), 0.0);
  EXPECT_FLOAT_EQ(3.0002499, kal2.fused_velocity_(0));
  EXPECT_FLOAT_EQ(2.0002499, kal2.fused_velocity_(1));
  EXPECT_FLOAT_EQ(1.5, kal2.fused_velocity_(2));
  kal2.UpdateWithMeasurement(radar_measurement_2, 151192277.124567989);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(0), 10.5);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(1), 0.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(2), 0.0);
  EXPECT_FLOAT_EQ(3.0004749, kal2.fused_velocity_(0));
  EXPECT_FLOAT_EQ(2.0005534, kal2.fused_velocity_(1));
  EXPECT_FLOAT_EQ(1.1, kal2.fused_velocity_(2));
}
*/

/* TODO(all): Initialize() not compiling. to be fixed
TEST(MotionFusionTest, radar_test) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "fusion/kalman_motion_fusion";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/kalman_motion_fusion/params";
  Eigen::Matrix4d pose;
  pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  std::shared_ptr<base::SensorInfo> lidar_sensor_info(new base::SensorInfo);
  lidar_sensor_info->type = base::SensorType::VELODYNE_64;
  lidar_sensor_info->name = "VELODYNE_64";
  SensorPtr lidar_sensor(new Sensor(*lidar_sensor_info));
  std::shared_ptr<base::SensorInfo> radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  SensorPtr radar_sensor(new Sensor(*radar_sensor_info));
  std::shared_ptr<base::SensorInfo> camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";
  SensorPtr camera_sensor(new Sensor(*camera_sensor_info));
  std::shared_ptr<base::SensorInfo> unknown_sensor_info(new base::SensorInfo);
  unknown_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  unknown_sensor_info->name = "UNKNOWN_SENSOR_TYPE";
  SensorPtr unknown_sensor(new Sensor(*unknown_sensor_info));
  // radar track
  base::ObjectPtr base_track_radar(new base::Object);
  base_track_radar->center = Eigen::Vector3d(10, 0, 0);
  base_track_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_track_radar->theta = 0.0;
  base_track_radar->direction = Eigen::Vector3f(1.0, 0.0, 0.0);
  base_track_radar->velocity = Eigen::Vector3f(2.2, 3.2, 1.2);
  std::vector<base::ObjectPtr> radar_track_objects;
  radar_track_objects.push_back(base_track_radar);
  base::FramePtr radar_track_frame(new base::Frame);
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
  base_object_lidar->velocity = Eigen::Vector3f(2.0, 1.0, 3.0);
  base_object_lidar->direction = Eigen::Vector3f(1, 0, 0);
  std::vector<base::ObjectPtr> lidar_objects;
  lidar_objects.push_back(base_object_lidar);
  base::FramePtr lidar_frame(new base::Frame);
  lidar_frame->sensor_info = *lidar_sensor_info;
  lidar_frame->timestamp = 151192277.124567989;
  lidar_frame->sensor2world_pose = pose;
  lidar_frame->objects = lidar_objects;
  SensorFramePtr lidar_sensor_frame_2(new SensorFrame);
  lidar_sensor_frame_2->Initialize(lidar_frame, lidar_sensor);
  SensorObjectConstPtr lidar_measurement(
      new const SensorObject(base_object_lidar, lidar_sensor_frame_2));
  // radar measurement
  base::ObjectPtr base_object_radar(new base::Object);
  base_object_radar->center = Eigen::Vector3d(10, 0, 0);
  base_object_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_radar->theta = SHAPE_FUSION_PI / 4;
  base_object_radar->direction = Eigen::Vector3f(0.707, 0.707, 0);
  base_object_radar->velocity = Eigen::Vector3f(2.0, 1.0, 3.0);
  std::vector<base::ObjectPtr> radar_objects;
  radar_objects.push_back(base_object_radar);
  base::FramePtr radar_frame(new base::Frame);
  radar_frame->sensor_info = *radar_sensor_info;
  radar_frame->timestamp = 151192277.124567989;
  radar_frame->sensor2world_pose = pose;
  radar_frame->objects = radar_objects;
  SensorFramePtr radar_sensor_frame2(new SensorFrame);
  radar_sensor_frame->Initialize(radar_frame, radar_sensor);
  SensorObjectConstPtr radar_measurement(
      new const SensorObject(base_object_radar, radar_sensor_frame2));
  // camera measurment
  base::ObjectPtr base_object_camera(new base::Object);
  base_object_camera->center = Eigen::Vector3d(10, 0, 0);
  base_object_camera->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_camera->theta = SHAPE_FUSION_PI / 4;
  base_object_camera->direction = Eigen::Vector3f(0.707, 0.707, 0);
  base_object_camera->velocity = Eigen::Vector3f(2.5, 3.5, 1.5);
  std::vector<base::ObjectPtr> camera_objects;
  camera_objects.push_back(base_object_camera);
  base::FramePtr camera_frame(new base::Frame);
  camera_frame->sensor_info = *camera_sensor_info;
  camera_frame->timestamp = 151192277.124567989;
  camera_frame->sensor2world_pose = pose;
  camera_frame->objects = camera_objects;
  SensorFramePtr camera_sensor_frame(new SensorFrame);
  camera_sensor_frame->Initialize(camera_frame, camera_sensor);
  SensorObjectConstPtr camera_measurement(
      new const SensorObject(base_object_camera, camera_sensor_frame));
  // motion fusion
  TrackPtr tpr = nullptr;
  KalmanMotionFusion kal0(tpr);
  EXPECT_FALSE(kal0.Init());
  KalmanMotionFusion kal(radar_track);
  bool state = kal.Init();
  EXPECT_TRUE(state);
  EXPECT_TRUE(kal.filter_init_);
  kal.UpdateWithMeasurement(camera_measurement, 151192277.124567989);

  KalmanMotionFusion kal2(radar_track);
  kal2.Init();
  for (size_t i = 0; i < 3; i++) {
    kal2.UpdateSensorHistory(base::SensorType::VELODYNE_64,
                             Eigen::Vector3d(3.0, 2.0, 1.0),
                             151192277.124567989);
    kal2.UpdateSensorHistory(base::SensorType::LONG_RANGE_RADAR,
                             Eigen::Vector3d(3.0, 2.0, 1.0),
                             151192277.124567989);
    kal2.UpdateSensorHistory(base::SensorType::MONOCULAR_CAMERA,
                             Eigen::Vector3d(3.0, 2.0, 1.0),
                             151192277.124567989);
  }
  kal2.UpdateWithMeasurement(lidar_measurement, 151192277.124567989);
  Eigen::Vector3d velocity_temp;

  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(0), 10.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(1), 0.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(2), 0.0);
  EXPECT_FLOAT_EQ(3.1803923, kal2.fused_velocity_(0));
  EXPECT_FLOAT_EQ(1.6313726, kal2.fused_velocity_(1));
  EXPECT_FLOAT_EQ(3.0, kal2.fused_velocity_(2));
  velocity_temp << 0, 0, 0;
  kal2.ComputeAccelerationMeasurement(base::SensorType::MONOCULAR_CAMERA,
                                      velocity_temp, 151192277.124567989);
  kal2.UpdateWithMeasurement(camera_measurement, 151192277.124567989);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(0), 10.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(1), 0.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(2), 0.0);
  EXPECT_FLOAT_EQ(3.1800454, kal2.fused_velocity_(0));
  EXPECT_FLOAT_EQ(1.6323247, kal2.fused_velocity_(1));
  EXPECT_FLOAT_EQ(1.5, kal2.fused_velocity_(2));
  kal2.UpdateWithMeasurement(radar_measurement, 151192277.224567989);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(0), 10.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(1), 0.0);
  EXPECT_FLOAT_EQ(kal2.fused_anchor_point_(2), 0.0);
  EXPECT_FLOAT_EQ(2, kal2.fused_velocity_(0));
  EXPECT_FLOAT_EQ(1, kal2.fused_velocity_(1));
  EXPECT_FLOAT_EQ(3, kal2.fused_velocity_(2));
}
*/

/* TODO(all): Initialize() not compiling. to be fixed
TEST(MotionFusionTest, get_history_test) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "fusion/kalman_motion_fusion";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/kalman_motion_fusion/params";
  Eigen::Matrix4d pose;
  pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  std::shared_ptr<base::SensorInfo> lidar_sensor_info(new base::SensorInfo);
  lidar_sensor_info->type = base::SensorType::VELODYNE_64;
  lidar_sensor_info->name = "VELODYNE_64";
  SensorPtr lidar_sensor(new Sensor(*lidar_sensor_info));
  // lidar track
  base::ObjectPtr base_track_lidar(new base::Object);
  std::vector<base::ObjectPtr> lidar_track_objects;
  lidar_track_objects.push_back(base_track_lidar);
  base::FramePtr lidar_track_frame(new base::Frame);
  lidar_track_frame->sensor_info = *lidar_sensor_info;
  lidar_track_frame->objects = lidar_track_objects;
  SensorFramePtr lidar_sensor_frame(new SensorFrame);
  lidar_sensor_frame->Initialize(lidar_track_frame, lidar_sensor);
  SensorObjectPtr lidar_obj(
      new SensorObject(base_track_lidar, lidar_sensor_frame));
  TrackPtr lidar_track(new Track());
  lidar_track->Initialize(lidar_obj, false);
  KalmanMotionFusion kal(lidar_track);
  bool state = kal.Init();
  for (size_t i = 0; i < 3; i++) {
    kal.UpdateSensorHistory(base::SensorType::VELODYNE_64,
                            Eigen::Vector3d(3.0, 2.0, 1.0),
                            151192277.124567989);
    kal.UpdateSensorHistory(base::SensorType::LONG_RANGE_RADAR,
                            Eigen::Vector3d(3.0, 2.0, 1.0),
                            151192277.124567989);
    kal.UpdateSensorHistory(base::SensorType::MONOCULAR_CAMERA,
                            Eigen::Vector3d(3.0, 2.0, 1.0),
                            151192277.124567989);
  }
  int x = kal.GetSensorHistoryIndex(base::SensorType::VELODYNE_64, 2);
  int y = kal.GetSensorHistoryIndex(base::SensorType::VELODYNE_64, 4);
  EXPECT_EQ(3, x);
  EXPECT_EQ(-1, y);
}
*/
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
