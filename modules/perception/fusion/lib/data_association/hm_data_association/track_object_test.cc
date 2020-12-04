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

#include "modules/perception/base/frame.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/fusion/base/sensor_object.h"
#include "modules/perception/fusion/base/track.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/hm_tracks_objects_match.h"

namespace apollo {
namespace perception {
namespace fusion {
/*
TODO(all): not compiling. to be fixed

TEST(MatcherTest, test_generate_unassign) {
  HMTrackersObjectsAssociation matcher;
  std::vector<TrackMeasurmentPair> assignments;
  assignments.push_back(std::make_pair(1, 5));
  assignments.push_back(std::make_pair(2, 6));
  assignments.push_back(std::make_pair(3, 7));
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  matcher.GenerateUnassignedData(5, 8, assignments, &unassigned_tracks,
                                 &unassigned_objects);
  EXPECT_EQ(unassigned_tracks.size(), 2);
  EXPECT_EQ(unassigned_objects.size(), 5);
}

TEST(MatcherTest, test_all) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/"
      "fusion/hm_data_association";
  FLAGS_obs_sensor_meta_path = "./data/sensor_meta.pt";
  FLAGS_obs_sensor_intrinsic_path = "/apollo/modules/perception/testdata/"
      "fusion/hm_data_association/params";
  Eigen::Matrix4d pose;
  pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  HMTrackersObjectsAssociation::s_match_distance_thresh_ = 4.0;
  HMTrackersObjectsAssociation::s_association_center_dist_threshold_ = 10.0;
  // lidar_sensor_info
  base::SensorInfoPtr lidar_sensor_info(new base::SensorInfo);
  lidar_sensor_info->type = base::SensorType::VELODYNE_64;
  lidar_sensor_info->name = "VELODYNE_64";
  SensorPtr lidar_sensor(new Sensor(*lidar_sensor_info));
  // radar_sensor_info
  base::SensorInfoPtr radar_sensor_info(new base::SensorInfo);
  radar_sensor_info->type = base::SensorType::LONG_RANGE_RADAR;
  radar_sensor_info->name = "LONG_RANGE_RADAR";
  SensorPtr radar_sensor(new Sensor(*radar_sensor_info));
  // camera_sensor_info
  base::SensorInfoPtr camera_sensor_info(new base::SensorInfo);
  camera_sensor_info->type = base::SensorType::MONOCULAR_CAMERA;
  camera_sensor_info->name = "MONOCULAR_CAMERA";
  SensorPtr camera_sensor(new Sensor(*camera_sensor_info));
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
  std::vector<base::ObjectPtr> lidar_track_objects;
  lidar_track_objects.push_back(base_track_lidar);
  base::FramePtr lidar_track_frame(new base::Frame);
  lidar_track_frame->sensor_info = *lidar_sensor_info;
  lidar_track_frame->timestamp = 151192277.124567989;
  lidar_track_frame->sensor2world_pose = pose;
  lidar_track_frame->objects = lidar_track_objects;
  SensorFramePtr lidar_sensor_frame(new SensorFrame);
  lidar_sensor_frame->Initialize(lidar_track_frame, lidar_sensor);
  // vector<Trackptr>
  std::vector<TrackPtr> fusion_tracks;
  // track1
  SensorObjectPtr lidar_obj(
      new SensorObject(base_track_lidar, lidar_sensor_frame));
  TrackPtr lidar_track1(new Track());
  lidar_track1->Initialize(lidar_obj, false);
  // track2
  base::ObjectPtr base_track_lidar2(new base::Object);
  base_track_lidar2->center = Eigen::Vector3d(40, 0, 0);
  base_track_lidar2->anchor_point = Eigen::Vector3d(40, 0, 0);
  base_track_lidar2->track_id = 1;
  SensorObjectPtr lidar_obj2(
      new SensorObject(base_track_lidar2, lidar_sensor_frame));
  TrackPtr lidar_track2(new Track());
  lidar_track2->Initialize(lidar_obj2, false);
  // track3
  base::ObjectPtr base_track_lidar3(new base::Object);
  base_track_lidar3->center = Eigen::Vector3d(30, 0, 0);
  base_track_lidar3->anchor_point = Eigen::Vector3d(30, 0, 0);
  base_track_lidar3->track_id = 2;
  SensorObjectPtr lidar_obj3(
      new SensorObject(base_track_lidar3, lidar_sensor_frame));
  TrackPtr lidar_track3(new Track());
  lidar_track3->Initialize(lidar_obj3, false);
  // add track to vector
  fusion_tracks.push_back(lidar_track1);
  fusion_tracks.push_back(lidar_track2);
  fusion_tracks.push_back(lidar_track3);
  // vector<SensorObjectPtr>
  std::vector<SensorObjectPtr> sensor_objects;
  base::ObjectPtr base_object_lidar(new base::Object);
  base_object_lidar->center = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->anchor_point = Eigen::Vector3d(10, 0, 0);
  base_object_lidar->track_id = 0;
  base_object_lidar->polygon.resize(3);
  for (size_t i = 0; i < 3; i++) {
    base_object_lidar->polygon[i].x = 10;
    base_object_lidar->polygon[i].y = 0;
    base_object_lidar->polygon[i].z = 0;
  }
  // std::vector<base::ObjectPtr> lidar_objects;
  // lidar_objects.push_back(base_object_lidar);
  base::FramePtr lidar_frame(new base::Frame);
  lidar_frame->sensor_info = *lidar_sensor_info;
  lidar_frame->timestamp = 151192277.124567989;
  lidar_frame->sensor2world_pose = pose;
  lidar_frame->objects.push_back(base_object_lidar);
  SensorFramePtr lidar_sensor_frame_2(new SensorFrame);
  lidar_sensor_frame_2->Initialize(lidar_frame, lidar_sensor);
  SensorObjectPtr lidar_measurement(
      new SensorObject(base_object_lidar, lidar_sensor_frame_2));
  base::ObjectPtr base_object_lidar2(new base::Object);
  base_object_lidar2->center = Eigen::Vector3d(20, 0, 0);
  base_object_lidar2->anchor_point = Eigen::Vector3d(20, 0, 0);
  base_object_lidar2->track_id = 4;
  lidar_frame->objects.push_back(base_object_lidar2);
  lidar_sensor_frame_2->Initialize(lidar_frame, lidar_sensor);
  SensorObjectPtr lidar_measurement2(
      new SensorObject(base_object_lidar2, lidar_sensor_frame_2));
  // add sensor_obj
  sensor_objects.push_back(lidar_measurement);
  sensor_objects.push_back(lidar_measurement2);
  // compute associtation mat
  Eigen::Vector3d ref_point = Eigen::Vector3d::Zero();
  std::vector<size_t> unassigned_tracks = {0, 1, 2};
  std::vector<size_t> unassigned_measurements = {0, 1};
  std::vector<std::vector<double>> association_mat;
  HMTrackersObjectsAssociation matcher;
  matcher.ComputeAssociationDistanceMat(
      fusion_tracks, sensor_objects, ref_point, unassigned_tracks,
      unassigned_measurements, &association_mat);
  EXPECT_FLOAT_EQ(association_mat[0][0], 0.0);
  EXPECT_FLOAT_EQ(association_mat[0][1], 4.0);
  EXPECT_FLOAT_EQ(association_mat[1][0], 4.0);
  EXPECT_FLOAT_EQ(association_mat[1][1], 4.0);
  EXPECT_FLOAT_EQ(association_mat[2][0], 4.0);
  EXPECT_FLOAT_EQ(association_mat[2][1], 4.0);
  unassigned_tracks.clear();
  unassigned_measurements.clear();
  std::vector<TrackMeasurmentPair> assignments;
  //test id assign with do_nothing equals to true
  matcher.IdAssign(fusion_tracks, sensor_objects, &assignments,
                   &unassigned_tracks, &unassigned_measurements, true);
  EXPECT_EQ(0, assignments.size());
  EXPECT_EQ(3, unassigned_tracks.size());
  EXPECT_EQ(2, unassigned_measurements.size());
  unassigned_tracks.clear();
  unassigned_measurements.clear();
  //test ordinary id assign
  matcher.IdAssign(fusion_tracks, sensor_objects, &assignments,
                   &unassigned_tracks, &unassigned_measurements);
  EXPECT_EQ(1, assignments.size());
  EXPECT_EQ(2, unassigned_tracks.size());
  EXPECT_EQ(1, unassigned_measurements.size());
  std::vector<TrackMeasurmentPair> post_assignments;
  matcher.PostIdAssign(fusion_tracks, sensor_objects, unassigned_tracks,
                       unassigned_measurements, &post_assignments);
  EXPECT_EQ(post_assignments.size(), 0);
  AssociationResult association_result;
  assignments.clear();
  unassigned_tracks.clear();
  unassigned_measurements.clear();
  assignments.push_back(std::make_pair(0, 0));
  unassigned_tracks.push_back(1);
  unassigned_tracks.push_back(2);
  unassigned_measurements.push_back(1);
  std::vector<int> track_ind_g2l = {0, 0, 1};
  std::vector<int> measurement_ind_g2l = {0, 0};
  std::vector<size_t> measurement_ind_l2g = {1};
  association_mat.clear();
  association_mat.push_back(std::vector<double>(4.0));
  association_mat.push_back(std::vector<double>(4.0));
  // add camera sensor_object
  base::ObjectPtr base_object_camera(new base::Object);
  std::vector<base::ObjectPtr> camera_objects;
  camera_objects.push_back(base_object_camera);
  base::FramePtr camera_frame(new base::Frame);
  camera_frame->sensor_info = *camera_sensor_info;
  camera_frame->timestamp = 151192277.124567989;
  camera_frame->sensor2world_pose = pose;
  camera_frame->objects = camera_objects;
  SensorFramePtr camera_sensor_frame(new SensorFrame);
  camera_sensor_frame->Initialize(camera_frame, camera_sensor);
  SensorObjectPtr camera_measurement(
      new SensorObject(base_object_camera, camera_sensor_frame));
  sensor_objects.push_back(camera_measurement);
  // add radar track
  base::ObjectPtr base_track_radar(new base::Object);
  base_track_radar->center = Eigen::Vector3d(10, 0, 0);
  base_track_radar->anchor_point = Eigen::Vector3d(10, 0, 0);
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
  fusion_tracks.push_back(radar_track);
  AssociationResult association_result2;
  assignments.clear();
  unassigned_tracks.clear();
  unassigned_measurements.clear();
  assignments.push_back(std::make_pair(0, 0));
  unassigned_tracks.push_back(1);
  unassigned_tracks.push_back(2);
  unassigned_tracks.push_back(3);
  unassigned_measurements.push_back(1);
  unassigned_measurements.push_back(2);
  track_ind_g2l.clear();
  track_ind_g2l = {0, 1, 2, 3};
  measurement_ind_g2l.clear();
  measurement_ind_g2l = {0, 1, 2};
  measurement_ind_l2g.clear();
  measurement_ind_l2g = {0, 1, 2};
  association_mat.clear();
  association_mat =
      std::vector<std::vector<double>>(4, std::vector<double>(3, 4.0));
  association_mat[1][2] = 3.0;
  association_mat[3][2] = 3.0;
  association_result2.assignments = assignments;
  association_result2.unassigned_tracks = unassigned_tracks;
  association_result2.track2measurements_dist.assign(fusion_tracks.size(), 0);
  association_result2.measurement2track_dist.assign(sensor_objects.size(), 0);
  association_result2.unassigned_measurements = unassigned_measurements;
  matcher.ComputeDistance(fusion_tracks, sensor_objects, unassigned_tracks,
                          track_ind_g2l, measurement_ind_g2l,
                          measurement_ind_l2g, association_mat,
                          &association_result2);
  EXPECT_FLOAT_EQ(association_result2.track2measurements_dist[0], 4.0);
  EXPECT_FLOAT_EQ(association_result2.track2measurements_dist[1], 0.0);
  EXPECT_FLOAT_EQ(association_result2.track2measurements_dist[2], 4.0);
  EXPECT_FLOAT_EQ(association_result2.track2measurements_dist[3], 0.0);
  EXPECT_FLOAT_EQ(association_result2.measurement2track_dist[0], 4.0);
  EXPECT_FLOAT_EQ(association_result2.measurement2track_dist[1], 4.0);
  EXPECT_FLOAT_EQ(association_result2.measurement2track_dist[2], 3.0);
  matcher.PostIdAssign(fusion_tracks, sensor_objects, unassigned_tracks,
                       unassigned_measurements, &post_assignments);
  EXPECT_EQ(post_assignments.size(), 0);
  ScenePtr scene(new Scene());
  for (auto track : fusion_tracks) {
    scene->AddForegroundTrack(track);
  }
  HMTrackersObjectsAssociation matcher2;
  AssociationOptions options;
  AssociationResult association_result3;
  matcher2.Associate(options, lidar_sensor_frame_2, scene,
                     &association_result3);
  EXPECT_EQ(association_result3.assignments.size(), 2);
  EXPECT_EQ(association_result3.unassigned_tracks.size(), 3);
  EXPECT_EQ(association_result3.unassigned_measurements.size(), 1);
}

TEST(MatcherTest, test_minimize) {
  HMTrackersObjectsAssociation matcher;
  std::vector<std::vector<double>> association_mat =
      std::vector<std::vector<double>>(4, std::vector<double>(3, 4.0));
  association_mat[0][0] = 0.0;
  std::vector<size_t> track_ind_l2g = {0, 1, 2, 3};
  std::vector<size_t> measurement_ind_l2g = {0, 1, 2};
  std::vector<TrackMeasurmentPair> assignments;
  std::vector<size_t> unassigned_tracks = {0, 1, 2, 3};
  std::vector<size_t> unassigned_measurements = {0, 1, 2};
  matcher.MinimizeAssignment(association_mat, track_ind_l2g,
                             measurement_ind_l2g, &assignments,
                             &unassigned_tracks, &unassigned_measurements);
  EXPECT_EQ(assignments.size(), 1);
}
*/
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
