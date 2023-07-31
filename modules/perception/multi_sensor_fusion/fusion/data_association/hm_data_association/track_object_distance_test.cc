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

#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/track_object_distance.h"

#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/multi_sensor_fusion/base/sensor.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_frame.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_object.h"
#include "modules/perception/multi_sensor_fusion/base/track.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/probabilities.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/projection_cache.h"
#include "modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association/track_object_similarity.h"

namespace apollo {
namespace perception {
namespace fusion {
/*
TODO(all): not compiling. to be fixed

class TrackObjectDistanceTest : public testing::Test {
 protected:
  void SetUp() {
    lib::FLAGS_work_root = "./fusion_test_data/hm_data_association";
    common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
    common::FLAGS_obs_sensor_intrinsic_path =
"./fusion_test_data/hm_data_association/params";
  }
};

TEST_F(TrackObjectDistanceTest, test_is_track_id_consistent) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr object1_ptr(new base::Object());
  object1_ptr->track_id = 0;
  base::ObjectConstPtr object1_const_ptr(object1_ptr);
  SensorFramePtr frame_ptr(new SensorFrame());
  SensorObjectConstPtr object1_const_sensor_ptr(
      new SensorObject(object1_const_ptr, frame_ptr));
  base::ObjectPtr object2_ptr(new base::Object());
  object2_ptr->track_id = 1;
  base::ObjectConstPtr object2_const_ptr(object2_ptr);
  SensorObjectConstPtr object2_const_sensor_ptr(
      new SensorObject(object2_const_ptr, frame_ptr));
  SensorObjectConstPtr object3_const_sensor_ptr = nullptr;
  SensorObjectConstPtr object4_const_sensor_ptr = nullptr;
  EXPECT_FALSE(track_object_distance.IsTrackIdConsistent(
      object1_const_sensor_ptr, object2_const_sensor_ptr));
  EXPECT_FALSE(track_object_distance.IsTrackIdConsistent(
      object1_const_sensor_ptr, object3_const_sensor_ptr));
  EXPECT_FALSE(track_object_distance.IsTrackIdConsistent(
      object3_const_sensor_ptr, object4_const_sensor_ptr));
}

TEST_F(TrackObjectDistanceTest, test_comppute_polygon_center) {
  TrackObjectDistance track_object_distance;
  base::PolygonDType polygon;
  Eigen::Vector3d center;
  EXPECT_FALSE(track_object_distance.ComputePolygonCenter(polygon, &center));
  base::PointD pt;
  pt.x = 1;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  polygon.push_back(pt);
  EXPECT_TRUE(track_object_distance.ComputePolygonCenter(polygon, &center));
}

TEST_F(TrackObjectDistanceTest, test_compute_polygon_distance3d) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr fusion_object_ptr(new base::Object());
  base::ObjectConstPtr fusion_object_const_ptr(fusion_object_ptr);
  SensorFramePtr fusion_frame_ptr(new SensorFrame());
  SensorObjectConstPtr fusion_const_sensor_ptr(
      new SensorObject(fusion_object_const_ptr, fusion_frame_ptr));

  base::ObjectPtr sensor_object_ptr(new base::Object());
  base::ObjectConstPtr sensor_object_const_ptr(sensor_object_ptr);
  SensorFramePtr sensor_frame_ptr(new SensorFrame());
  SensorObjectPtr sensor_sensor_object_ptr(
      new SensorObject(sensor_object_const_ptr, sensor_frame_ptr));

  Eigen::Vector3d ref_pos(0, 0, 0);
  int range = 1;
  EXPECT_EQ(track_object_distance.ComputePolygonDistance3d(
      fusion_const_sensor_ptr, sensor_sensor_object_ptr, ref_pos,
      range), std::numeric_limits<float>::max());
  base::PointD pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  fusion_object_ptr->polygon.push_back(pt);
  EXPECT_EQ(track_object_distance.ComputePolygonDistance3d(
      fusion_const_sensor_ptr, sensor_sensor_object_ptr, ref_pos,
      range), std::numeric_limits<float>::max());
  sensor_object_ptr->polygon.push_back(pt);
  EXPECT_EQ(track_object_distance.ComputePolygonDistance3d(
      fusion_const_sensor_ptr,
      sensor_sensor_object_ptr, ref_pos, range), 0);
}

TEST_F(TrackObjectDistanceTest, test_query_polygon_dcenter) {
  TrackObjectDistance track_object_distance;
  base::ObjectConstPtr object_const_ptr;
  Eigen::Vector3d ref_pos(0, 0, 0);
  int range = 1;
  Eigen::Vector3d polygon_ct;
  EXPECT_FALSE(track_object_distance.QueryPolygonDCenter(
      object_const_ptr, ref_pos, range, &polygon_ct));

  base::ObjectPtr object_ptr(new base::Object());
  object_const_ptr = base::ObjectConstPtr(object_ptr);
  EXPECT_FALSE(track_object_distance.QueryPolygonDCenter(
      object_const_ptr, ref_pos, range, &polygon_ct));
  base::PointD pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  object_ptr->polygon.push_back(pt);
  EXPECT_TRUE(track_object_distance.QueryPolygonDCenter(
      object_const_ptr, ref_pos, range, &polygon_ct));
}

TEST_F(TrackObjectDistanceTest, test_query_camera_model) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr sensor_object_ptr(new base::Object());
  base::ObjectConstPtr sensor_object_const_ptr(sensor_object_ptr);
  base::SensorInfo sensor_info;
  sensor_info.name = "front_camera";
  sensor_info.type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(sensor_info));
  base::FramePtr frame_ptr(new base::Frame());
  frame_ptr->sensor_info = sensor_info;
  SensorFramePtr sensor_frame_ptr(new SensorFrame());
  sensor_frame_ptr->Initialize(base::FrameConstPtr(frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(sensor_object_const_ptr, sensor_frame_ptr));
  EXPECT_NE(track_object_distance.QueryCameraModel(
      camera_sensor_object_ptr), nullptr);
}

TEST_F(TrackObjectDistanceTest, test_query_world2camera_pose) {
  TrackObjectDistance track_object_distance;
  {
    base::ObjectPtr sensor_object_ptr(new base::Object());
    base::ObjectConstPtr sensor_object_const_ptr(sensor_object_ptr);
    base::SensorInfo* sensor_info_ptr = new base::SensorInfo;
    sensor_info_ptr->name = "front_camera";
    sensor_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
    SensorPtr camera_sensor_ptr(new Sensor(*sensor_info_ptr));

    base::FramePtr frame_ptr(new base::Frame());
    frame_ptr->timestamp = 0.0;
    frame_ptr->sensor_info = *sensor_info_ptr;
    camera_sensor_ptr->AddFrame(frame_ptr);
    SensorFramePtr sensor_frame_ptr(new SensorFrame());
    sensor_frame_ptr->Initialize(base::FrameConstPtr(frame_ptr),
        camera_sensor_ptr);
    SensorObjectPtr camera_sensor_object_ptr(
        new SensorObject(sensor_object_const_ptr, sensor_frame_ptr));
    SensorDataManager* sensor_data_manager_ = SensorDataManager::Instance();
    sensor_data_manager_->AddSensorMeasurements(frame_ptr);
    Eigen::Matrix4d pose;

    EXPECT_TRUE(track_object_distance.QueryWorld2CameraPose(
        camera_sensor_object_ptr, &pose));

    delete sensor_info_ptr;
    sensor_info_ptr = nullptr;
  }

  {
    base::ObjectPtr sensor_object_ptr_2(new base::Object());
    base::ObjectConstPtr sensor_object_const_ptr_2(sensor_object_ptr_2);

    base::SensorInfo* sensor_info_ptr_2 = new base::SensorInfo;
    sensor_info_ptr_2->name = "front_camera1";
    sensor_info_ptr_2->type = base::SensorType::MONOCULAR_CAMERA;
    SensorPtr camera_sensor_ptr_2(new Sensor(*sensor_info_ptr_2));
    base::FramePtr frame_ptr_2(new base::Frame());
    frame_ptr_2->timestamp = 0.0;
    frame_ptr_2->sensor_info = *sensor_info_ptr_2;
    camera_sensor_ptr_2->AddFrame(frame_ptr_2);
    SensorFramePtr sensor_frame_ptr_2(new SensorFrame());

    sensor_frame_ptr_2->Initialize(base::FrameConstPtr(frame_ptr_2),
        camera_sensor_ptr_2);
    SensorObjectPtr camera_sensor_object_ptr_2(
        new SensorObject(sensor_object_const_ptr_2, sensor_frame_ptr_2));
    SensorDataManager* sensor_data_manager_ = SensorDataManager::Instance();
    Eigen::Matrix4d pose;
    EXPECT_FALSE(track_object_distance.QueryWorld2CameraPose(
       camera_sensor_object_ptr_2, &pose));

    delete sensor_info_ptr_2;
    sensor_info_ptr_2 = nullptr;
  }
}

TEST_F(TrackObjectDistanceTest, test_query_world2camera_pose_2) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr sensor_object_ptr(new base::Object());
  base::ObjectConstPtr sensor_object_const_ptr(sensor_object_ptr);
  base::SensorInfo* sensor_info_ptr = new base::SensorInfo;
  sensor_info_ptr->name = "front_camera";
  sensor_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*sensor_info_ptr));

  base::FramePtr frame_ptr(new base::Frame());
  frame_ptr->timestamp = 0.0;
  frame_ptr->sensor_info = *sensor_info_ptr;
  camera_sensor_ptr->AddFrame(frame_ptr);
  SensorFramePtr sensor_frame_ptr(new SensorFrame());
  sensor_frame_ptr->Initialize(base::FrameConstPtr(frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(sensor_object_const_ptr, sensor_frame_ptr));
  SensorDataManager* sensor_data_manager_ = SensorDataManager::Instance();
  sensor_data_manager_->AddSensorMeasurements(frame_ptr);
  Eigen::Matrix4d pose;
  EXPECT_TRUE(track_object_distance.QueryWorld2CameraPose(
      camera_sensor_object_ptr, &pose));
  delete sensor_info_ptr;
  sensor_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_query_lidar2world_pose) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr sensor_object_ptr(new base::Object());
  base::ObjectConstPtr sensor_object_const_ptr(sensor_object_ptr);
  base::SensorInfo* sensor_info_ptr = new base::SensorInfo;
  sensor_info_ptr->name = "lidar";
  sensor_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr lidar_sensor_ptr(new Sensor(*sensor_info_ptr));

  base::FramePtr frame_ptr(new base::Frame());
  frame_ptr->timestamp = 0.0;
  frame_ptr->sensor_info = *sensor_info_ptr;
  lidar_sensor_ptr->AddFrame(frame_ptr);
  SensorFramePtr sensor_frame_ptr = nullptr;
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(sensor_object_const_ptr, sensor_frame_ptr));
  Eigen::Matrix4d pose;
  EXPECT_FALSE(track_object_distance.QueryLidar2WorldPose(
      lidar_sensor_object_ptr, &pose));
  sensor_frame_ptr.reset(new SensorFrame());
  sensor_frame_ptr->Initialize(base::FrameConstPtr(frame_ptr),
      lidar_sensor_ptr);
  lidar_sensor_object_ptr.reset(new SensorObject(
      sensor_object_const_ptr, sensor_frame_ptr));
  EXPECT_TRUE(track_object_distance.QueryLidar2WorldPose(
      lidar_sensor_object_ptr, &pose));
  delete sensor_info_ptr;
  sensor_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_query_projection_cache_object_0) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->id = 1;
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
    camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
    new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  base::PinholeCameraModelPtr pinhole_camera_model_ptr(
    new base::PinholeCameraModel);
  pinhole_camera_model_ptr->set_width(100);
  pinhole_camera_model_ptr->set_height(100);
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Identity();
  pinhole_camera_model_ptr->set_intrinsic_params(intrinsic_params);

  ProjectionCache& projection_cache =
  track_object_distance.projection_cache_;
  projection_cache.Reset("lidar", 0.0);
  Eigen::Vector2f test_pt(2, 3);
  projection_cache.AddPoint(test_pt);
  ProjectionCacheObject* cache_object_ptr =
      projection_cache.BuildObject("lidar",  0.0, "front_camera", 0.0, 1);
  EXPECT_TRUE(track_object_distance.QueryProjectionCacheObject(
      lidar_sensor_object_ptr, camera_sensor_object_ptr,
      pinhole_camera_model_ptr, true) != nullptr);

  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_query_projection_cache_object_1) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->id = 1;
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = -1;
  pt.intensity = 1;
  cloud.push_back(pt);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar1";
  lidar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 1.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::PinholeCameraModelPtr pinhole_camera_model_ptr(
      new base::PinholeCameraModel);
  pinhole_camera_model_ptr->set_width(100);
  pinhole_camera_model_ptr->set_height(100);
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Identity();
  pinhole_camera_model_ptr->set_intrinsic_params(intrinsic_params);

  ProjectionCache& projection_cache =
  track_object_distance.projection_cache_;
  projection_cache.Reset("lidar", 0.0);
  Eigen::Vector2f test_pt(2, 3);
  projection_cache.AddPoint(test_pt);
  ProjectionCacheObject* cache_object_ptr =
      projection_cache.BuildObject("lidar", 0.0, "front_camera", 0.0, 1);
  EXPECT_FALSE(track_object_distance.QueryProjectionCacheObject(
      lidar_sensor_object_ptr, camera_sensor_object_ptr,
      pinhole_camera_model_ptr, true) != nullptr);

  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_query_projection_cache_object_2) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->id = 1;
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = -1;
  pt.intensity = 1;
  cloud.push_back(pt);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar1";
  lidar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::PinholeCameraModelPtr pinhole_camera_model_ptr(
      new base::PinholeCameraModel);
  pinhole_camera_model_ptr->set_width(100);
  pinhole_camera_model_ptr->set_height(100);
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Identity();
  pinhole_camera_model_ptr->set_intrinsic_params(intrinsic_params);
  ProjectionCacheObject* cache_object_ptr = new ProjectionCacheObject;
  cache_object_ptr->SetStartInd(0);
  cache_object_ptr->SetEndInd(1);
  ProjectionCache& projection_cache = track_object_distance.projection_cache_;
  projection_cache.Reset("lidar", 0.0);
  Eigen::Vector2f test_pt(2, 3);
  projection_cache.AddPoint(test_pt);
  projection_cache.Reset("lidar", 0.0);
  cache_object_ptr = projection_cache.BuildObject("lidar", 0.0,
      "front_camera", 0.0, 1);

  projection_cache.Reset("lidar1", 0.0);
  lidar_sensor_frame_ptr = (nullptr);
  lidar_sensor_object_ptr.reset(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));
  EXPECT_FALSE(track_object_distance.QueryProjectionCacheObject(
      lidar_sensor_object_ptr, camera_sensor_object_ptr,
      pinhole_camera_model_ptr, true) != nullptr);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_query_projection_cache_object_3) {
  TrackObjectDistance track_object_distance;
  track_object_distance.ResetProjectionCache("lidar", 0.0);
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->id = 1;
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::PinholeCameraModelPtr pinhole_camera_model_ptr(
      new base::PinholeCameraModel);
  pinhole_camera_model_ptr->set_width(100);
  pinhole_camera_model_ptr->set_height(100);
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Identity();
  pinhole_camera_model_ptr->set_intrinsic_params(intrinsic_params);

  EXPECT_TRUE(track_object_distance.QueryProjectionCacheObject(
      lidar_sensor_object_ptr, camera_sensor_object_ptr,
      pinhole_camera_model_ptr, true) != nullptr);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_query_projection_cache_object_4) {
  TrackObjectDistance track_object_distance;
  track_object_distance.ResetProjectionCache("lidar", 0.0);
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->id = 1;
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  for (size_t i = 0; i <= 20; ++i) {
    for (size_t j = 0; j <= 20; ++j) {
      for (size_t k = 0; k < 4; ++k) {
        base::PointF pt;
        pt.x = 20 + i * 0.2;
        pt.y = 20 + j * 0.2;
        pt.z = k * 1;
        pt.intensity = 1;
        cloud.push_back(pt);
      }
    }
  }
  lidar_object_ptr->center = Eigen::Vector3d(22, 22, 1);
  lidar_object_ptr->direction = Eigen::Vector3f(1, 0, 0);
  lidar_object_ptr->size = Eigen::Vector3f(4, 4, 2);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::PinholeCameraModelPtr pinhole_camera_model_ptr(
      new base::PinholeCameraModel);
  pinhole_camera_model_ptr->set_width(100);
  pinhole_camera_model_ptr->set_height(100);
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Identity();
  pinhole_camera_model_ptr->set_intrinsic_params(intrinsic_params);

  EXPECT_TRUE(track_object_distance.QueryProjectionCacheObject(
      lidar_sensor_object_ptr, camera_sensor_object_ptr,
      pinhole_camera_model_ptr, true) != nullptr);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_similiarity_0) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera1";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_EQ(track_object_distance.ComputeRadarCameraSimilarity(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 0.0);

  delete radar_info_ptr;
  radar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_similiarity_1) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 1.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  // camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_EQ(track_object_distance.ComputeRadarCameraSimilarity(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 0.0);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_similiarity_2) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(2, 2, 2);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 100, 100);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_EQ(track_object_distance.ComputeRadarCameraSimilarity(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 0.0);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_similiarity_3) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(-1, -1, -1);
  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(2, 2, 2);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 100, 100);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  // std::cout << camera_frame_ptr->sensor2world_pose;
  // camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_EQ(track_object_distance.ComputeRadarCameraSimilarity(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 0.0);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_similiarity_4) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1.001, 1, 1);
  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.01, 1.01, 1.01);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 100, 100);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  // std::cout << camera_frame_ptr->sensor2world_pose;
  // camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_LT(track_object_distance.ComputeRadarCameraSimilarity(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 1e-3);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_similiarity_5) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  // std::cout << camera_frame_ptr->sensor2world_pose;
  // camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_GT(track_object_distance.ComputeRadarCameraSimilarity(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 0.9);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_similiarity_0) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera1";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_EQ(track_object_distance.ComputeLidarCameraSimilarity(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true), 0.0);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_similiarity_1) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 1.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_EQ(track_object_distance.ComputeLidarCameraSimilarity(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true), 0.0);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_similiarity_2) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr = nullptr;
  // lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
  //   lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_EQ(track_object_distance.ComputeLidarCameraSimilarity(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true), 0.0);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_similiarity_3) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_EQ(track_object_distance.ComputeLidarCameraSimilarity(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true), 0.0);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_similiarity_4) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  lidar_object_ptr->id = 1;

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  track_object_distance.projection_cache_.Reset("lidar", 0.0); //= new
ProjectionCache("lidar", 0.0); ProjectionCache& projection_cache =
  track_object_distance.projection_cache_;
  // projection_cache.Reset("lidar", 0.0);
  Eigen::Vector2f test_pt(2, 3);
  projection_cache.AddPoint(test_pt);
  // projection_cache.Reset("lidar", 0.0);
  ProjectionCacheObject* cache_object_ptr =
      projection_cache.BuildObject("lidar", 0.0, "front_camera", 0.0, 1);

  EXPECT_NEAR(0, track_object_distance.ComputeLidarCameraSimilarity(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true), 1e-2);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_similiarity_5) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  lidar_object_ptr->id = 1;

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_NEAR(track_object_distance.ComputeLidarCameraSimilarity(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true), 0.0, 1e-3);
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_0) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera1";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_EQ(track_object_distance.ComputeRadarCamera(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 4);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_1) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 1.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  EXPECT_EQ(track_object_distance.ComputeRadarCamera(
      radar_sensor_object_ptr, camera_sensor_object_ptr), 4);
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_2) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, -1);
  radar_object_ptr->id = 1;

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  float ans_val = track_object_distance.ComputeRadarCamera(
      radar_sensor_object_ptr, camera_sensor_object_ptr);
  EXPECT_GT(ans_val, 4.4);
  EXPECT_LT(ans_val, 4.5);
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_3) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1.1, 1.1, 1);
  radar_object_ptr->id = 1;

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  float ans_val = track_object_distance.ComputeRadarCamera(
      radar_sensor_object_ptr, camera_sensor_object_ptr);
  EXPECT_GT(ans_val, 4.4);
  EXPECT_LT(ans_val, 4.5);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_radar_camera_4) {
  TrackObjectDistance track_object_distance;
  common::FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  float ans_val = track_object_distance.ComputeRadarCamera(
      radar_sensor_object_ptr, camera_sensor_object_ptr);
  EXPECT_GT(ans_val, 0);
  EXPECT_LT(ans_val, 1);
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_0) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera1";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_EQ(track_object_distance.ComputeLidarCamera(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true, false), 4);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_1) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 1.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_EQ(track_object_distance.ComputeLidarCamera(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true, false), 4);
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_2) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr = nullptr;
  // SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  // lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
  //   lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_EQ(track_object_distance.ComputeLidarCamera(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true, false), 4);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_3) {
  TrackObjectDistance track_object_distance;
  track_object_distance.ResetProjectionCache("lidar", 0.0);
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  lidar_object_ptr->id = 1;

  lidar_object_ptr->polygon.push_back(pt1);
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_GT(track_object_distance.ComputeLidarCamera(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true, false), 4);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_4) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, -1);
  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
    camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
    new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));
  EXPECT_GT(track_object_distance.ComputeLidarCamera(
    lidar_sensor_object_ptr, camera_sensor_object_ptr, true, true), 1e20);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_5) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, -1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  lidar_object_ptr->id = 1;

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  track_object_distance.projection_cache_.Reset("lidar", 0.0);
  ProjectionCache& projection_cache = track_object_distance.projection_cache_;
  Eigen::Vector2f test_pt(pt.x, pt.y);
  projection_cache.AddPoint(test_pt);
  ProjectionCacheObject* cache_object_ptr =
      projection_cache.BuildObject("lidar", 0.0, "front_camera", 0.0, 1);
  cache_object_ptr->SetStartInd(0);
  cache_object_ptr->SetEndInd(1);
  cache_object_ptr->SetBox(base::BBox2DF(pt.x, pt.y, pt.x, pt.y));

  EXPECT_GT(track_object_distance.ComputeLidarCamera(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true, true), 4);
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_lidar_camera_6) {
  TrackObjectDistance track_object_distance;
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, -1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  lidar_object_ptr->id = 1;

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  track_object_distance.projection_cache_.Reset("lidar", 0.0);
  //new ProjectionCache("lidar", 0.0);
  ProjectionCache& projection_cache = track_object_distance.projection_cache_;
  Eigen::Vector2f test_pt(2, 3);
  projection_cache.AddPoint(test_pt);
  ProjectionCacheObject* cache_object_ptr =
      projection_cache.BuildObject("lidar", 0.0, "front_camera", 0.0, 1);
  cache_object_ptr->SetStartInd(0);
  cache_object_ptr->SetEndInd(1);
  cache_object_ptr->SetBox(base::BBox2DF(pt.x, pt.y, pt.x, pt.y));
  EXPECT_GT(track_object_distance.ComputeLidarCamera(
      lidar_sensor_object_ptr, camera_sensor_object_ptr, true, true), 4);
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_0) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, -1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  lidar_object_ptr->id = 1;

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  TrackObjectDistanceOptions options;
  float val = track_object_distance.Compute(fused_track,
      lidar_sensor_object_ptr, options);
  EXPECT_EQ(val, std::numeric_limits<float>::max());

  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_1) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  lidar_object_ptr->id = 1;

  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  lidar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(lidar_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;
  float val = track_object_distance.Compute(fused_track,
      lidar_sensor_object_ptr, options);
  EXPECT_EQ(val, 0);

  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_2) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  radar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  lidar_object_ptr->id = 1;

  lidar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(radar_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      lidar_sensor_object_ptr, options);
  EXPECT_FLOAT_EQ(val, 0);

  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_3) {
  TrackObjectDistance track_object_distance;
  track_object_distance.ResetProjectionCache("lidar", 0.0);
  TrackPtr fused_track(new Track());

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  // build camera object
  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  // build lidar object
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  lidar_object_ptr->id = 1;
  lidar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  // prepare to fuse
  fused_track->UpdateWithSensorObject(camera_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(
      fused_track, lidar_sensor_object_ptr, options);
  EXPECT_GT(val, 4);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_4) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  lidar_object_ptr->id = 1;

  lidar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      lidar_sensor_object_ptr, options);
  EXPECT_EQ(val, std::numeric_limits<float>::max());

  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_5) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  radar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(camera_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      radar_sensor_object_ptr, options);
  EXPECT_GT(val, 0);
  EXPECT_LT(val, 1);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_6) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  radar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(radar_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      radar_sensor_object_ptr, options);
  EXPECT_EQ(val, std::numeric_limits<float>::max());
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_7) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  lidar_object_ptr->id = 1;

  lidar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;

  radar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(lidar_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      radar_sensor_object_ptr, options);
  EXPECT_EQ(val, 0);

  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;

  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_8) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());
  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  lidar_object_ptr->id = 1;

  lidar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;

  radar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      radar_sensor_object_ptr, options);
  EXPECT_EQ(val, std::numeric_limits<float>::max());
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_9) {
  TrackObjectDistance track_object_distance;
  track_object_distance.ResetProjectionCache("front_camera", 0.0);
  TrackPtr fused_track(new Track());

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::ObjectPtr lidar_object_ptr(new base::Object());
  lidar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  base::PointFCloud& cloud = lidar_object_ptr->lidar_supplement.cloud;
  base::PointF pt;
  pt.x = 100;
  pt.y = 1;
  pt.z = 1;
  pt.intensity = 1;
  cloud.push_back(pt);
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  lidar_object_ptr->id = 1;
  lidar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr lidar_object_const_ptr(lidar_object_ptr);
  base::SensorInfo* lidar_info_ptr = new base::SensorInfo;
  lidar_info_ptr->name = "lidar";
  lidar_info_ptr->type = base::SensorType::VELODYNE_64;
  SensorPtr lidar_sensor_ptr(new Sensor(*lidar_info_ptr));
  base::FramePtr lidar_frame_ptr(new base::Frame());
  lidar_frame_ptr->timestamp = 0.0;
  lidar_frame_ptr->sensor_info = *lidar_info_ptr;
  lidar_sensor_ptr->AddFrame(lidar_frame_ptr);
  SensorFramePtr lidar_sensor_frame_ptr(new SensorFrame());
  lidar_sensor_frame_ptr->Initialize(base::FrameConstPtr(lidar_frame_ptr),
      lidar_sensor_ptr);
  SensorObjectPtr lidar_sensor_object_ptr(
      new SensorObject(lidar_object_const_ptr, lidar_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(lidar_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      camera_sensor_object_ptr, options);
  EXPECT_GT(val, 4);

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete lidar_info_ptr;
  lidar_info_ptr = nullptr;
}
TEST_F(TrackObjectDistanceTest, test_compute_10) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  base::ObjectPtr radar_object_ptr(new base::Object());
  radar_object_ptr->center = Eigen::Vector3d(1, 1, 1);
  radar_object_ptr->id = 1;
  base::PointD pt1;
  pt1.x = 100;
  pt1.y = 1;
  pt1.z = 1;
  pt1.intensity = 1;
  radar_object_ptr->polygon.push_back(pt1);

  base::ObjectConstPtr radar_object_const_ptr(radar_object_ptr);
  base::SensorInfo* radar_info_ptr = new base::SensorInfo;
  radar_info_ptr->name = "radar";
  radar_info_ptr->type = base::SensorType::LONG_RANGE_RADAR;
  SensorPtr radar_sensor_ptr(new Sensor(*radar_info_ptr));
  base::FramePtr radar_frame_ptr(new base::Frame());
  radar_frame_ptr->timestamp = 0.0;
  radar_frame_ptr->sensor_info = *radar_info_ptr;
  radar_sensor_ptr->AddFrame(radar_frame_ptr);
  SensorFramePtr radar_sensor_frame_ptr(new SensorFrame());
  radar_sensor_frame_ptr->Initialize(base::FrameConstPtr(radar_frame_ptr),
      radar_sensor_ptr);
  SensorObjectPtr radar_sensor_object_ptr(
      new SensorObject(radar_object_const_ptr, radar_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(radar_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      camera_sensor_object_ptr, options);
  EXPECT_EQ(val, (std::numeric_limits<float>::max)());

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
  delete radar_info_ptr;
  radar_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_11) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  fused_track->UpdateWithSensorObject(camera_sensor_object_ptr);

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      camera_sensor_object_ptr, options);
  EXPECT_EQ(val, std::numeric_limits<float>::max());

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

TEST_F(TrackObjectDistanceTest, test_compute_12) {
  TrackObjectDistance track_object_distance;
  TrackPtr fused_track(new Track());

  base::ObjectPtr camera_object_ptr(new base::Object());
  camera_object_ptr->center = Eigen::Vector3d(1.001, 1.001, 1.001);
  camera_object_ptr->camera_supplement.Reset();
  camera_object_ptr->camera_supplement.box = base::BBox2DF(0, 0, 2, 2);

  base::ObjectConstPtr camera_object_const_ptr(camera_object_ptr);
  base::SensorInfo* camera_info_ptr = new base::SensorInfo;
  camera_info_ptr->name = "front_camera";
  camera_info_ptr->type = base::SensorType::MONOCULAR_CAMERA;
  SensorPtr camera_sensor_ptr(new Sensor(*camera_info_ptr));
  base::FramePtr camera_frame_ptr(new base::Frame());
  camera_frame_ptr->timestamp = 0.0;
  camera_frame_ptr->sensor_info = *camera_info_ptr;
  camera_sensor_ptr->AddFrame(camera_frame_ptr);
  SensorFramePtr camera_sensor_frame_ptr(new SensorFrame());
  camera_sensor_frame_ptr->Initialize(base::FrameConstPtr(camera_frame_ptr),
      camera_sensor_ptr);
  SensorObjectPtr camera_sensor_object_ptr(
      new SensorObject(camera_object_const_ptr, camera_sensor_frame_ptr));

  TrackObjectDistanceOptions options;
  Eigen::Vector3d ref_point(100, 100, 100);
  options.ref_point = &ref_point;

  float val = track_object_distance.Compute(fused_track,
      camera_sensor_object_ptr, options);
  EXPECT_EQ(val, std::numeric_limits<float>::max());

  delete camera_info_ptr;
  camera_info_ptr = nullptr;
}

*/

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
