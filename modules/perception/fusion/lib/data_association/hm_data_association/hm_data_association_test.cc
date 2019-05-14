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

#include "modules/perception/base/frame.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/sensor.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/fusion/base/sensor_object.h"
#include "modules/perception/fusion/base/track.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/probabilities.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/projection_cache.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/track_object_distance.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/track_object_similarity.h"

namespace apollo {
namespace perception {
namespace fusion {

/*
TODO(all): not compiling. to be fixed

TEST(ProjectionCacheTest, test_projection_cache_basic_function) {
  ProjectionCacheObject projection_cache_object;
  projection_cache_object.SetStartInd(2);
  projection_cache_object.SetEndInd(5);
  EXPECT_EQ(projection_cache_object.GetStartInd(), 2);
  EXPECT_EQ(projection_cache_object.GetEndInd(), 5);
  EXPECT_FALSE(projection_cache_object.Empty());
  EXPECT_EQ(projection_cache_object.Size(), 3);
}

TEST(ProjectionCacheFrame, test_projection_cache_query_object_1) {
  ProjectionCacheObject* cache_object_ptr_1 = new ProjectionCacheObject;
  cache_object_ptr_1->SetStartInd(2);
  cache_object_ptr_1->SetEndInd(5);
  EXPECT_EQ(cache_object_ptr_1->GetStartInd(), 2);
  EXPECT_EQ(cache_object_ptr_1->GetEndInd(), 5);
  ProjectionCacheFrame projection_cache_frame("lidar", 1.0);
  ProjectionCacheObject* cache_object_ptr_2 =
      projection_cache_frame.BuildObject(2);
  EXPECT_TRUE(projection_cache_frame.VerifyKey("lidar", 1.0));
  EXPECT_FALSE(projection_cache_frame.VerifyKey("lidar", 2.0));
  EXPECT_FALSE(projection_cache_frame.VerifyKey("lidar2", 2.0));
  EXPECT_FALSE(projection_cache_frame.VerifyKey("lidar2", 1.0));
  EXPECT_NE(projection_cache_frame.QueryObject(2), nullptr);
  EXPECT_EQ(projection_cache_frame.QueryObject(1), nullptr);
  delete cache_object_ptr_1;
  cache_object_ptr_1 = nullptr;
}

TEST(ProjectionCache, test_projection_cache_query_object_2) {
  ProjectionCache projection_cache;
  Eigen::Vector2f test_pt(2, 3);
  projection_cache.AddPoint(test_pt);
  EXPECT_EQ(projection_cache.GetPoint2dsSize(), 1);
  projection_cache.Reset("lidar", 1.0);
  ProjectionCacheObject* cache_object_ptr = projection_cache.BuildObject(
      "lidar", 1.0, "camera_obstacle", 1.0, 1);
  EXPECT_FALSE(projection_cache.QueryObject("lidar", 2.0,
      "camera_obstacle", 1.0, 1) != nullptr);
  EXPECT_FALSE(projection_cache.QueryObject("lidar", 1.0,
      "camera_obstacle", 2.0, 1) != nullptr);
  EXPECT_FALSE(projection_cache.QueryObject("lidar", 1.0,
      "camera_obstacle", 1.0, 2) != nullptr);
  EXPECT_TRUE(projection_cache.QueryObject("lidar", 1.0,
      "camera_obstacle", 1.0, 1) != nullptr);
  cache_object_ptr = nullptr;
}

TEST(BoundedScalePositiveProbability, test_probability_1) {
  double p = 0.2;
  double max_p = 0.4;
  double min_p = 0.3;
  EXPECT_EQ(BoundedScalePositiveProbability(p, max_p, min_p), min_p);
}

TEST(ScalePositiveProbability, test_probability_2) {
  double p = 0.2;
  double max_p = 0.4;
  double th_p = 0.3;
  EXPECT_EQ(ScalePositiveProbability(p, max_p, th_p), p);
  p = 1;
  EXPECT_EQ(ScalePositiveProbability(p, max_p, th_p), max_p);
}

TEST(WelshVarLossFun, test_probability_3) {
  double p = 0;
  EXPECT_EQ(WelshVarLossFun(p, 0.2, 0.5), 1 - 1e-6);
  p  = 20.0;
  EXPECT_LT(WelshVarLossFun(p, 0, 1), 1e-3);
}

TEST(FuseTwoProbabilities, test_probability_3) {
  double p1 = 1.0;
  double p2 = 1.0;
  EXPECT_EQ(FuseTwoProbabilities(p1, p2), 1.0);
}

TEST(FuseMultipleProbabilities, test_probability_4) {
  std::vector<double> probs;
  probs.push_back(1.0);
  probs.push_back(1.0);
  probs.push_back(1.0);
  EXPECT_GT(FuseMultipleProbabilities(probs), 1-1e-3);
}

TEST(ComputePtsBoxLocationSimilarity, test_compute_pts_box_location_sim) {
  ProjectionCachePtr cache_ptr = new ProjectionCache;
  ProjectionCacheObject* cache_object_ptr = new ProjectionCacheObject;
  base::BBox2DF camera_bbox = base::BBox2DF(100, 100, 200, 200);
  EXPECT_EQ(ComputePtsBoxLocationSimilarity(cache_ptr,
      cache_object_ptr, camera_bbox), 1e-6);
  cache_ptr->AddPoint(Eigen::Vector2f(101, 101));
  cache_ptr->AddPoint(Eigen::Vector2f(101, 102));
  cache_object_ptr->SetStartInd(0);
  cache_object_ptr->SetEndInd(2);
  EXPECT_GT(ComputePtsBoxLocationSimilarity(cache_ptr,
      cache_object_ptr, camera_bbox), 1 - 1e-3);
  delete cache_ptr;
  cache_ptr = nullptr;
  delete cache_object_ptr;
  cache_object_ptr = nullptr;
}

TEST(ComputePtsBoxShapeSimilarity, test_compute_pts_box_shape_sim) {
  ProjectionCachePtr cache_ptr = new ProjectionCache;
  base::BBox2DF camera_bbox = base::BBox2DF(100, 100, 200, 200);
  ProjectionCacheObject* cache_object_ptr = new ProjectionCacheObject;
  EXPECT_EQ(ComputePtsBoxShapeSimilarity(cache_ptr,
      cache_object_ptr, camera_bbox), 1e-3);
  cache_object_ptr->SetStartInd(0);
  cache_object_ptr->SetEndInd(2);
  cache_object_ptr->SetBox(base::BBox2DF(100, 100, 200, 200));
  cache_ptr->AddPoint(Eigen::Vector2f(100, 100));
  cache_ptr->AddPoint(Eigen::Vector2f(200, 200));
  EXPECT_GE(ComputePtsBoxShapeSimilarity(cache_ptr,
      cache_object_ptr, camera_bbox), 1 - 1e-3);
  delete cache_ptr;
  cache_ptr = nullptr;
  delete cache_object_ptr;
  cache_object_ptr = nullptr;
}

TEST(ComputePtsBoxSimilarity, test_compute_pts_box_sim) {
  ProjectionCachePtr cache_ptr = new ProjectionCache;
  ProjectionCacheObject* cache_object_ptr = new ProjectionCacheObject;
  base::BBox2DF camera_bbox = base::BBox2DF(100, 100, 200, 200);
  cache_ptr->AddPoint(Eigen::Vector2f(100, 100));
  cache_ptr->AddPoint(Eigen::Vector2f(200, 200));
  cache_object_ptr->SetStartInd(0);
  cache_object_ptr->SetEndInd(2);
  cache_object_ptr->SetBox(base::BBox2DF(100, 100, 200, 200));
  EXPECT_GT(ComputePtsBoxSimilarity(cache_ptr,
      cache_object_ptr, camera_bbox), 1 - 1e-3);
  delete cache_ptr;
  cache_ptr = nullptr;
  delete cache_object_ptr;
  cache_object_ptr = nullptr;
}

TEST(ComputeRadarCameraXSimilarity, test_compute_radar_camera_x_sim) {
  double velo_ct_x = 100;
  double camera_ct_x = 100;
  double size_x = 100;
  XSimilarityParams x_similarity_param;
  EXPECT_GT(ComputeRadarCameraXSimilarity(velo_ct_x, camera_ct_x,
      size_x, x_similarity_param),
  x_similarity_param.scale_positive_max_p_ - 1e-3);
}

TEST(ComputeRadarCameraYSimilarity, test_compute_radar_camera_y_sim) {
  double velo_ct_y = 110;
  double camera_ct_y = 100;
  double size_y = 100;
  YSimilarityParams y_similarity_param;
  EXPECT_GT(ComputeRadarCameraYSimilarity(velo_ct_y, camera_ct_y,
      size_y, y_similarity_param),
  y_similarity_param.bounded_scale_positive_max_p_ - 1e-3);
}

TEST(ComputeRadarCameraHSimilarity, test_compute_radar_camera_h_sim) {
  SensorObjectConstPtr sensor_radar_ptr;
  std::vector<Eigen::Vector2d> radar_box2d_vertices;
  radar_box2d_vertices.push_back(Eigen::Vector2d(0, 0));
  radar_box2d_vertices.push_back(Eigen::Vector2d(1, 0));
  radar_box2d_vertices.push_back(Eigen::Vector2d(2, 0));
  radar_box2d_vertices.push_back(Eigen::Vector2d(3, 0));
  radar_box2d_vertices.push_back(Eigen::Vector2d(0, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(1, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(2, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(3, 100));

  double size_y = 100;
  base::ObjectPtr camera_object_ptr =
  std::shared_ptr<base::Object>(new base::Object);
  camera_object_ptr->size = Eigen::Vector3f(0, 0, 100);
  base::ObjectConstPtr camera_const_object_ptr(camera_object_ptr);
  const SensorFramePtr camera_frame_ptr = nullptr;
  SensorObjectConstPtr camera_const_sensor_ptr(
      new SensorObject(camera_const_object_ptr, camera_frame_ptr));
  HSimilarityParams h_similiar_params;
  EXPECT_GT(ComputeRadarCameraHSimilarity(sensor_radar_ptr,
      camera_const_sensor_ptr, size_y, radar_box2d_vertices,
      h_similiar_params), h_similiar_params.scale_positive_max_p_ - 1e-3);
}

TEST(ComputeRadarCameraWSimilarity, test_compute_radar_camera_w_sim) {
  std::vector<Eigen::Vector2d> radar_box2d_vertices;
  radar_box2d_vertices.push_back(Eigen::Vector2d(0, 0));
  radar_box2d_vertices.push_back(Eigen::Vector2d(100, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(0, 0));
  radar_box2d_vertices.push_back(Eigen::Vector2d(100, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(0, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(1, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(2, 100));
  radar_box2d_vertices.push_back(Eigen::Vector2d(3, 100));

  double width = 300;
  double size_x = 100;
  const SensorObjectConstPtr radar;
  WSimilarityParams w_similarity_params;
  EXPECT_GT(ComputeRadarCameraWSimilarity(radar, width, size_x,
      radar_box2d_vertices, w_similarity_params),
      w_similarity_params.bounded_scale_positive_max_p_ - 1e-3);
}

TEST(ComputeRadarCameraLocSimilarity, test_compute_radar_camera_loc_sim) {
  base::ObjectPtr camera_object_ptr =
  std::shared_ptr<base::Object>(new base::Object);
  camera_object_ptr->center = Eigen::Vector3d(0, 0, 100);
  base::ObjectConstPtr camera_const_object_ptr(camera_object_ptr);
  const SensorFramePtr camera_frame_ptr = nullptr;
  SensorObjectConstPtr camera_const_sensor_ptr(
      new SensorObject(camera_const_object_ptr, camera_frame_ptr));
  Eigen::Vector3d radar_ct(0, 0, 100);
  Eigen::Matrix4d world2camera_pose = Eigen::Matrix4d::Identity();
  LocSimilarityParams loc_similiarity_params;
  EXPECT_GT(ComputeRadarCameraLocSimilarity(radar_ct,
      camera_const_sensor_ptr, world2camera_pose, loc_similiarity_params),
      loc_similiarity_params.scale_positive_max_p_ - 1e-3);
}

TEST(ComputeRadarCameraVelocitySimilarity, test_compute_radar_camera_vel_sim) {
  base::ObjectPtr camera_object_ptr =
      std::shared_ptr<base::Object>(new base::Object);
  camera_object_ptr->velocity = Eigen::Vector3f(0, 0, 21);
  base::ObjectConstPtr camera_const_object_ptr(camera_object_ptr);
  const SensorFramePtr camera_frame_ptr = nullptr;
  SensorObjectConstPtr camera_const_sensor_ptr(
      new SensorObject(camera_const_object_ptr, camera_frame_ptr));

  base::ObjectPtr radar_object_ptr =
      std::shared_ptr<base::Object>(new base::Object);
  radar_object_ptr->velocity = Eigen::Vector3f(0, 0, 16);
  base::ObjectConstPtr radar_const_object_ptr(radar_object_ptr);
  const SensorFramePtr radar_frame_ptr = nullptr;
  SensorObjectConstPtr radar_const_sensor_ptr(
      new SensorObject(radar_const_object_ptr, radar_frame_ptr));

  EXPECT_LT(ComputeRadarCameraVelocitySimilarity(radar_const_sensor_ptr,
camera_const_sensor_ptr), 0.8);
}

TEST(TrackObjectDistance, test) {
  common::FLAGS_obs_sensor_meta_path = "./data/sensor_meta.pt";
  common::FLAGS_obs_sensor_intrinsic_path = "./params";
  TrackObjectDistance track_object_distance;

}*/

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
