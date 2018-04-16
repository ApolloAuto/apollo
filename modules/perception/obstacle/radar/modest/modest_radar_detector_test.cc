/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/obstacle/radar/modest/modest_radar_detector.h"

#include "gtest/gtest.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

TEST(ModestRadarDetectorTest, modest_radar_detector_test) {
  FLAGS_work_root = "modules/perception";
  FLAGS_config_manager_path = "./conf/config_manager.config";
  const double time_diff = 0.074;
  ModestRadarDetector *radar_detector = new ModestRadarDetector();
  EXPECT_TRUE(radar_detector != nullptr);
  radar_detector->Init();
  radar_detector->object_builder_.SetDelayFrame(0);
  ContiRadar raw_obstacles;
  auto *header = raw_obstacles.mutable_header();
  header->set_timestamp_sec(123456789.0);
  ContiRadarObs *radar_obs = raw_obstacles.add_contiobs();
  radar_obs->set_longitude_dist(0.0);
  radar_obs->set_lateral_dist(0.0);
  radar_obs->set_longitude_vel(3.0);
  radar_obs->set_lateral_vel(4.0);
  radar_obs->set_oritation_angle(0);
  radar_obs->set_obstacle_id(1);
  radar_obs->set_obstacle_class(static_cast<int>(ContiObjectType::CONTI_CAR));
  radar_obs->set_longitude_dist_rms(0.1);
  radar_obs->set_lateral_dist_rms(0.1);
  radar_obs->set_longitude_vel_rms(0.1);
  radar_obs->set_lateral_vel_rms(0.1);
  radar_obs->set_length(1.0);
  radar_obs->set_width(1.0);
  radar_obs->set_probexist(1.0);
  radar_obs->set_meas_state(static_cast<int>(ContiMeasState::CONTI_NEW));
  RadarDetectorOptions options;
  Eigen::Matrix4d radar2world_pose;
  radar2world_pose << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Vector3f main_car_velocity;
  main_car_velocity[0] = 0.3;
  main_car_velocity[1] = 0.4;
  main_car_velocity[2] = 0.0;
  options.radar2world_pose = &radar2world_pose;
  options.car_linear_speed = main_car_velocity;
  std::vector<PolygonDType> map_polygons;
  map_polygons.resize(1);
  map_polygons[0].points.resize(4);
  map_polygons[0].points[0].x = -20;
  map_polygons[0].points[0].y = -20;
  map_polygons[0].points[1].x = -20;
  map_polygons[0].points[1].y = 20;
  map_polygons[0].points[2].x = 20;
  map_polygons[0].points[2].y = 20;
  map_polygons[0].points[3].x = 20;
  map_polygons[0].points[3].y = -20;
  std::vector<std::shared_ptr<Object>> objects;
  radar_detector->Detect(raw_obstacles, map_polygons, options, &objects);
  EXPECT_EQ(objects.size(), 1);
  EXPECT_TRUE(fabs(objects[0]->center(0) - 0.0) < 1e-5);
  EXPECT_TRUE(fabs(objects[0]->center(1) - 0.0) < 1e-5);
  EXPECT_TRUE(fabs(objects[0]->velocity(0) - 3.3) < 1e-5);
  EXPECT_TRUE(fabs(objects[0]->velocity(1) - 4.4) < 1e-5);
  EXPECT_TRUE(objects[0]->type == ObjectType::UNKNOWN);
  objects.resize(0);
  header->set_timestamp_sec(123456789.074);
  Eigen::Vector2d location(3.0 * time_diff, 4.0 * time_diff);
  radar_obs->set_longitude_dist(location(0));
  radar_obs->set_lateral_dist(location(1));
  radar_detector->Detect(raw_obstacles, map_polygons, options, &objects);
  EXPECT_EQ(objects.size(), 1);
  EXPECT_TRUE(fabs(objects[0]->center(0) - location(0)) < 1e-2);
  EXPECT_TRUE(fabs(objects[0]->center(1) - location(1)) < 1e-2);
  EXPECT_TRUE(fabs(objects[0]->velocity(0) - 3.3) < 1e-2);
  EXPECT_TRUE(fabs(objects[0]->velocity(1) - 4.4) < 1e-2);
  EXPECT_TRUE(objects[0]->type == ObjectType::UNKNOWN);
  delete radar_detector;
}

}  // namespace perception
}  // namespace apollo
