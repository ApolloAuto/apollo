/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar_detection/lib/detector/conti_ars_detector/conti_ars_detector.h"

#include "gtest/gtest.h"

#include "modules/perception/radar_detection/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

TEST(ContiArsDetector, detect) {
  drivers::ContiRadar corrected_obstacles;
  corrected_obstacles.mutable_header()->set_timestamp_sec(151237772.355345434);
  drivers::ContiRadarObs* conti_obs = corrected_obstacles.add_contiobs();
  conti_obs->set_clusterortrack(0);
  conti_obs->set_obstacle_id(80);
  conti_obs->set_longitude_dist(20);
  conti_obs->set_lateral_dist(10);
  conti_obs->set_longitude_vel(10);
  conti_obs->set_lateral_vel(5);
  conti_obs->set_rcs(15);
  conti_obs->set_dynprop(0);
  conti_obs->set_probexist(0.8);
  conti_obs->set_longitude_dist_rms(0.2);
  conti_obs->set_lateral_dist_rms(0.1);
  conti_obs->set_longitude_vel_rms(0.2);
  conti_obs->set_lateral_vel_rms(0.1);
  conti_obs->set_oritation_angle(10);
  conti_obs->set_oritation_angle_rms(2.0);
  conti_obs->set_length(2.0);
  conti_obs->set_width(1.0);
  conti_obs->set_obstacle_class(CONTI_TRUCK);
  conti_obs->set_meas_state(2);

  DetectorOptions options;
  Eigen::Matrix4d pose;
  pose << 0, -1, 0, 4, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix4d radar2novatel_trans;
  radar2novatel_trans << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  options.radar2world_pose = &pose;
  options.radar2novatel_trans = &radar2novatel_trans;
  options.car_linear_speed = Eigen::Vector3f(3, 1, 0);
  options.car_angular_speed = Eigen::Vector3f(0, 0, 0);

  auto radar_frame = std::shared_ptr<base::Frame>(new base::Frame);
  ContiArsDetector detector;
  DetectorInitOptions init_options;
  detector.Init(init_options);
  detector.Detect(corrected_obstacles, options, radar_frame);
  EXPECT_LT(fabs(radar_frame->timestamp - 151237772.355345434), 1.0e-6);
  Eigen::Matrix4d diff_pose = radar_frame->sensor2world_pose.matrix() - pose;
  EXPECT_LT(diff_pose.norm(), 1.0e-6);
  base::ObjectPtr radar_object = radar_frame->objects.front();
  EXPECT_EQ(radar_object->id, 80);
  Eigen::Vector3d world_loc(-6, 21, 0);
  EXPECT_LT((world_loc - radar_object->center).norm(), 1.0e-6);
  Eigen::Vector3f world_vel(-2, 11, 0);
  EXPECT_LT((world_vel - radar_object->velocity).norm(), 1.0e-6);

  Eigen::Vector3f size(2.0, 1.0, 2.0);
  EXPECT_LT((size - radar_object->size).norm(), 1.0e-6);
  EXPECT_EQ(radar_object->type, base::ObjectType::VEHICLE);
  EXPECT_FLOAT_EQ(radar_object->confidence, 0.8);

  Eigen::Matrix3f dist_uncertain;
  Eigen::Matrix3f vel_uncertain;
  dist_uncertain << 0.01f, 0.0f, 0.0f, 0.0f, 0.04f, 0.0f, 0.0f, 0.0f, 0.0f;
  vel_uncertain << 0.01f, 0.0f, 0.0f, 0.0f, 0.04f, 0.0f, 0.0f, 0.0f, 0.0f;
  float dist_diff = (dist_uncertain - radar_object->center_uncertainty).norm();
  float vel_diff = (vel_uncertain - radar_object->velocity_uncertainty).norm();
  EXPECT_LT(dist_diff, 1.0e-6);
  EXPECT_LT(vel_diff, 1.0e-6);

  auto radar_frame1 = std::shared_ptr<base::Frame>(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_CAR);
  detector.Detect(corrected_obstacles, options, radar_frame1);
  radar_object = radar_frame1->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::VEHICLE);

  auto radar_frame2 = std::shared_ptr<base::Frame>(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_PEDESTRIAN);
  detector.Detect(corrected_obstacles, options, radar_frame2);
  radar_object = radar_frame2->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::PEDESTRIAN);

  auto radar_frame3 = std::shared_ptr<base::Frame>(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_MOTOCYCLE);
  detector.Detect(corrected_obstacles, options, radar_frame3);
  radar_object = radar_frame3->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::BICYCLE);

  auto radar_frame4 = std::shared_ptr<base::Frame>(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_BICYCLE);
  detector.Detect(corrected_obstacles, options, radar_frame4);
  radar_object = radar_frame4->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::BICYCLE);

  base::FramePtr radar_frame5(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_POINT);
  detector.Detect(corrected_obstacles, options, radar_frame5);
  radar_object = radar_frame5->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::UNKNOWN);

  base::FramePtr radar_frame6(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_WIDE);
  detector.Detect(corrected_obstacles, options, radar_frame6);
  radar_object = radar_frame6->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::UNKNOWN);

  base::FramePtr radar_frame7(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(
      CONTI_TYPE_UNKNOWN);
  detector.Detect(corrected_obstacles, options, radar_frame7);
  radar_object = radar_frame7->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::UNKNOWN);

  base::FramePtr radar_frame8(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_length(0);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_CAR);
  detector.Detect(corrected_obstacles, options, radar_frame8);
  radar_object = radar_frame8->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::VEHICLE);

  base::FramePtr radar_frame9(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_obstacle_class(CONTI_PEDESTRIAN);
  detector.Detect(corrected_obstacles, options, radar_frame9);
  radar_object = radar_frame9->objects.front();
  EXPECT_EQ(radar_object->type, base::ObjectType::PEDESTRIAN);

  base::FramePtr radar_frame10(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(CONTI_MOVING);
  detector.Detect(corrected_obstacles, options, radar_frame10);
  radar_object = radar_frame10->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::MOVING);

  base::FramePtr radar_frame11(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(CONTI_STATIONARY);
  detector.Detect(corrected_obstacles, options, radar_frame11);
  radar_object = radar_frame11->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::STATIONARY);

  base::FramePtr radar_frame12(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(CONTI_ONCOMING);
  detector.Detect(corrected_obstacles, options, radar_frame12);
  radar_object = radar_frame12->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::MOVING);

  base::FramePtr radar_frame13(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(
      CONTI_STATIONARY_CANDIDATE);
  detector.Detect(corrected_obstacles, options, radar_frame13);
  radar_object = radar_frame13->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::STATIONARY);

  base::FramePtr radar_frame14(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(CONTI_DYNAMIC_UNKNOWN);
  detector.Detect(corrected_obstacles, options, radar_frame14);
  radar_object = radar_frame14->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::UNKNOWN);

  auto radar_frame15 = std::shared_ptr<base::Frame>(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(
      CONTI_CROSSING_STATIONARY);
  detector.Detect(corrected_obstacles, options, radar_frame15);
  radar_object = radar_frame15->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::STATIONARY);

  auto radar_frame16 = std::shared_ptr<base::Frame>(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(CONTI_CROSSING_MOVING);
  detector.Detect(corrected_obstacles, options, radar_frame16);
  radar_object = radar_frame16->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::MOVING);

  auto radar_frame17 = std::shared_ptr<base::Frame>(new base::Frame);
  corrected_obstacles.mutable_contiobs(0)->set_dynprop(CONTI_STOPPED);
  detector.Detect(corrected_obstacles, options, radar_frame17);
  radar_object = radar_frame17->objects.front();
  EXPECT_EQ(radar_object->motion_state, base::MotionState::STATIONARY);
}

TEST(ContiArsDetector, name) {
  ContiArsDetector detector;
  EXPECT_EQ(detector.Name(), "ContiArsDetector");
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
