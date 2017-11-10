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
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"

#include <vector>
#include <map>
#include <string>
#include <functional>
#include <algorithm>
#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

class PbfMotionFusionTest : public testing::Test {
 protected:
  PbfMotionFusionTest() {
  }
  ~PbfMotionFusionTest() {
  }
  void SetUp() override {
    PbfBaseMotionFusion *kalman_motion_fusion = new PbfKalmanMotionFusion();
    _motion_fusion_algs.push_back(kalman_motion_fusion);
  }
  void TearDown() override {
    for (auto &seg_alg : _motion_fusion_algs) {
      delete seg_alg;
    }
    std::vector<PbfBaseMotionFusion *>().swap(_motion_fusion_algs);
  }
 protected:
  std::vector<PbfBaseMotionFusion *> _motion_fusion_algs;
};
TEST_F(PbfMotionFusionTest, test_initialize_with_lidar_object) {
  ObjectPtr lidar_object(new Object());
  double lidar_timestamp = 1234567891.012;
  Eigen::Vector3d lidar_position(30, 0, 0);
  Eigen::Vector3d lidar_velocity(10, 0, 0);
  lidar_object->center = lidar_position;
  lidar_object->anchor_point = lidar_position;
  lidar_object->velocity = lidar_velocity;
  PbfSensorObjectPtr pbf_lidar_object(
      new PbfSensorObject(lidar_object, VELODYNE_64, lidar_timestamp));
  for (auto motion_fusion_alg: _motion_fusion_algs) {
    motion_fusion_alg->Initialize(pbf_lidar_object);
    CHECK_EQ(motion_fusion_alg->Initialized(), true);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(location, velocity);
    EXPECT_TRUE((location - lidar_position).norm() < 1.0e-6);
    EXPECT_TRUE((velocity - lidar_velocity).norm() < 1.0e-6);
  }
}
TEST_F(PbfMotionFusionTest, test_initialize_with_radar_object) {
  ObjectPtr radar_object(new Object());
  double radar_timestamp = 1234567891.112;
  Eigen::Vector3d radar_position(31.1, 0, 0);
  Eigen::Vector3d radar_velocity(10.01, 0, 0);
  radar_object->center = radar_position;
  radar_object->anchor_point = radar_position;
  radar_object->velocity = radar_velocity;
  PbfSensorObjectPtr pbf_radar_object(
      new PbfSensorObject(radar_object, RADAR, radar_timestamp));
  for (auto motion_fusion_alg : _motion_fusion_algs) {
    motion_fusion_alg->Initialize(pbf_radar_object);
    CHECK_EQ(motion_fusion_alg->Initialized(), true);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(location, velocity);
    EXPECT_TRUE((location - radar_position).norm() < 1.0e-6);
    EXPECT_TRUE((velocity - radar_velocity).norm() < 1.0e-6);
  }
}
TEST_F(PbfMotionFusionTest, test_update_with_measurement) {
  ObjectPtr radar_object(new Object());
  double radar_timestamp = 1234567891.012;
  Eigen::Vector3d radar_position(30, 0, 0);
  Eigen::Vector3d radar_velocity(10.01, 0, 0);
  radar_object->center = radar_position;
  radar_object->anchor_point = radar_position;
  radar_object->velocity = radar_velocity;
  PbfSensorObjectPtr pbf_radar_object(
      new PbfSensorObject(radar_object, RADAR, radar_timestamp));

  ObjectPtr lidar_object(new Object());
  double lidar_timestamp = 1234567891.112;
  Eigen::Vector3d lidar_position(31.01, 0, 0);
  Eigen::Vector3d lidar_velocity(10, 0, 0);
  lidar_object->center = lidar_position;
  lidar_object->anchor_point = lidar_position;
  lidar_object->velocity = lidar_velocity;
  PbfSensorObjectPtr pbf_lidar_object(
      new PbfSensorObject(lidar_object, VELODYNE_64, lidar_timestamp));
  for (auto motion_fusion_alg : _motion_fusion_algs) {
    motion_fusion_alg->Initialize(pbf_radar_object);
    motion_fusion_alg->UpdateWithObject(pbf_lidar_object, lidar_timestamp - radar_timestamp);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(location, velocity);
    AINFO << "algorithm " << motion_fusion_alg->name() << ": " << location
          << lidar_position << (location - lidar_position).norm();
    EXPECT_TRUE((location - lidar_position).norm() < 2.0e-1);
    EXPECT_TRUE((velocity - lidar_velocity).norm() < 1.0e-1);
  }
}
TEST_F(PbfMotionFusionTest, test_update_without_measurement) {
  ObjectPtr lidar_object(new Object());
  double lidar_timestamp = 1234567891.012;
  Eigen::Vector3d lidar_position(30, 0, 0);
  Eigen::Vector3d lidar_velocity(10, 0, 0);
  lidar_object->center = lidar_position;
  lidar_object->anchor_point = lidar_position;
  lidar_object->velocity = lidar_velocity;
  PbfSensorObjectPtr pbf_lidar_object(
      new PbfSensorObject(lidar_object, VELODYNE_64, lidar_timestamp));

  double time_diff = 0.1;
  Eigen::Vector3d expected_position = lidar_position + lidar_velocity * time_diff;
  for (auto motion_fusion_alg : _motion_fusion_algs) {
    motion_fusion_alg->Initialize(pbf_lidar_object);
    motion_fusion_alg->UpdateWithoutObject(time_diff);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(location, velocity);
    EXPECT_TRUE((location - expected_position).norm() < 1.0e-6);
    EXPECT_TRUE((velocity - lidar_velocity).norm() < 1e-3);
  }
}

} //namespace perception
} //namespace apollo