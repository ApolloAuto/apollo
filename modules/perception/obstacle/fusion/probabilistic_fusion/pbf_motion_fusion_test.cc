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

#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_imf_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_kalman_motion_fusion.h"

#include <algorithm>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

class PbfMotionFusionTest : public testing::Test {
 protected:
  PbfMotionFusionTest() {}
  ~PbfMotionFusionTest() {}
  void SetUp() override {
    PbfBaseMotionFusion *kalman_motion_fusion = new PbfKalmanMotionFusion();
    PbfIMFFusion *imf_motion_fusion = new PbfIMFFusion();
    motion_fusion_algs_.push_back(kalman_motion_fusion);
    motion_fusion_algs_.push_back(imf_motion_fusion);
    LidarFrameSupplement::state_vars.initialized_ = true;
    RadarFrameSupplement::state_vars.initialized_ = true;
    CameraFrameSupplement::state_vars.initialized_ = true;
    PbfTrack::SetMaxLidarInvisiblePeriod(10);
    PbfTrack::SetMaxRadarInvisiblePeriod(10);
    PbfTrack::SetMaxCameraInvisiblePeriod(10);
  }
  void TearDown() override {
    for (auto &seg_alg : motion_fusion_algs_) {
      delete seg_alg;
    }
    std::vector<PbfBaseMotionFusion *>().swap(motion_fusion_algs_);
  }

 protected:
  std::vector<PbfBaseMotionFusion *> motion_fusion_algs_;
};

TEST_F(PbfMotionFusionTest, test_initialize_with_lidar_object) {
  std::shared_ptr<Object> lidar_object(new Object());
  double lidar_timestamp = 1234567891.012;
  Eigen::Vector3d lidar_position(30, 0, 0);
  Eigen::Vector3d lidar_velocity(10, 0, 0);
  lidar_object->center = lidar_position;
  lidar_object->anchor_point = lidar_position;
  lidar_object->velocity = lidar_velocity;
  lidar_object->state_uncertainty.setIdentity();
  std::shared_ptr<PbfSensorObject> pbf_lidar_object(new PbfSensorObject(
      lidar_object, SensorType::VELODYNE_64, lidar_timestamp));
  for (auto motion_fusion_alg : motion_fusion_algs_) {
    motion_fusion_alg->Initialize(pbf_lidar_object);
    CHECK_EQ(motion_fusion_alg->Initialized(), true);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(&location, &velocity);
    EXPECT_TRUE((location - lidar_position).norm() < 1.0e-6);
    EXPECT_TRUE((velocity - lidar_velocity).norm() < 1.0e-6);
  }
}
TEST_F(PbfMotionFusionTest, test_initialize_with_radar_object) {
  std::shared_ptr<Object> radar_object(new Object());
  double radar_timestamp = 1234567891.112;
  Eigen::Vector3d radar_position(31.1, 0, 0);
  Eigen::Vector3d radar_velocity(10.01, 0, 0);
  radar_object->center = radar_position;
  radar_object->anchor_point = radar_position;
  radar_object->velocity = radar_velocity;
  std::shared_ptr<PbfSensorObject> pbf_radar_object(
      new PbfSensorObject(radar_object, SensorType::RADAR, radar_timestamp));
  for (auto motion_fusion_alg : motion_fusion_algs_) {
    motion_fusion_alg->Initialize(pbf_radar_object);
    CHECK_EQ(motion_fusion_alg->Initialized(), true);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(&location, &velocity);
    EXPECT_TRUE((location - radar_position).norm() < 1.0e-6);
    EXPECT_TRUE((velocity - radar_velocity).norm() < 1.0e-6);
  }
}
TEST_F(PbfMotionFusionTest, test_update_with_measurement_kalman) {
  std::shared_ptr<Object> radar_object(new Object());
  double radar_timestamp = 1234567891.012;
  Eigen::Vector3d radar_position(30, 0, 0);
  Eigen::Vector3d radar_velocity(10.01, 0, 0);
  radar_object->center = radar_position;
  radar_object->anchor_point = radar_position;
  radar_object->velocity = radar_velocity;
  std::shared_ptr<PbfSensorObject> pbf_radar_object(
      new PbfSensorObject(radar_object, SensorType::RADAR, radar_timestamp));

  std::shared_ptr<Object> lidar_object(new Object());
  double lidar_timestamp = 1234567891.112;
  Eigen::Vector3d lidar_position(31.01, 0, 0);
  Eigen::Vector3d lidar_velocity(10, 0, 0);
  lidar_object->center = lidar_position;
  lidar_object->anchor_point = lidar_position;
  lidar_object->velocity = lidar_velocity;
  std::shared_ptr<PbfSensorObject> pbf_lidar_object(new PbfSensorObject(
      lidar_object, SensorType::VELODYNE_64, lidar_timestamp));
  for (auto motion_fusion_alg : motion_fusion_algs_) {
    if (motion_fusion_alg->name() != "PbfKalmanMotionFusion") {
      continue;
    }
    motion_fusion_alg->setCurrentFuseTS(radar_timestamp + 0.2);
    motion_fusion_alg->Initialize(pbf_radar_object);
    motion_fusion_alg->setLastFuseTS(radar_timestamp + 0.2);
    motion_fusion_alg->setCurrentFuseTS(lidar_timestamp + 0.2);
    motion_fusion_alg->UpdateWithObject(pbf_lidar_object,
                                        lidar_timestamp - radar_timestamp);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(&location, &velocity);
    std::cout << "algorithm " << motion_fusion_alg->name() << ": " << location
              << lidar_position << (location - lidar_position).norm();
    EXPECT_TRUE((location - lidar_position).norm() < 2.0e-1);
    EXPECT_TRUE((velocity - lidar_velocity).norm() < 1.0e-1);
  }
}

TEST_F(PbfMotionFusionTest, test_update_with_measurement_imf) {
  std::shared_ptr<Object> radar_object(new Object());
  double radar_timestamp = 1234567891.012;
  Eigen::Vector3d radar_position(30, 0, 0);
  Eigen::Vector3d radar_velocity(10.01, 0, 0);
  radar_object->center = radar_position;
  radar_object->anchor_point = radar_position;
  radar_object->velocity = radar_velocity;
  std::shared_ptr<PbfSensorObject> pbf_radar_object(
      new PbfSensorObject(radar_object, SensorType::RADAR, radar_timestamp));

  std::shared_ptr<Object> lidar_object(new Object());
  double lidar_timestamp = 1234567891.112;
  Eigen::Vector3d lidar_position(31.01, 0, 0);
  Eigen::Vector3d lidar_velocity(10, 0, 0);
  lidar_object->center = lidar_position;
  lidar_object->anchor_point = lidar_position;
  lidar_object->velocity = lidar_velocity;
  std::shared_ptr<PbfSensorObject> pbf_lidar_object(new PbfSensorObject(
      lidar_object, SensorType::VELODYNE_64, lidar_timestamp));

  for (auto motion_fusion_alg : motion_fusion_algs_) {
    if (motion_fusion_alg->name() != "PbfInformationMotionFusion") continue;

    motion_fusion_alg->setCurrentFuseTS(radar_timestamp);
    motion_fusion_alg->Initialize(pbf_radar_object);
    motion_fusion_alg->setLastFuseTS(radar_timestamp);
    motion_fusion_alg->setCurrentFuseTS(lidar_timestamp + 0.2);
    motion_fusion_alg->UpdateWithObject(
        pbf_lidar_object, lidar_timestamp - radar_timestamp + 0.2);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(&location, &velocity);
    double time_diff = 0.2;
    AINFO << "algorithm " << motion_fusion_alg->name() << ": " << location
          << lidar_position + time_diff * lidar_velocity << velocity
          << lidar_velocity;
    EXPECT_TRUE(
        (location - (lidar_position + time_diff * lidar_velocity)).norm() <
        2.0e-1);
    EXPECT_TRUE((velocity - lidar_velocity).norm() < 1.0e-1);
  }
}

TEST_F(PbfMotionFusionTest, test_update_with_measurement_imf_seq) {
  std::shared_ptr<Object> radar_object(new Object());
  double radar_timestamp = 1234567891.012;
  Eigen::Vector3d radar_position(30, 0, 0);
  Eigen::Vector3d radar_velocity(10.01, 0, 0);
  radar_object->center = radar_position;
  radar_object->anchor_point = radar_position;
  radar_object->velocity = radar_velocity;
  std::shared_ptr<PbfSensorObject> pbf_radar_object(
      new PbfSensorObject(radar_object, SensorType::RADAR, radar_timestamp));
  pbf_radar_object->timestamp = radar_timestamp;
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0, 2);

  int steps = 10;

  for (auto motion_fusion_alg : motion_fusion_algs_) {
    if (motion_fusion_alg->name() != "PbfInformationMotionFusion") continue;

    motion_fusion_alg->setCurrentFuseTS(radar_timestamp);
    motion_fusion_alg->Initialize(pbf_radar_object);
    motion_fusion_alg->setLastFuseTS(radar_timestamp);
    AINFO << "algorithm " << motion_fusion_alg->name() << " long sequence test";
    double mutable_radar_timestamp = radar_timestamp;
    Eigen::Vector3d ground_truth_location = radar_position;
    for (int i = 0; i < steps; ++i) {
      mutable_radar_timestamp += 0.1;
      // double measure_position_noise = distribution(generator);
      Eigen::Vector3d measure_position_noise_vec;
      // measure_position_noise_vec << measure_position_noise, 0, 0;
      measure_position_noise_vec << 0, 0, 0;
      ground_truth_location += radar_velocity * 0.1;
      std::shared_ptr<Object> radar_object(new Object());
      radar_object->center = ground_truth_location + measure_position_noise_vec;
      AINFO << "radar object center is " << radar_object->center(0);
      radar_object->anchor_point = radar_object->center;
      radar_object->velocity = radar_velocity;
      std::shared_ptr<PbfSensorObject> pbf_radar_object(new PbfSensorObject(
          radar_object, SensorType::RADAR, mutable_radar_timestamp));
      pbf_radar_object->timestamp = mutable_radar_timestamp;
      motion_fusion_alg->setCurrentFuseTS(mutable_radar_timestamp + 0.2);
      motion_fusion_alg->UpdateWithObject(pbf_radar_object,
                                          motion_fusion_alg->getFuseTimeDiff());
      motion_fusion_alg->setLastFuseTS(mutable_radar_timestamp + 0.2);
      Eigen::Vector3d location;
      Eigen::Vector3d velocity;
      ground_truth_location += 0.2 * radar_velocity;
      motion_fusion_alg->GetState(&location, &velocity);
      AINFO << "ground truth:" << ground_truth_location(0) << " "
            << radar_velocity(0);
      AINFO << "filtered value:" << location(0) << " " << velocity(0);
    }
  }
}

TEST_F(PbfMotionFusionTest, test_update_without_measurement) {
  std::shared_ptr<Object> lidar_object(new Object());
  double lidar_timestamp = 1234567891.012;
  Eigen::Vector3d lidar_position(30, 0, 0);
  Eigen::Vector3d lidar_velocity(10, 0, 0);
  lidar_object->center = lidar_position;
  lidar_object->anchor_point = lidar_position;
  lidar_object->velocity = lidar_velocity;
  std::shared_ptr<PbfSensorObject> pbf_lidar_object(new PbfSensorObject(
      lidar_object, SensorType::VELODYNE_64, lidar_timestamp));

  double time_diff = 0.1;
  Eigen::Vector3d expected_position =
      lidar_position + lidar_velocity * time_diff;
  for (auto motion_fusion_alg : motion_fusion_algs_) {
    motion_fusion_alg->Initialize(pbf_lidar_object);
    motion_fusion_alg->UpdateWithoutObject(time_diff);
    Eigen::Vector3d location;
    Eigen::Vector3d velocity;
    motion_fusion_alg->GetState(&location, &velocity);
    EXPECT_TRUE((location - expected_position).norm() < 1.0e-6);
    EXPECT_TRUE((velocity - lidar_velocity).norm() < 1e-3);
  }
}

}  // namespace perception
}  // namespace apollo
