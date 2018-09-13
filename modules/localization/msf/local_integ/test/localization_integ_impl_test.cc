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

#include "modules/localization/msf/local_integ/localization_integ_impl.h"
#include "modules/common/time/time.h"
#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace msf {

class LocalizationIntegImplTestSuite : public ::testing::Test {
 public:
  LocalizationIntegImplTestSuite() {
    localization_integ_.reset(new LocalizationIntegImpl());
  }
  virtual ~LocalizationIntegImplTestSuite() {}

  virtual void SetUp() {}

  virtual void TearDown() {}

 protected:
  std::unique_ptr<LocalizationIntegImpl> localization_integ_;
};

TEST_F(LocalizationIntegImplTestSuite, CheckImuDelayStatus) {
  double cur_imu_time = apollo::common::time::Clock::NowInSeconds();
  usleep(10000);  // delay 10ms
  localization_integ_->CheckImuDelayStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_delay_status(),
            apollo::localization::IMU_DELAY_NORMAL);
  usleep(20000);  // delay 30ms
  localization_integ_->CheckImuDelayStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_delay_status(),
            apollo::localization::IMU_DELAY_1);
  usleep(20000);  // delay 50ms
  localization_integ_->CheckImuDelayStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_delay_status(),
            apollo::localization::IMU_DELAY_2);
  usleep(50000);  // delay 100ms
  localization_integ_->CheckImuDelayStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_delay_status(),
            apollo::localization::IMU_DELAY_3);
}

TEST_F(LocalizationIntegImplTestSuite, CheckImuMissingStatus) {
  double cur_imu_time = apollo::common::time::Clock::NowInSeconds();
  localization_integ_->CheckImuMissingStatus(cur_imu_time);
  cur_imu_time += 0.02;
  localization_integ_->CheckImuMissingStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_missing_status(),
            apollo::localization::IMU_MISSING_1);

  cur_imu_time += 0.06;
  localization_integ_->CheckImuMissingStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_missing_status(),
            apollo::localization::IMU_MISSING_2);

  cur_imu_time += 0.12;
  localization_integ_->CheckImuMissingStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_missing_status(),
            apollo::localization::IMU_MISSING_3);

  cur_imu_time -= 0.1;
  localization_integ_->CheckImuMissingStatus(cur_imu_time);
  EXPECT_EQ(localization_integ_->sensor_status_.imu_missing_status(),
            apollo::localization::IMU_MISSING_ABNORMAL);
}

TEST_F(LocalizationIntegImplTestSuite, SetLocalizationStatus) {
  LocalizationEstimate local_estimate;
  apollo::localization::Uncertainty *uncertainty =
    local_estimate.mutable_uncertainty();
  apollo::common::Point3D *pos_std =
    uncertainty->mutable_position_std_dev();
  pos_std->set_x(0.1);
  pos_std->set_y(0.1);
  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROW_INT);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_LIDAR_GNSS);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROW_INT);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_X_GNSS);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROWLANE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_LIDAR_X);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROWLANE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_X_X);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::SINGLE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_LIDAR_XX);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::SINGLE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_X_XX);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NONE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_LIDAR_XXX);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NONE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SOL_X_XXX);

  pos_std->set_x(0.2);
  pos_std->set_y(0.2);
  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROW_INT);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_LIDAR_GNSS);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROW_INT);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_X_GNSS);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROWLANE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_LIDAR_X);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NARROWLANE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_X_X);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::SINGLE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_LIDAR_XX);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::SINGLE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_X_XX);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NONE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NORMAL);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_LIDAR_XXX);

  localization_integ_->msf_status_.set_gnsspos_position_type(
    apollo::localization::NONE);
  localization_integ_->msf_status_.set_local_lidar_status(
    apollo::localization::MSF_LOCAL_LIDAR_NOT_GOOD);
  localization_integ_->SetLocalizationStatus(&local_estimate);
  EXPECT_EQ(local_estimate.msf_status().msf_running_status(),
            apollo::localization::MSF_SSOL_X_XXX);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
