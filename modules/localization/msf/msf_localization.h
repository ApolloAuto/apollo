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

/**
 * @file msf_localization.h
 * @brief The class of MSFLocalization
 */

#ifndef MODULES_LOCALIZATION_MSF_LOCALIZATION_H_
#define MODULES_LOCALIZATION_MSF_LOCALIZATION_H_

#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"
#include "ros/include/ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/measure.pb.h"
#include "modules/localization/proto/sins_pva.pb.h"

#include "include/localization_integ.h"
#include "modules/common/log.h"
#include "modules/common/monitor/monitor.h"
#include "modules/common/status/status.h"
#include "modules/localization/localization_base.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class MSFLocalization
 *
 * @brief generate localization info based on MSF
 */
class MSFLocalization : public LocalizationBase {
 public:
  MSFLocalization();
  virtual ~MSFLocalization();

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   * @return stop status
   */
  apollo::common::Status Stop() override;

 private:
  apollo::common::Status Init();
  void InitParams();
  void OnPointCloud(const sensor_msgs::PointCloud2 &message);
  void OnRawImu(const drivers::gnss::Imu &imu_msg);
  void OnGnssRtkObs(const EpochObservation &raw_obs_msg);
  void OnGnssRtkEph(const GnssEphemeris &gnss_orbit_msg);
  void OnGnssBestPose(const GnssBestPose &bestgnsspos_msg);

  void PublishPoseBroadcastTF(const LocalizationEstimate &localization);

 private:
  apollo::common::monitor::Monitor monitor_;
  LocalizationInteg localization_integ_;
  LocalizationIntegParam localizaiton_param_;
  tf2_ros::TransformBroadcaster *tf2_broadcaster_;
  LocalizationMeasureState localization_state_;
  uint64_t pcd_msg_index_;

  // FRIEND_TEST(MSFLocalizationTest, InitParams);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCALIZATION_H_
