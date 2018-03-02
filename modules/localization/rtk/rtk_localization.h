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
 * @file rtk_localization.h
 * @brief The class of RTKLocalization
 */

#ifndef MODULES_LOCALIZATION_RTK_RTK_LOCALIZATION_H_
#define MODULES_LOCALIZATION_RTK_RTK_LOCALIZATION_H_

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "glog/logging.h"
#include "gtest/gtest_prod.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/localization/localization_base.h"

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {

/**
 * @class RTKLocalization
 *
 * @brief generate localization info based on RTK
 */
class RTKLocalization : public LocalizationBase {
 public:
  RTKLocalization();
  virtual ~RTKLocalization();

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
  void OnTimer(const ros::TimerEvent &event);
  void PublishLocalization();
  void RunWatchDog();

  void PrepareLocalizationMsg(LocalizationEstimate *localization);
  void ComposeLocalizationMsg(const localization::Gps &gps,
                              const localization::Imu &imu,
                              LocalizationEstimate *localization);
  bool FindMatchingIMU(const double gps_timestamp_sec, Imu *imu_msg);
  bool InterpolateIMU(const Imu &imu1, const Imu &imu2,
                      const double timestamp_sec, Imu *msgbuf);
  template <class T>
  T InterpolateXYZ(const T &p1, const T &p2, const double &frac1);

 private:
  ros::Timer timer_;
  apollo::common::monitor::MonitorLogger monitor_logger_;
  const std::vector<double> map_offset_;
  double last_received_timestamp_sec_ = 0.0;
  double last_reported_timestamp_sec_ = 0.0;
  bool service_started_ = false;

  FRIEND_TEST(RTKLocalizationTest, InterpolateIMU);
  FRIEND_TEST(RTKLocalizationTest, ComposeLocalizationMsg);
};

}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_RTK_RTK_LOCALIZATION_H_
