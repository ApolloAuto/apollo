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

#pragma once

#include <list>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest_prod.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/rtk_config.pb.h"

namespace apollo {
namespace localization {

class RTKLocalization {
 public:
  RTKLocalization();
  ~RTKLocalization() = default;

  void InitConfig(const rtk_config::Config &config);

  void GpsCallback(const std::shared_ptr<localization::Gps> &gps_msg);
  void ImuCallback(const std::shared_ptr<localization::CorrectedImu> &imu_msg);

  bool IsServiceStarted();
  void GetLocalization(LocalizationEstimate *localization);

 private:
  void RunWatchDog(double gps_timestamp);

  void PrepareLocalizationMsg(const localization::Gps &gps_msg,
                              LocalizationEstimate *localization);
  void ComposeLocalizationMsg(const localization::Gps &gps,
                              const localization::CorrectedImu &imu,
                              LocalizationEstimate *localization);
  void FillLocalizationMsgHeader(LocalizationEstimate *localization);

  bool FindMatchingIMU(const double gps_timestamp_sec, CorrectedImu *imu_msg);
  bool InterpolateIMU(const CorrectedImu &imu1, const CorrectedImu &imu2,
                      const double timestamp_sec, CorrectedImu *imu_msg);
  template <class T>
  T InterpolateXYZ(const T &p1, const T &p2, const double frac1);

 private:
  std::string module_name_ = "localization";

  std::list<localization::CorrectedImu> imu_list_;
  size_t imu_list_max_size_ = 50;
  std::mutex imu_list_mutex_;

  std::vector<double> map_offset_;

  double gps_time_delay_tolerance_ = 1.0;
  double gps_imu_time_diff_threshold_ = 0.02;

  double last_received_timestamp_sec_ = 0.0;
  double last_reported_timestamp_sec_ = 0.0;

  bool enable_watch_dog_ = true;
  bool service_started_ = false;

  int64_t localization_seq_num_ = 0;
  LocalizationEstimate last_localization_result_;

  int localization_publish_freq_ = 100;
  int report_threshold_err_num_ = 10;
  apollo::common::monitor::MonitorLogBuffer monitor_logger_;

  FRIEND_TEST(RTKLocalizationTest, InterpolateIMU);
  FRIEND_TEST(RTKLocalizationTest, ComposeLocalizationMsg);
};

}  // namespace localization
}  // namespace apollo
