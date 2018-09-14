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

#include <mutex>

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"
#include "cybertron/cybertron.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/tf2_cybertron/buffer.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"

namespace apollo {
namespace localization {

class RTKLocalizationComponent final
    : public cybertron::Component<localization::Gps> {
 public:
  RTKLocalizationComponent();
  ~RTKLocalizationComponent() = default;

  bool Init() override;

  bool Proc(const std::shared_ptr<localization::Gps>& gps_msg) override;

  void ImuCallback(const std::shared_ptr<localization::CorrectedImu>& imu_msg);

 private:
  bool InitConfig();
  bool InitIO();

  void PublishLocalization(const localization::Gps& gps_msg);
  void RunWatchDog(double gps_timestamp);

  void PrepareLocalizationMsg(const localization::Gps& gps_msg,
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

  void PublishPoseBroadcastTF(const LocalizationEstimate& localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate& localization);

 private:
  std::shared_ptr<cybertron::Reader<localization::CorrectedImu>>
      corrected_imu_listener_ = nullptr;

  std::shared_ptr<cybertron::Writer<LocalizationEstimate>>
      localization_talker_ = nullptr;

  std::string module_name_ = "localization";
  long long localization_seq_num_ = 0;

  std::string localization_topic_ = "";
  std::string gps_topic_ = "";
  std::string imu_topic_ = "";

  std::string broadcast_tf_frame_id_ = "";
  std::string broadcast_tf_child_frame_id_ = "";
  cybertron::tf2_cybertron::TransformBroadcaster tf2_broadcaster_;

  bool enable_gps_timestamp_ = false;
  double gps_time_delay_tolerance_ = 1.0;

  double gps_imu_time_diff_threshold_ = 0.02;

  double last_received_timestamp_sec_ = 0.0;
  double last_reported_timestamp_sec_ = 0.0;

  bool enable_watch_dog_ = true;
  bool service_started_ = false;

  int localization_publish_freq_ = 100;
  int report_threshold_err_num_ = 10;

  std::list<localization::CorrectedImu> imu_list_;
  size_t imu_list_max_size_ = 20;
  std::mutex imu_list_mutex_;

  std::vector<double> map_offset_;

  apollo::common::monitor::MonitorLogger monitor_logger_;
};

CYBERTRON_REGISTER_COMPONENT(RTKLocalizationComponent);

}  // namespace localization
}  // namespace apollo