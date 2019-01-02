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

#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"

#include "modules/localization/msf/msf_localization.h"

#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/transform/transform_broadcaster.h"

namespace apollo {
namespace localization {

class MSFLocalizationComponent final
    : public cyber::Component<drivers::gnss::Imu> {
 public:
  MSFLocalizationComponent();
  ~MSFLocalizationComponent() = default;

  bool Init() override;

  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& imu_msg) override;

 private:
  bool InitConfig();
  bool InitIO();

 private:
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_listener_ = nullptr;
  std::string lidar_topic_ = "";

  std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>>
      bestgnsspos_listener_ = nullptr;
  std::string bestgnsspos_topic_ = "";

  std::shared_ptr<cyber::Reader<drivers::gnss::Heading>>
      gnss_heading_listener_ = nullptr;
  std::string gnss_heading_topic_ = "";

 private:
  std::shared_ptr<LocalizationMsgPublisher> publisher_;
  MSFLocalization localization_;
};

CYBER_REGISTER_COMPONENT(MSFLocalizationComponent);

class LocalizationMsgPublisher {
 public:
  explicit LocalizationMsgPublisher(const std::shared_ptr<cyber::Node>& node);
  ~LocalizationMsgPublisher() = default;

  bool InitConfig();
  bool InitIO();

  void PublishPoseBroadcastTF(const LocalizationEstimate& localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate& localization);

  void PublishLocalizationMsfGnss(const LocalizationEstimate& localization);
  void PublishLocalizationMsfLidar(const LocalizationEstimate& localization);
  void PublishLocalizationStatus(const LocalizationStatus& localization_status);

 private:
  std::shared_ptr<cyber::Node> node_;

  std::string localization_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> localization_talker_ =
      nullptr;

  std::string broadcast_tf_frame_id_ = "";
  std::string broadcast_tf_child_frame_id_ = "";
  apollo::transform::TransformBroadcaster tf2_broadcaster_;

  std::string lidar_local_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> lidar_local_talker_ =
      nullptr;

  std::string gnss_local_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> gnss_local_talker_ =
      nullptr;

  std::string localization_status_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationStatus>>
      localization_status_talker_ = nullptr;
};

}  // namespace localization
}  // namespace apollo
