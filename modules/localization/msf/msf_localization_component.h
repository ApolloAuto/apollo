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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"
#include "cybertron/cybertron.h"
#include "cybertron/message/raw_message.h"
#include "cybertron/tf2_cybertron/buffer.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"

#include "modules/localization/msf/msf_localization.h"

#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/msf_config.pb.h"

namespace apollo {
namespace localization {

class LocalizationMsgPublisher;

class MSFLocalizationComponent final
    : public cybertron::Component<drivers::gnss::Imu> {
 public:
  MSFLocalizationComponent();
  ~MSFLocalizationComponent() = default;

  bool Init() override;

  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& imu_msg) override;

 private:
  bool InitConfig();
  bool InitIO();

 private:
  //   std::shared_ptr<cybertron::Reader<drivers::gnss::Imu>>
  //       imu_listener_ = nullptr;
  //   std::string imu_topic_ = "";

  std::shared_ptr<cybertron::Reader<drivers::PointCloud>> lidar_listener_ =
      nullptr;
  std::string lidar_topic_ = "";

  std::shared_ptr<cybertron::Reader<drivers::gnss::GnssBestPose>>
      bestgnsspos_listener_ = nullptr;
  std::string bestgnsspos_topic_ = "";

 private:
  std::shared_ptr<LocalizationMsgPublisher> publisher_;
  MSFLocalization localization_;
};

CYBERTRON_REGISTER_COMPONENT(MSFLocalizationComponent);

class LocalizationMsgPublisher {
 public:
  explicit LocalizationMsgPublisher(
      const std::shared_ptr<cybertron::Node>& node);
  ~LocalizationMsgPublisher() = default;

  bool InitConfig(const msf_config::Config& config);
  bool InitIO();

  void PublishPoseBroadcastTF(const LocalizationEstimate& localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate& localization);

  //   void PublishLocalizationMsfGnss();
 private:
  std::shared_ptr<cybertron::Node> node_;

  std::string localization_topic_ = "";
  std::shared_ptr<cybertron::Writer<LocalizationEstimate>>
      localization_talker_ = nullptr;

  std::string broadcast_tf_frame_id_ = "";
  std::string broadcast_tf_child_frame_id_ = "";
  cybertron::tf2_cybertron::TransformBroadcaster tf2_broadcaster_;
};

}  // namespace localization
}  // namespace apollo
