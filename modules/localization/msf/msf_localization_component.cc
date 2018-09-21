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

#include "modules/localization/msf/msf_localization_component.h"

#include <limits>

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"

#include "modules/localization/proto/msf_config.pb.h"

namespace apollo {
namespace localization {

using apollo::cybertron::proto::RoleAttributes;
using apollo::common::time::Clock;

MSFLocalizationComponent::MSFLocalizationComponent() {}

bool MSFLocalizationComponent::Init() {
  Clock::SetMode(Clock::CYBERTRON);
  publisher_.reset(new LocalizationMsgPublisher(this->node_));

  if (InitConfig() != true) {
    AERROR << "Init Config failed.";
    return false;
  }

  if (InitIO() != true) {
    AERROR << "Init IO failed.";
    return false;
  }

  return true;
}

bool MSFLocalizationComponent::InitConfig() {
  msf_config::Config msf_config;
  if (!apollo::cybertron::common::GetProtoFromFile(config_file_path_,
                                                   &msf_config)) {
    return false;
  }
  AINFO << "Msf localization config: " << msf_config.DebugString();

  //   imu_topic_ = msf_config.imu_topic();
  lidar_topic_ = msf_config.lidar_topic();
  bestgnsspos_topic_ = msf_config.bestgnsspos_topic();

  if (publisher_->InitConfig(msf_config) != true) {
    AERROR << "Init publisher config failed.";
    return false;
  }

  if (localization_.Init(msf_config).ok() != true) {
    AERROR << "Init class MSFLocalization failed.";
    return false;
  }

  return true;
}

bool MSFLocalizationComponent::InitIO() {
  // init reader
  //   std::function<void(const std::shared_ptr <
  //                      cybertron::Reader<drivers::gnss::Imu>&)>
  //       imu_register_call = std::bind(&MSFLocalization::OnRawImu,
  //       &localization_,
  //                                     std::placeholders::_1);
  //   imu_listener_ =
  //       this->node_->CreateReader <
  //       cybertron::Reader<drivers::gnss::Imu>(imu_topic_, imu_register_call);

  RoleAttributes attr;
  attr.set_channel_name(lidar_topic_);
  attr.mutable_qos_profile()->set_depth(2);
  std::function<void(const std::shared_ptr<drivers::PointCloud>&)>
      lidar_register_call = std::bind(&MSFLocalization::OnPointCloud,
                                      &localization_, std::placeholders::_1);
  lidar_listener_ = this->node_->CreateReader<drivers::PointCloud>(
      attr, lidar_register_call);

  std::function<void(const std::shared_ptr<drivers::gnss::GnssBestPose>&)>
      bestgnsspos_register_call =
          std::bind(&MSFLocalization::OnGnssBestPose, &localization_,
                    std::placeholders::_1);
  bestgnsspos_listener_ =
      this->node_->CreateReader<drivers::gnss::GnssBestPose>(
          bestgnsspos_topic_, bestgnsspos_register_call);

  // init writer
  if (publisher_->InitIO() != true) {
    AERROR << "Init publisher io failed.";
    return false;
  }

  localization_.SetPublisher(publisher_);

  return true;
}

bool MSFLocalizationComponent::Proc(
    const std::shared_ptr<drivers::gnss::Imu>& imu_msg) {
  localization_.OnRawImu(imu_msg);
  return true;
}

LocalizationMsgPublisher::LocalizationMsgPublisher(
    const std::shared_ptr<cybertron::Node>& node)
    : node_(node), tf2_broadcaster_(node) {}

bool LocalizationMsgPublisher::InitConfig(const msf_config::Config& config) {
  localization_topic_ = config.localization_topic();
  broadcast_tf_frame_id_ = config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = config.broadcast_tf_child_frame_id();
  lidar_local_topic_ = config.lidar_localization_topic();
  gnss_local_topic_ = config.gnss_localization_topic();
  return true;
}

bool LocalizationMsgPublisher::InitIO() {
  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  lidar_local_talker_ =
      node_->CreateWriter<LocalizationEstimate>(lidar_local_topic_);

  gnss_local_talker_ =
      node_->CreateWriter<LocalizationEstimate>(gnss_local_topic_);
  return true;
}

void LocalizationMsgPublisher::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
  // broadcast tf message
  apollo::transform::TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_timestamp_sec(localization.measurement_time());
  mutable_head->set_frame_id(broadcast_tf_frame_id_);
  tf2_msg.set_child_frame_id(broadcast_tf_child_frame_id_);

  auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
  mutable_translation->set_x(localization.pose().position().x());
  mutable_translation->set_y(localization.pose().position().y());
  mutable_translation->set_z(localization.pose().position().z());

  auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
  mutable_rotation->set_qx(localization.pose().orientation().qx());
  mutable_rotation->set_qy(localization.pose().orientation().qy());
  mutable_rotation->set_qz(localization.pose().orientation().qz());
  mutable_rotation->set_qw(localization.pose().orientation().qw());

  tf2_broadcaster_.sendTransform(tf2_msg);
  return;
}

void LocalizationMsgPublisher::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
  return;
}

void LocalizationMsgPublisher::PublishLocalizationMsfGnss(
    const LocalizationEstimate& localization) {
  gnss_local_talker_->Write(localization);
  return;
}

void LocalizationMsgPublisher::PublishLocalizationMsfLidar(
    const LocalizationEstimate& localization) {
  lidar_local_talker_->Write(localization);
  return;
}

}  // namespace localization
}  // namespace apollo
