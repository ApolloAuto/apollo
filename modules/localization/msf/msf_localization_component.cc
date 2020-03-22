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

#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::common::time::Clock;

MSFLocalizationComponent::MSFLocalizationComponent() {}

bool MSFLocalizationComponent::Init() {
  publisher_.reset(new LocalizationMsgPublisher(this->node_));

  if (!InitConfig()) {
    AERROR << "Init Config failed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "Init IO failed.";
    return false;
  }

  return true;
}

bool MSFLocalizationComponent::InitConfig() {
  lidar_topic_ = FLAGS_lidar_topic;
  bestgnsspos_topic_ = FLAGS_gnss_best_pose_topic;
  gnss_heading_topic_ = FLAGS_heading_topic;

  if (!publisher_->InitConfig()) {
    AERROR << "Init publisher config failed.";
    return false;
  }

  if (!localization_.Init().ok()) {
    AERROR << "Init class MSFLocalization failed.";
    return false;
  }

  return true;
}

bool MSFLocalizationComponent::InitIO() {
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = lidar_topic_;
  reader_config.pending_queue_size = 1;

  std::function<void(const std::shared_ptr<drivers::PointCloud>&)>
      lidar_register_call = std::bind(&MSFLocalization::OnPointCloud,
                                      &localization_, std::placeholders::_1);

  lidar_listener_ = this->node_->CreateReader<drivers::PointCloud>(
      reader_config, lidar_register_call);

  std::function<void(const std::shared_ptr<drivers::gnss::GnssBestPose>&)>
      bestgnsspos_register_call =
          std::bind(&MSFLocalization::OnGnssBestPose, &localization_,
                    std::placeholders::_1);
  bestgnsspos_listener_ =
      this->node_->CreateReader<drivers::gnss::GnssBestPose>(
          bestgnsspos_topic_, bestgnsspos_register_call);

  std::function<void(const std::shared_ptr<drivers::gnss::Heading>&)>
      gnss_heading_call = std::bind(&MSFLocalization::OnGnssHeading,
                                    &localization_, std::placeholders::_1);
  gnss_heading_listener_ = this->node_->CreateReader<drivers::gnss::Heading>(
      gnss_heading_topic_, gnss_heading_call);

  // init writer
  if (!publisher_->InitIO()) {
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
    const std::shared_ptr<cyber::Node>& node)
    : node_(node), tf2_broadcaster_(node) {}

bool LocalizationMsgPublisher::InitConfig() {
  localization_topic_ = FLAGS_localization_topic;
  broadcast_tf_frame_id_ = FLAGS_broadcast_tf_frame_id;
  broadcast_tf_child_frame_id_ = FLAGS_broadcast_tf_child_frame_id;
  lidar_local_topic_ = FLAGS_localization_lidar_topic;
  gnss_local_topic_ = FLAGS_localization_gnss_topic;
  localization_status_topic_ = FLAGS_localization_msf_status;

  return true;
}

bool LocalizationMsgPublisher::InitIO() {
  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  lidar_local_talker_ =
      node_->CreateWriter<LocalizationEstimate>(lidar_local_topic_);

  gnss_local_talker_ =
      node_->CreateWriter<LocalizationEstimate>(gnss_local_topic_);

  localization_status_talker_ =
      node_->CreateWriter<LocalizationStatus>(localization_status_topic_);
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

  tf2_broadcaster_.SendTransform(tf2_msg);
}

void LocalizationMsgPublisher::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}

void LocalizationMsgPublisher::PublishLocalizationMsfGnss(
    const LocalizationEstimate& localization) {
  gnss_local_talker_->Write(localization);
}

void LocalizationMsgPublisher::PublishLocalizationMsfLidar(
    const LocalizationEstimate& localization) {
  lidar_local_talker_->Write(localization);
}

void LocalizationMsgPublisher::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

}  // namespace localization
}  // namespace apollo
