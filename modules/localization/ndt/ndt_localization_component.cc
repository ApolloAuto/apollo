/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/ndt/ndt_localization_component.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace ndt {

using apollo::cyber::Clock;

NDTLocalizationComponent::NDTLocalizationComponent()
    : localization_(new NDTLocalization()) {}

bool NDTLocalizationComponent::Init() {
  tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
  if (!InitConfig()) {
    AERROR << "Init Config false.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "Init Interval false.";
    return false;
  }

  return true;
}

bool NDTLocalizationComponent::InitConfig() {
  localization_topic_ = FLAGS_localization_topic;
  lidar_topic_ = FLAGS_lidar_topic;
  lidar_pose_topic_ = FLAGS_localization_ndt_topic;
  broadcast_tf_frame_id_ = FLAGS_broadcast_tf_frame_id;
  broadcast_tf_child_frame_id_ = FLAGS_broadcast_tf_child_frame_id;
  odometry_status_topic_ = FLAGS_ins_stat_topic;
  localization_status_topic_ = FLAGS_localization_msf_status;

  localization_->Init();

  return true;
}

bool NDTLocalizationComponent::InitIO() {
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = lidar_topic_;
  reader_config.pending_queue_size = 1;

  std::function<void(const std::shared_ptr<drivers::PointCloud>&)>
      lidar_register_call = std::bind(&NDTLocalizationComponent::LidarCallback,
                                      this, std::placeholders::_1);
  lidar_listener_ = this->node_->CreateReader<drivers::PointCloud>(
      reader_config, lidar_register_call);

  reader_config.channel_name = odometry_status_topic_;
  reader_config.pending_queue_size = 1;
  std::function<void(const std::shared_ptr<drivers::gnss::InsStat>&)>
      odometry_status_call =
          std::bind(&NDTLocalizationComponent::OdometryStatusCallback, this,
                    std::placeholders::_1);
  odometry_status_listener_ = this->node_->CreateReader<drivers::gnss::InsStat>(
      reader_config, odometry_status_call);

  localization_talker_ =
      this->node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  lidar_pose_talker_ =
      this->node_->CreateWriter<LocalizationEstimate>(lidar_pose_topic_);

  localization_status_talker_ =
      this->node_->CreateWriter<LocalizationStatus>(localization_status_topic_);

  return true;
}

bool NDTLocalizationComponent::Proc(
    const std::shared_ptr<localization::Gps>& gps_msg) {
  localization_->OdometryCallback(gps_msg);

  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);

    LocalizationStatus localization_status;
    localization_->GetLocalizationStatus(&localization_status);

    // publish localization messages
    PublishPoseBroadcastTopic(localization);
    PublishPoseBroadcastTF(localization);
    PublishLocalizationStatusTopic(localization_status);
    ADEBUG << "[OnTimer]: Localization message publish success!";
  }

  return true;
}

void NDTLocalizationComponent::LidarCallback(
    const std::shared_ptr<drivers::PointCloud>& lidar_msg) {
  localization_->LidarCallback(lidar_msg);
  // for test to output lidar pose
  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLidarLocalization(&localization);
    // publish localization messages
    PublishLidarPoseBroadcastTopic(localization);
  }
}

void NDTLocalizationComponent::OdometryStatusCallback(
    const std::shared_ptr<drivers::gnss::InsStat>& status_msg) {
  localization_->OdometryStatusCallback(status_msg);
}

void NDTLocalizationComponent::PublishPoseBroadcastTF(
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

  tf2_broadcaster_->SendTransform(tf2_msg);
}

void NDTLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}

void NDTLocalizationComponent::PublishLidarPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  lidar_pose_talker_->Write(localization);
}

void NDTLocalizationComponent::PublishLocalizationStatusTopic(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
