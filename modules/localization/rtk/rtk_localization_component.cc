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

#include "modules/localization/rtk/rtk_localization_component.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace localization {

using apollo::common::time::Clock;

RTKLocalizationComponent::RTKLocalizationComponent()
    : localization_(new RTKLocalization()) {}

bool RTKLocalizationComponent::Init() {
  tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
  if (!InitConfig()) {
    AERROR << "Init Config falseed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "Init Interval falseed.";
    return false;
  }

  return true;
}

bool RTKLocalizationComponent::InitConfig() {
  rtk_config::Config rtk_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &rtk_config)) {
    return false;
  }
  AINFO << "Rtk localization config: " << rtk_config.DebugString();

  localization_topic_ = rtk_config.localization_topic();
  localization_status_topic_ = rtk_config.localization_status_topic();
  imu_topic_ = rtk_config.imu_topic();
  gps_topic_ = rtk_config.gps_topic();
  gps_status_topic_ = rtk_config.gps_status_topic();
  broadcast_tf_frame_id_ = rtk_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = rtk_config.broadcast_tf_child_frame_id();

  localization_->InitConfig(rtk_config);

  return true;
}

bool RTKLocalizationComponent::InitIO() {
  corrected_imu_listener_ = node_->CreateReader<localization::CorrectedImu>(
      imu_topic_, std::bind(&RTKLocalization::ImuCallback, localization_.get(),
                            std::placeholders::_1));
  CHECK(corrected_imu_listener_);

  gps_status_listener_ = node_->CreateReader<drivers::gnss::InsStat>(
      gps_status_topic_, std::bind(&RTKLocalization::GpsStatusCallback,
                                   localization_.get(), std::placeholders::_1));
  CHECK(gps_status_listener_);

  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);
  CHECK(localization_talker_);

  localization_status_talker_ =
      node_->CreateWriter<LocalizationStatus>(localization_status_topic_);
  CHECK(localization_status_talker_);
  return true;
}

bool RTKLocalizationComponent::Proc(
    const std::shared_ptr<localization::Gps>& gps_msg) {
  localization_->GpsCallback(gps_msg);

  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);
    LocalizationStatus localization_status;
    localization_->GetLocalizationStatus(&localization_status);

    // publish localization messages
    PublishPoseBroadcastTopic(localization);
    PublishPoseBroadcastTF(localization);
    PublishLocalizationStatus(localization_status);
    ADEBUG << "[OnTimer]: Localization message publish success!";
  }

  return true;
}

void RTKLocalizationComponent::PublishPoseBroadcastTF(
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

void RTKLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}

void RTKLocalizationComponent::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

}  // namespace localization
}  // namespace apollo
