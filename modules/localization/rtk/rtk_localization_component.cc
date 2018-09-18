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
  Clock::SetMode(Clock::CYBERTRON);

  if (InitConfig() != true) {
    AERROR << "Init Config falseed.";
    return false;
  }

  if (InitIO() != true) {
    AERROR << "Init Interval falseed.";
    return false;
  }

  return true;
}

bool RTKLocalizationComponent::InitConfig() {
  rtk_config::Config rtk_config;
  if (!apollo::cybertron::common::GetProtoFromFile(config_file_path_,
                                                   &rtk_config)) {
    return false;
  }
  AINFO << "Rtk localization config: " << rtk_config.DebugString();

  localization_topic_ = rtk_config.localization_topic();
  imu_topic_ = rtk_config.imu_topic();
  gps_topic_ = rtk_config.gps_topic();
  broadcast_tf_frame_id_ = rtk_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = rtk_config.broadcast_tf_child_frame_id();

  localization_->InitConfig(rtk_config);

  return true;
}

bool RTKLocalizationComponent::InitIO() {
  std::function<void(const std::shared_ptr<localization::CorrectedImu>&)>
      imu_register_call = std::bind(&RTKLocalizationComponent::ImuCallback,
                                    this, std::placeholders::_1);
  corrected_imu_listener_ =
      this->node_->CreateReader<localization::CorrectedImu>(imu_topic_,
                                                            imu_register_call);

  localization_talker_ =
      this->node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  return true;
}

bool RTKLocalizationComponent::Proc(
    const std::shared_ptr<localization::Gps>& gps_msg) {
  localization_->GpsCallback(gps_msg);

  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);

    // publish localization messages
    PublishPoseBroadcastTopic(localization);
    PublishPoseBroadcastTF(localization);
    ADEBUG << "[OnTimer]: Localization message publish success!";
  }

  return true;
}

void RTKLocalizationComponent::ImuCallback(
    const std::shared_ptr<localization::CorrectedImu>& imu_msg) {
  localization_->ImuCallback(imu_msg);
}

void RTKLocalizationComponent::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
  // broadcast tf message
  adu::common::TransformStamped tf2_msg;

  auto mutable_head = tf2_msg.mutable_header();
  mutable_head->set_stamp(
      cybertron::Time(localization.measurement_time()).ToNanosecond());
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

void RTKLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
  return;
}

}  // namespace localization
}  // namespace apollo
