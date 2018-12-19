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

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace ndt {

using apollo::common::time::Clock;

NDTLocalizationComponent::NDTLocalizationComponent()
    : localization_(new NDTLocalization()) {}

bool NDTLocalizationComponent::Init() {
  Clock::SetMode(Clock::CYBER);
  tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
  if (InitConfig() != true) {
    AERROR << "Init Config false.";
    return false;
  }

  if (InitIO() != true) {
    AERROR << "Init Interval false.";
    return false;
  }

  return true;
}

bool NDTLocalizationComponent::InitConfig() {
  localization_topic_ = FLAGS_localization_topic;
  lidar_topic_ = FLAGS_lidar_topic;
  lidar_pose_topic_ = FLAGS_localization_lidar_topic;
  broadcast_tf_frame_id_ = FLAGS_broadcast_tf_frame_id;
  broadcast_tf_child_frame_id_ = FLAGS_broadcast_tf_child_frame_id;

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

  localization_talker_ =
      this->node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  lidar_pose_talker_ =
      this->node_->CreateWriter<LocalizationEstimate>(lidar_pose_topic_);

  return true;
}

bool NDTLocalizationComponent::Proc(
    const std::shared_ptr<localization::Gps>& gps_msg) {
  localization_->OdometryCallback(gps_msg);

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

void NDTLocalizationComponent::LidarCallback(
    const std::shared_ptr<drivers::PointCloud>& lidar_msg) {
  localization_->LidarCallback(lidar_msg);
  // for test to output lidar pose
  if (localization_->IsServiceStarted()) {
    LocalizationEstimate localization;
    localization_->GetLocalization(&localization);
    Eigen::Affine3d lidar_pose = localization_->GetLidarPose();
    auto mutable_pose = localization.mutable_pose();
    mutable_pose->mutable_position()->set_x(lidar_pose.translation().x());
    mutable_pose->mutable_position()->set_y(lidar_pose.translation().y());
    mutable_pose->mutable_position()->set_z(lidar_pose.translation().z());

    Eigen::Quaterniond quat(lidar_pose.linear());
    mutable_pose->mutable_orientation()->set_qw(quat.w());
    mutable_pose->mutable_orientation()->set_qx(quat.x());
    mutable_pose->mutable_orientation()->set_qy(quat.y());
    mutable_pose->mutable_orientation()->set_qz(quat.z());
    double heading = common::math::QuaternionToHeading(quat.w(), quat.x(),
                                                       quat.y(), quat.z());
    mutable_pose->set_heading(heading);

    common::math::EulerAnglesZXYd euler(quat.w(), quat.x(), quat.y(), quat.z());
    mutable_pose->mutable_euler_angles()->set_x(euler.pitch());
    mutable_pose->mutable_euler_angles()->set_y(euler.roll());
    mutable_pose->mutable_euler_angles()->set_z(euler.yaw());
    // publish localization messages
    PublishLidarPoseBroadcastTopic(localization);
  }
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
  return;
}

void NDTLocalizationComponent::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
  return;
}

void NDTLocalizationComponent::PublishLidarPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
  lidar_pose_talker_->Write(localization);
  return;
}

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
