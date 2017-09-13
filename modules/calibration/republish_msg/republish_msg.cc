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

#include "modules/calibration/republish_msg/common/republish_msg_gflags.h"
#include "modules/calibration/republish_msg/republish_msg.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace calibration {

using apollo::common::adapter::AdapterManager;
using apollo::common::Status;
using apollo::common::ErrorCode;

std::string RepublishMsg::Name() const { return "republish_msg"; }

Status RepublishMsg::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);

  CHECK(AdapterManager::GetInsStat()) << "INS status is not initialized.";
  CHECK(AdapterManager::GetGps()) << "Gps is not initialized.";
  CHECK(AdapterManager::GetRelativeOdometry())
      << "Relative odometry is not initialized.";

  AdapterManager::AddGpsCallback(&RepublishMsg::OnGps, this);
  AdapterManager::AddInsStatCallback(&RepublishMsg::OnInsStat, this);

  is_first_gps_msg_ = true;
  position_type_ = 0;

  return Status::OK();
}

void RepublishMsg::OnInsStat(const drivers::gnss::InsStat& msg) {
  position_type_ = msg.pos_type();
}

void RepublishMsg::OnGps(const localization::Gps& msg) {
  if (msg.has_localization()) {
    const auto pose_msg = msg.localization();

    Eigen::Quaterniond rotation(
        pose_msg.orientation().qw(), pose_msg.orientation().qx(),
        pose_msg.orientation().qy(), pose_msg.orientation().qz());
    Eigen::Translation3d translation(pose_msg.position().x(),
                                     pose_msg.position().y(),
                                     pose_msg.position().z());
    Eigen::Affine3d pose = translation * rotation;

    if (is_first_gps_msg_) {
      is_first_gps_msg_ = false;
      offset_ = pose.inverse();
    }

    Eigen::Affine3d pub_pose = offset_ * pose;
    Eigen::Quaterniond pub_rot(pub_pose.rotation());
    Eigen::Translation3d pub_trans(pub_pose.translation());

    calibration::republish_msg::RelativeOdometry pub_msg;
    pub_msg.mutable_header()->set_timestamp_sec(msg.header().timestamp_sec());
    pub_msg.mutable_orientation()->set_qw(pub_rot.w());
    pub_msg.mutable_orientation()->set_qx(pub_rot.x());
    pub_msg.mutable_orientation()->set_qy(pub_rot.y());
    pub_msg.mutable_orientation()->set_qz(pub_rot.z());
    pub_msg.mutable_position()->set_x(pub_trans.x());
    pub_msg.mutable_position()->set_y(pub_trans.y());
    pub_msg.mutable_position()->set_z(pub_trans.z());

    pub_msg.set_position_type(position_type_);

    AdapterManager::PublishRelativeOdometry(pub_msg);
  }
}

Status RepublishMsg::Start() { return Status::OK(); }

void RepublishMsg::Stop() {}

}  // namespace calibration
}  // namespace apollo
