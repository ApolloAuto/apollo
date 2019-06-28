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

#include "modules/prediction/container/pose/pose_container.h"

#include "cyber/common/log.h"
#include "modules/common/math/quaternion.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using Point = apollo::common::Point3D;

void PoseContainer::Insert(const ::google::protobuf::Message& message) {
  localization::LocalizationEstimate localization;
  localization.CopyFrom(dynamic_cast<const LocalizationEstimate&>(message));
  Update(localization);
}

void PoseContainer::Update(
    const localization::LocalizationEstimate& localization) {
  if (!localization.has_header() ||
      !localization.header().has_timestamp_sec()) {
    AERROR << "Localization message has no timestamp ["
           << localization.ShortDebugString() << "].";
    return;
  } else if (!localization.has_pose()) {
    AERROR << "Localization message has no pose ["
           << localization.ShortDebugString() << "].";
    return;
  } else if (!localization.pose().has_position() ||
             !localization.pose().has_linear_velocity()) {
    AERROR << "Localization message has no position or linear velocity ["
           << localization.ShortDebugString() << "].";
    return;
  }

  if (obstacle_ptr_.get() == nullptr) {
    obstacle_ptr_.reset(new PerceptionObstacle());
  }
  obstacle_ptr_->Clear();

  obstacle_ptr_->set_id(FLAGS_ego_vehicle_id);
  Point position;
  position.set_x(localization.pose().position().x());
  position.set_y(localization.pose().position().y());
  position.set_z(localization.pose().position().z());
  obstacle_ptr_->mutable_position()->CopyFrom(position);

  double theta = 0.0;
  if (localization.pose().has_orientation() &&
      localization.pose().orientation().has_qx() &&
      localization.pose().orientation().has_qy() &&
      localization.pose().orientation().has_qz() &&
      localization.pose().orientation().has_qw()) {
    double qw = localization.pose().orientation().qw();
    double qx = localization.pose().orientation().qx();
    double qy = localization.pose().orientation().qy();
    double qz = localization.pose().orientation().qz();
    theta = common::math::QuaternionToHeading(qw, qx, qy, qz);
  }
  obstacle_ptr_->set_theta(theta);

  Point velocity;
  velocity.set_x(localization.pose().linear_velocity().x());
  velocity.set_y(localization.pose().linear_velocity().y());
  velocity.set_z(localization.pose().linear_velocity().z());
  obstacle_ptr_->mutable_velocity()->CopyFrom(velocity);

  obstacle_ptr_->set_type(type_);
  obstacle_ptr_->set_timestamp(localization.header().timestamp_sec());

  ADEBUG << "ADC obstacle [" << obstacle_ptr_->ShortDebugString() << "].";
}

double PoseContainer::GetTimestamp() {
  if (obstacle_ptr_ != nullptr) {
    return obstacle_ptr_->timestamp();
  } else {
    return 0.0;
  }
}

const PerceptionObstacle* PoseContainer::ToPerceptionObstacle() {
  return obstacle_ptr_.get();
}

}  // namespace prediction
}  // namespace apollo
