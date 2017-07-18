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

namespace apollo {
namespace prediction {

using apollo::perception::PerceptionObstacle;
using apollo::perception::Point;

std::mutex PoseContainer::g_mutex_;
int PoseContainer::id_ = -1;
PerceptionObstacle::Type PoseContainer::type_ = PerceptionObstacle::VEHICLE;

void PoseContainer::Insert(const ::google::protobuf::Message& message) {}

void PoseContainer::Update(
    const localization::LocalizationEstimate &localization) {
  if (obstacle_ptr_.get() == nullptr) {
    obstacle_ptr_.reset(new PerceptionObstacle());
  }
  obstacle_ptr_->set_id(id_);
  Point position;
  position.set_x(localization.pose().position().x());
  position.set_y(localization.pose().position().y());
  position.set_z(localization.pose().position().z());
  obstacle_ptr_->mutable_position()->CopyFrom(position);

  Point velocity;
  position.set_x(localization.pose().linear_velocity().x());
  position.set_y(localization.pose().linear_velocity().y());
  position.set_z(localization.pose().linear_velocity().z());
  obstacle_ptr_->mutable_position()->CopyFrom(position);

  obstacle_ptr_->set_type(type_);
  obstacle_ptr_->set_timestamp(localization.measurement_time());
}

PerceptionObstacle* PoseContainer::ToPerceptionObstacle() {
  std::lock_guard<std::mutex> lock(g_mutex_);
  return obstacle_ptr_.get();
}

}  // namespace prediction
}  // namespace apollo
