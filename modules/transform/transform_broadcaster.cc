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

#include "modules/transform/transform_broadcaster.h"

namespace apollo {
namespace transform {

TransformBroadcaster::TransformBroadcaster(
    const std::shared_ptr<cybertron::Node>& node)
    : node_(node) {
  cybertron::proto::RoleAttributes attr;
  attr.set_channel_name("/tf");
  writer_ = node_->CreateWriter<::apollo::transform::TransformStampeds>(attr);
}

void TransformBroadcaster::sendTransform(
    const ::apollo::transform::TransformStamped& transform) {
  std::vector<::apollo::transform::TransformStamped> transforms;
  transforms.emplace_back(transform);
  sendTransform(transforms);
}

void TransformBroadcaster::sendTransform(
    const std::vector<::apollo::transform::TransformStamped>& transforms) {
  auto message = std::make_shared<::apollo::transform::TransformStampeds>();
  for (const auto& transform : transforms) {
    *(message->add_transforms()) = transform;
  }
  writer_->Write(message);
}

}  // namespace transform
}  // namespace apollo
