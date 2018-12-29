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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "modules/transform/proto/static_transform_conf.pb.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace transform {

class StaticTransformComponent final : public apollo::cyber::Component<> {
 public:
  StaticTransformComponent() = default;
  ~StaticTransformComponent() = default;

 public:
  bool Init() override;

 private:
  void SendTransforms();
  void SendTransform(const std::vector<TransformStamped>& msgtf);
  bool ParseFromYaml(const std::string& file_path, TransformStamped* transform);

  apollo::static_transform::Conf conf_;
  std::shared_ptr<cyber::Writer<TransformStampeds>> writer_;
  TransformStampeds transform_stampeds_;
};

CYBER_REGISTER_COMPONENT(StaticTransformComponent)

}  // namespace transform
}  // namespace apollo
