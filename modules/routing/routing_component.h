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

#ifndef MODULES_ROUTING_ROUTING_COMPONENT_H_
#define MODULES_ROUTING_ROUTING_COMPONENT_H_

#include <memory>

#include "cybertron/class_loader/class_loader.h"
#include "cybertron/component/component.h"

#include "modules/routing/routing.h"

namespace apollo {
namespace routing {

class RoutingComponent final
    : public ::apollo::cybertron::Component<RoutingRequest> {
 public:
  RoutingComponent() = default;
  ~RoutingComponent() = default;
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<RoutingRequest>& request) override;
 private:
  std::shared_ptr<::apollo::cybertron::Writer<RoutingResponse>>
      response_writer_ = nullptr;
  Routing routing_;
};

CYBERTRON_REGISTER_COMPONENT(RoutingComponent)

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_ROUTING_COMPONENT_H_

