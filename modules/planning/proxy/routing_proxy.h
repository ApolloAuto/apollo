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

/**
 * @file
 **/

#ifndef MODULES_PLANNING_PROXY_ROUTING_PROXY_H_
#define MODULES_PLANNING_PROXY_ROUTING_PROXY_H_

#include <string>

#include "modules/common/status/status.h"
#include "modules/map/proto/routing.pb.h"

namespace apollo {
namespace planning {

class RoutingProxy {
 public:
  RoutingProxy() = default;
  apollo::common::Status Init();
  void set_routing(const ::apollo::hdmap::RoutingResult &routing);
  const ::apollo::hdmap::RoutingResult &routing() const;

 private:
  ::apollo::hdmap::RoutingResult routing_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PROXY_ROUTING_PROXY_H_
