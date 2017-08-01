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

#ifndef BAIDU_ADU_ROUTING_CORE_ARBITER_H
#define BAIDU_ADU_ROUTING_CORE_ARBITER_H

#include <memory>
#include "ros/ros.h"

#include "common/routing_gflags.h"
#include "common/routing_macros.h"
#include "arbiter/routing_signal.h"

namespace adu {
namespace routing {

class Navigator;

class Arbiter {
public:
    ~Arbiter();
    bool run();

private:
    bool on_request(arbiter::routing_signal::Request& req,
                    arbiter::routing_signal::Response& res);

    // for old routing request
    bool on_request_old_routing(arbiter::routing_signal::Request& req,
                                arbiter::routing_signal::Response& res);
private:
    std::unique_ptr<ros::NodeHandle> _node_handle_ptr;
    ros::ServiceServer _service;
    ros::Publisher _publisher;
    std::unique_ptr<Navigator> _navigator_ptr;
    DECLARE_ARBITER_SINGLETON(Arbiter);
};

} // namespace routing
} // namespace adu

#endif // BAIDU_ADU_ROUTING_CORE_ARBITER_H

