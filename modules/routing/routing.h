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

#ifndef MODULES_ROUTING_ROUTING_H_
#define MODULES_ROUTING_ROUTING_H_

#include <memory>
#include <string>

#include "modules/routing/proto/routing.pb.h"
#include "modules/routing/proto/routing_config.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/core/navigator.h"

namespace apollo {
namespace routing {

class Routing : public apollo::common::ApolloApp {
  // friend class RoutingTestBase;
 public:
  Routing();

  /**
   * @brief module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  apollo::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief destructor
   */
  virtual ~Routing() = default;

 private:
  void OnRoutingRequest(const RoutingRequest &routing_request);

  RoutingRequest FillLaneInfoIfMissing(const RoutingRequest &routing_request);

 private:
  std::unique_ptr<Navigator> navigator_ptr_;
  apollo::common::monitor::MonitorLogger monitor_logger_;

  RoutingConfig routing_conf_;
  const hdmap::HDMap *hdmap_ = nullptr;
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_ROUTING_H_
