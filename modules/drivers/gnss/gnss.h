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
 */

#ifndef MODULES_DRIVERS_GNSS_GNSS_H_
#define MODULES_DRIVERS_GNSS_GNSS_H_

#include <memory>
#include <string>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/drivers/gnss/stream/raw_stream.h"

/**
 * @namespace apollo::drivers
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace gnss {

/**
* @class
*
* @brief smarterere camera sensor module main class.
*/

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::Status;
using apollo::common::ErrorCode;

class GnssDriver : public apollo::common::ApolloApp {
 public:
  GnssDriver()
      : monitor_logger_(apollo::common::monitor::MonitorMessageItem::GNSS) {}

  /**
  * @brief obtain module name
  * @return module name
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

 private:
  Status OnError(const std::string &error_msg);
  std::unique_ptr<RawStream> raw_stream_;

  apollo::common::monitor::MonitorLogger monitor_logger_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_GNSS_H_
