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

#pragma once

#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>

#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/util/util.h"
#include "modules/bridge/common/bridge_gflags.h"

namespace apollo {
namespace bridge {

using apollo::cyber::io::Session;
using apollo::localization::LocalizationEstimate;

template<typename T>
class UDPBridgeComponent final
    : public cyber::Component<T> {
 public:
  UDPBridgeComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

  bool Init() override;
  bool Proc(const std::shared_ptr<T> &pb_msg) override;

  std::string Name() const {
    return FLAGS_bridge_module_name;
  }

 private:
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(UDPBridgeComponent<planning::ADCTrajectory>)
CYBER_REGISTER_COMPONENT(UDPBridgeComponent<LocalizationEstimate>)

}  // namespace bridge
}  // namespace apollo
