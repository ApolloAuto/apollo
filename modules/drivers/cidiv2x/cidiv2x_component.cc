/******************************************************************************
 * Copyright 2019 The CiDi Authors. All Rights Reserved.
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

#include "modules/drivers/cidiv2x/cidiv2x_component.h"
#include "modules/drivers/cidiv2x/stream/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace cidiv2x {

using apollo::cyber::proto::RoleAttributes;

using apollo::drivers::cidiv2x::config::Config;

CidiV2xDriverComponent::CidiV2xDriverComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CIDIV2X) {}

bool CidiV2xDriverComponent::Init() {
  Config config;

  if (!::apollo::common::util::GetProtoFromFile(config_file_path_, &config)) {
    monitor_logger_buffer_.ERROR("Unable to load cidiv2x conf file: " +
                                 config_file_path_);
    return false;
  }
  AINFO << "CidiV2x config: " << config.DebugString();

  raw_stream_.reset(new RawStream(config, node_));

  if (!raw_stream_->Init()) {
    monitor_logger_buffer_.ERROR("Init stream failed!");
    AERROR << "Init CidiV2x rawstream failed";
    return false;
  }

  raw_stream_->Start();
  monitor_logger_buffer_.INFO("cidiv2x is started");
  AINFO << "The cidiv2x driver successfully initialized.";
  return true;
}

}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo
