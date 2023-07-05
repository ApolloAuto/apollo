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
#include "modules/drivers/gnss/gnss_component.h"

namespace apollo {
namespace drivers {
namespace gnss {

using apollo::cyber::proto::RoleAttributes;

GnssDriverComponent::GnssDriverComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::GNSS) {}

bool GnssDriverComponent::Init() {
  config::Config gnss_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &gnss_config)) {
    monitor_logger_buffer_.ERROR("Unable to load gnss conf file: " +
                                 config_file_path_);
    return false;
  }
  AINFO << "Gnss config: " << gnss_config.DebugString();

  raw_stream_.reset(new RawStream(gnss_config, node_));

  if (!raw_stream_->Init()) {
    monitor_logger_buffer_.ERROR("Init gnss stream failed");
    AERROR << "Init gnss stream failed";
    return false;
  }
  raw_stream_->Start();
  monitor_logger_buffer_.INFO("gnss is started.");
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
