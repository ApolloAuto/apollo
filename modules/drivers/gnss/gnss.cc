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

#include "modules/drivers/gnss/gnss.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/gnss/gnss_gflags.h"
#include "modules/drivers/gnss/proto/config.pb.h"

/**
 * @namespace apollo::drivers::smartereye
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::common::adapter::AdapterManager;

std::string GnssDriver::Name() const { return FLAGS_sensor_node_name; }

apollo::common::Status GnssDriver::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);
  AINFO << "The adapter manager is successfully initialized.";

  config::Config config;
  // load camera config
  if (!::apollo::common::util::GetProtoFromFile(FLAGS_sensor_conf_file,
                                                &config)) {
    return OnError("Unable to load gnss conf file: " + FLAGS_sensor_conf_file);
  }
  AINFO << "Gnss config: " << config.DebugString();

  raw_stream_.reset(new RawStream(config));

  if (!raw_stream_->Init()) {
    return OnError("Init stream failed!");
  }

  AINFO << "The gnss driver successfully initialized.";
  return Status::OK();
}

apollo::common::Status GnssDriver::Start() {
  raw_stream_->Start();

  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("gnss is started.");

  return Status::OK();
}

void GnssDriver::Stop() {}

// Send the error to monitor and return it
Status GnssDriver::OnError(const std::string &error_msg) {
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::DRIVER_ERROR_GNSS, error_msg);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
