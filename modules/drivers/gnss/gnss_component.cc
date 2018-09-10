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

using apollo::cybertron::proto::RoleAttributes;

GnssDriverComponent::GnssDriverComponent() {}

bool GnssDriverComponent::Init() {
  config::Config gnss_config;
  if(!apollo::cybertron::common::GetProtoFromFile(config_file_path_, &gnss_config)){
    return false;
  }
  AINFO << "Gnss config: " << gnss_config.DebugString();

  raw_stream_.reset(new RawStream(gnss_config, node_));

  if (!raw_stream_->Init()) {
    return false;
  }
  return true;
}

bool GnssDriverComponent::Proc(const std::shared_ptr<RawData>& rawdata) {
  return true;
}

}
}
}
