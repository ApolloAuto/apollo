/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/radar/oculii_radar/oculii_radar_component.h"

#include "modules/common/util/message_util.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace radar {

bool OculiiRadarComponent::Init() {
  config_ = std::make_shared<OculiiRadarConf>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               config_.get())) {
    return false;
  }
  AINFO << "OculiiRadarComponent config: " << config_->DebugString();
  parser_.reset(new OculiiRadarUdpParser);
  if (!parser_->Init(config_->port())) {
    AERROR << "Init parser of oculii radar failed";
    return false;
  }
  writer_ = node_->CreateWriter<OculiiPointCloud>(config_->channel_name());
  frame_drop_interval_ = static_cast<float>(0.9 / config_->frame_rate());
  last_process_time_ = 0;
  async_result_ = cyber::Async(&OculiiRadarComponent::run, this);
  return true;
}

void OculiiRadarComponent::run() {
  running_.exchange(true);
  while (!cyber::IsShutdown()) {
    OculiiPointCloud pb_output;

    cyber::Time process_time = cyber::Time::Now();
    auto output_time = process_time.ToSecond();

    pb_output.mutable_header()->set_frame_id(config_->frame_id());
    common::util::FillHeader(node_->Name(), &pb_output);
    pb_output.set_frame_id(config_->frame_id());
    pb_output.set_is_dense(true);
    pb_output.set_measurement_time(output_time);

    if (parser_->Parse(pb_output) != 0) {
      AERROR << "parse oculii udp packet failed";
      continue;
    }
    writer_->Write(pb_output);
  }
}

OculiiRadarComponent::~OculiiRadarComponent() {
  if (running_.load()) {
    running_.exchange(false);
  }
}

}  // namespace radar
}  // namespace drivers
}  // namespace apollo
