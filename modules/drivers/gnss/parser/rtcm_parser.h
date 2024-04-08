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

#include <memory>
#include <string>

#include "cyber/cyber.h"

#include "modules/drivers/gnss/parser/parser.h"

#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

class RtcmParser {
 public:
  using MessagePtr = ::google::protobuf::Message*;
  RtcmParser(const config::Config& config,
             const std::shared_ptr<apollo::cyber::Node>& node);
  ~RtcmParser() {}
  bool Init();
  void ParseRtcmData(const std::string& msg);

 private:
  void DispatchMessage(MessageType type, MessagePtr message);
  void PublishEphemeris(const MessagePtr& message);
  void PublishObservation(const MessagePtr& message);

  config::Config config_;
  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<GnssEphemeris>> gnssephemeris_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<EpochObservation>>
      epochobservation_writer_ = nullptr;
  bool init_flag_ = false;
  std::unique_ptr<Parser> rtcm_parser_ = nullptr;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
