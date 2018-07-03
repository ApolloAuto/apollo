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

#ifndef MODULES_DRIVERS_GNSS_RTCM_PARSER_H_
#define MODULES_DRIVERS_GNSS_RTCM_PARSER_H_

#include <std_msgs/String.h>
#include <memory>
#include "ros/include/ros/ros.h"

#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class RtcmParser {
 public:
  RtcmParser() {}
  ~RtcmParser() {}
  bool Init();
  void ParseRtcmData(const std_msgs::String::ConstPtr &msg);

 private:
  void DispatchMessage(Parser::MessageType type, MessagePtr message);
  void PublishEphemeris(const MessagePtr message);
  void PublishObservation(const MessagePtr message);

  bool inited_flag_ = false;
  std::unique_ptr<Parser> rtcm_parser_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_RTCM_PARSER_H_
