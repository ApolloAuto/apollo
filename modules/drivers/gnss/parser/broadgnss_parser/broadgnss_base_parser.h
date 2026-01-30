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

#include <string>

#include "modules/common_msgs/sensor_msgs/gnss.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/broadgnss_parser/broadgnss_message.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/parser_common.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

class BroadGnssBaseParser : public Parser {
 public:
  BroadGnssBaseParser() {}
  explicit BroadGnssBaseParser(const config::Config &config);

  virtual bool PrepareMessage(const uint8_t &channel) = 0;

  virtual void GetMessages(const uint8_t &channel, MessageInfoVec *messages);
  virtual void GetMessages(MessageInfoVec *messages);

 protected:
  void PrepareMessageStatus(const uint8_t &solution_status,
                            const uint8_t &solution_type);
  BroadGnssMessage broadgnss_message_;

 private:
  void FillGnssBestpos();
  void FillIns();
  void FillInsStat();
  void FillImu();
  void FillHeading();

  GnssBestPose bestpos_;
  // bestpos 1hz rate control
  RateControl bestpos_ratecontrol_{PERIOD_NS_1HZ};
  Imu imu_;
  Heading heading_;
  Ins ins_;
  InsStat ins_stat_;
  bool auto_fill_gps_msg_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
