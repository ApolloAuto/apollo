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
#include "modules/drivers/gnss/parser/asensing_parser/asensing_base_parser.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace apollo {
namespace drivers {
namespace gnss {

AsensingBaseParser::AsensingBaseParser(const config::Config &config)
    : auto_fill_gps_msg_(config.auto_fill_gps_msg()) {}

void AsensingBaseParser::GetMessages(MessageInfoVec *messages) {
  if (data_ == nullptr) {
    return;
  }
  if (!PrepareMessage()) {
    return;
  }

  FillGnssBestpos();
  FillImu();
  FillHeading();
  FillIns();
  FillInsStat();

  if (bestpos_ratecontrol_.check()) {
    messages->push_back(MessageInfo{MessageType::BEST_GNSS_POS,
                                    reinterpret_cast<MessagePtr>(&bestpos_)});
  }
  messages->push_back(
      MessageInfo{MessageType::IMU, reinterpret_cast<MessagePtr>(&imu_)});
  messages->push_back(MessageInfo{MessageType::HEADING,
                                  reinterpret_cast<MessagePtr>(&heading_)});
  messages->push_back(
      MessageInfo{MessageType::INS, reinterpret_cast<MessagePtr>(&ins_)});
  messages->push_back(MessageInfo{MessageType::INS_STAT,
                                  reinterpret_cast<MessagePtr>(&ins_stat_)});
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
