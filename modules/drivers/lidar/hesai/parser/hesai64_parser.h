/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "cyber/cyber.h"
#include "modules/drivers/lidar/hesai/parser/parser.h"

namespace apollo {
namespace drivers {
namespace hesai {

class Hesai64Parser : public Parser {
 public:
  Hesai64Parser(const std::shared_ptr<::apollo::cyber::Node>& node,
                const Config& conf);
  ~Hesai64Parser();

 protected:
  void ParseRawPacket(const uint8_t* buf, const int len, bool* is_end) override;

 private:
  void CalcPointXYZIT(Hesai64Packet* pkt, int blockid, uint8_t chLaserNumber);
  double block_offset_[BLOCKS_PER_PACKET_L64] = {0};
  double laser_offset_[LASER_COUNT_L64] = {0};
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
