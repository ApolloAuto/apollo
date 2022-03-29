/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include <map>

#include "modules/drivers/lidar/lidar_robosense/parser/robosense_parser.h"

namespace apollo {
namespace drivers {
namespace robosense {

class Robosense16Parser : public RobosenseParser {
 public:
  explicit Robosense16Parser(
      const apollo::drivers::suteng::SutengConfig& config);
  ~Robosense16Parser() {}

  void generate_pointcloud(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan const>&
          scan_msg,
      const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
  void order(const std::shared_ptr<apollo::drivers::PointCloud>& cloud);
  uint32_t GetPointSize() override;
  void setup() override;
  void init_setup();

 private:
  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id);

  void unpack_robosense(
      const apollo::drivers::suteng::SutengPacket& pkt,
      const std::shared_ptr<apollo::drivers::PointCloud>& cloud,
      uint32_t* index);

  // Previous suteng packet time stamp. (offset to the top hour)
  double previous_packet_stamp_;
  uint64_t gps_base_usec_;  // full time
  std::map<uint32_t, uint32_t> order_map_;
  uint32_t getOrderIndex(uint32_t index);
  void init_orderindex();
  RslidarPic pic;

  // sutegn
  int temp_packet_num = 0;
  float temper = 31.f;

  static bool pkt_start;
  static uint64_t base_stamp;

  uint64_t first_pkt_stamp;
  uint64_t final_pkt_stamp;
  uint64_t last_pkt_stamp = 0;
};  // class Robosense32Parser

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
