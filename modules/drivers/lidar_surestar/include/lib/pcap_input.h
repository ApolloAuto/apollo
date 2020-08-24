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

#ifndef CYBERTRON_ONBOARD_DRIVERS_SURESTAR_INCLUDE_VELODYNE_LIB_PCAP_INPUT_H
#define CYBERTRON_ONBOARD_DRIVERS_SURESTAR_INCLUDE_VELODYNE_LIB_PCAP_INPUT_H

#include <pcap.h>
#include <stdio.h>
#include <unistd.h>

#include <string>

#include "modules/drivers/lidar_surestar/include/lib/input.h"

namespace autobot {
namespace drivers {
namespace surestar {

class PcapInput : public Input {
 public:
  PcapInput(double packet_rate, const std::string& filename,
            bool read_once = false, bool read_fast = false,
            double repeat_delay = 0.0);
  virtual ~PcapInput();

  void init();
  int get_firing_data_packet(
      adu::common::sensor::Surestar::SurestarPacket* pkt);
  int get_positioning_data_packtet(const NMEATimePtr& nmea_time);

 private:
  std::string _filename;
  FILE* _fp;
  pcap* _pcap;
  char _errbuf[PCAP_ERRBUF_SIZE];
  bool _empty;
  bool _read_once;
  bool _read_fast;
  double _repeat_delay;
  double _packet_rate;
};
}  // namespace surestar
}  // namespace drivers
}  // namespace autobot

#endif  // CYBERTRON_ONBOARD_DRIVERS_SURESTAR_INCLUDE_VELODYNE_LIB_PCAP_INPUT_H
