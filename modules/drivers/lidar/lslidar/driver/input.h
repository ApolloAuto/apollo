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

#include <unistd.h>
#include <cstdio>
#include <string>

#include <pcap.h>
#include "cyber/cyber.h"
#include "modules/drivers/lidar/lslidar/proto/lslidar.pb.h"

namespace apollo {
namespace drivers {
namespace lslidar {
namespace driver {

static const size_t FIRING_DATA_PACKET_SIZE = 1212;
static uint16_t MSOP_DATA_PORT_NUMBER
        = 2368;  // lslidar default data port on PC

class Input {
 public:
    Input(uint16_t portport = MSOP_DATA_PORT_NUMBER,
          std::string lidar_ip = "192.168.1.200",
          int packet_size = 1212);
    virtual ~Input();
    virtual int GetPacket(LslidarPacket *pkt);

 protected:
    int port_;
    int sockfd_;
    uint64_t pointcloudTimeStamp;
    in_addr devip_;
    int packet_size_;
};

/** @brief Live lslidar input from socket. */
class InputSocket : public Input {
 public:
    InputSocket(
            uint16_t port = MSOP_DATA_PORT_NUMBER,
            std::string lidar_ip = "192.168.1.200",
            int packet_size = 1212);

    virtual ~InputSocket();

    virtual int GetPacket(LslidarPacket *pkt);
};

/** @brief lslidar input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap
 */
class InputPCAP : public Input {
 public:
    InputPCAP(
            uint16_t port = MSOP_DATA_PORT_NUMBER,
            std::string lidar_ip = "192.168.1.200",
            int packet_size = 1212,
            double packet_rate = 0.0,
            std::string filename = "",
            bool read_once = false,
            bool read_fast = false,
            double repeat_delay = 0.0);

    virtual ~InputPCAP();

    virtual int GetPacket(LslidarPacket *pkt);

 private:
    double packet_rate_;
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
    std::string lidar_ip_;
};

}  // namespace driver
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
