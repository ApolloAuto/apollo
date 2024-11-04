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

#include <atomic>
#include <cmath>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>

#include "modules/drivers/lidar/lslidar/proto/config.pb.h"
#include "modules/drivers/lidar/lslidar/proto/lslidar.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/lidar/lslidar/driver/input.h"

namespace apollo {
namespace drivers {
namespace lslidar {
namespace driver {

constexpr int BLOCKS_PER_PACKET = 12;
constexpr int BLOCK_SIZE = 100;

static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;
static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;
static const int POINTS_PER_PACKET = 171 * 7;  // ch系列
static const int LS_POINTS_PER_PACKET_SINGLE_ECHO
        = 149 * 8;  // LS 1550nm系列 单回波
static const int LS_POINTS_PER_PACKET_DOUBLE_ECHO
        = 99 * 12;  // LS 1550nm系列 单回波

class LslidarDriver {
 public:
    explicit LslidarDriver(const Config &config) : config_(config) {
        //    scan_start = new LslidarPacket();
    }
    ~LslidarDriver();

    bool Poll(
            const std::shared_ptr<apollo::drivers::lslidar::LslidarScan> &scan);
    void Init();
    void difopPoll();
    void SetPacketRate(const double packet_rate) {
        packet_rate_ = packet_rate;
    }
    int npackets;
    struct tm current_time;

 protected:
    Config config_;
    std::unique_ptr<Input> input_ = nullptr;
    std::unique_ptr<Input> positioning_input_ = nullptr;
    std::string topic_;
    double packet_rate_ = 0.0;
    bool scan_fill = false;
    uint64_t gps_time = 0;
    uint64_t last_gps_time = 0;

    uint64_t basetime_ = 0;
    uint64_t packet_time_ns_ = 0;

    uint32_t last_gps_time_ = 0;
    uint64_t last_count_ = 0;
    static uint64_t sync_counter;

    std::thread difop_thread_;
    int PollStandard(
            std::shared_ptr<apollo::drivers::lslidar::LslidarScan> scan);
    LslidarPacket scan_start;
    LslidarPacket last_scan_start;

    LslidarPacket scan_start1;
    LslidarPacket scan_start2;
    std::mutex mutex_;
    uint8_t bytes[FIRING_DATA_PACKET_SIZE] = {0x00};
    std::string time_service_mode = {"gps"};
};

class LslidarDriverFactory {
 public:
    static LslidarDriver *CreateDriver(const Config &config);
};

}  // namespace driver
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
