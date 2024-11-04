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

#include "modules/drivers/lidar/lslidar/driver/driver.h"

namespace apollo {
namespace drivers {
namespace lslidar {
namespace driver {

LslidarDriver::~LslidarDriver() {
    if (difop_thread_.joinable()) {
        difop_thread_.join();
    }
}

void LslidarDriver::Init() {
    int packets_rate;

    if (config_.model() == LSLIDAR16P || config_.model() == LSLIDAR_C16_V4) {
        packets_rate = 840;
    } else if (config_.model() == LSLIDAR_C8_V4) {
        packets_rate = 420;
    } else if (config_.model() == LSLIDAR_C1_V4) {
        packets_rate = 420;
    } else if (config_.model() == LSLIDAR_C32_V4) {
        packets_rate = 1693;  // 64000/384
    } else if (config_.model() == LSLIDAR32P) {
        if (1 == config_.degree_mode())
            packets_rate = 1693;  // 65000/384
        if (2 == config_.degree_mode())
            packets_rate = 1700;  // 64000/384
    } else if (config_.model() == LSLIDAR_CH16) {
        packets_rate = 3571;
    } else if (config_.model() == LSLIDAR_CH32) {
        packets_rate = 3512;
    } else if (config_.model() == LSLIDAR_CH64) {
        packets_rate = 3388;
    } else if (config_.model() == LSLIDAR_CH64w) {
        packets_rate = 11228;
    } else if (config_.model() == LSLIDAR_CH128) {
        packets_rate = 3571;
    } else if (config_.model() == LSLIDAR_CH128X1) {
        packets_rate = 6720;
    } else if (config_.model() == LSLIDAR_LS128S2) {
        packets_rate = 12440;
    } else {
        packets_rate = 4561;
    }

    // default number of packets for each scan is a single revolution
    if (config_.model() == LSLIDAR16P || config_.model() == LSLIDAR32P
        || config_.model() == LSLIDAR_C32_V4
        || config_.model() == LSLIDAR_C16_V4 || config_.model() == LSLIDAR_C8_V4
        || config_.model() == LSLIDAR_C1_V4)
        config_.set_npackets(static_cast<int>(
                ceil(packets_rate * 60 / config_.rpm()) * config_.return_mode()
                * 1.1));
    else
        config_.set_npackets(static_cast<int>(
                ceil(packets_rate * 60 / config_.rpm() * 1.1)));

    AERROR << "config_.pcap_path(): " << config_.pcap_path();

    if (!config_.pcap_path().empty()) {
        if (config_.model() == LSLIDAR_C32_V4
            || config_.model() == LSLIDAR_C16_V4
            || config_.model() == LSLIDAR_C8_V4
            || config_.model() == LSLIDAR_C1_V4) {
            // open Lslidar input device

            input_.reset(new InputPCAP(
                    config_.msop_port(),
                    config_.device_ip(),
                    config_.packet_size(),
                    packets_rate,
                    config_.pcap_path()));  // 数据包
            positioning_input_.reset(new InputPCAP(
                    config_.difop_port(),
                    config_.device_ip(),
                    1206,
                    1,
                    config_.pcap_path()));  // 设备包
        } else {
            // open Lslidar input device
            input_.reset(new InputPCAP(
                    config_.msop_port(),
                    config_.device_ip(),
                    config_.packet_size(),
                    packets_rate,
                    config_.pcap_path()));  // 数据包
            positioning_input_.reset(new InputPCAP(
                    config_.difop_port(),
                    config_.device_ip(),
                    config_.packet_size(),
                    1,
                    config_.pcap_path()));  // 设备包
        }
    } else {
        if (config_.model() == LSLIDAR_C32_V4
            || config_.model() == LSLIDAR_C16_V4
            || config_.model() == LSLIDAR_C8_V4
            || config_.model() == LSLIDAR_C1_V4) {
            // open Lslidar input device
            input_.reset(new InputSocket(
                    config_.msop_port(),
                    config_.device_ip(),
                    config_.packet_size()));  // 数据包
            positioning_input_.reset(new InputSocket(
                    config_.difop_port(),
                    config_.device_ip(),
                    1206));  // 设备包
        } else {
            // open Lslidar input device
            input_.reset(new InputSocket(
                    config_.msop_port(),
                    config_.device_ip(),
                    config_.packet_size()));  // 数据包
            positioning_input_.reset(new InputSocket(
                    config_.difop_port(),
                    config_.device_ip(),
                    config_.packet_size()));  // 设备包
        }
    }
    difop_thread_ = std::thread(&LslidarDriver::difopPoll, this);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool LslidarDriver::Poll(
        const std::shared_ptr<apollo::drivers::lslidar::LslidarScan> &scan) {
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    int poll_result = PollStandard(scan);
    if (poll_result < 0)
        return false;

    if (scan->firing_pkts().empty()) {
        AINFO << "Get an empty scan from port: " << config_.msop_port();
        return false;
    }
    // publish message using time of last packet read
    ADEBUG << "Publishing a full Lslidar scan.";
    LslidarPacket *packet = scan->add_difop_pkts();
    std::unique_lock<std::mutex> lock(mutex_);
    packet->set_data(bytes, FIRING_DATA_PACKET_SIZE);
    scan->mutable_header()->set_timestamp_sec(gps_time);
    ADEBUG << "**************************************************************"
              "GPS "
              "time: "
           << gps_time;
    scan->mutable_header()->set_frame_id(config_.frame_id());
    scan->set_model(config_.model());
    scan->set_basetime(gps_time);
    scan->mutable_header()->set_timestamp_sec(gps_time);
    return true;
}

int LslidarDriver::PollStandard(
        std::shared_ptr<apollo::drivers::lslidar::LslidarScan> scan) {
    // Since the lslidar delivers data at a very high rate, keep reading and
    // publishing scans as fast as possible.

    if (config_.model() == LSLIDAR16P || config_.model() == LSLIDAR32P
        || config_.model() == LSLIDAR_C32_V4
        || config_.model() == LSLIDAR_C16_V4 || config_.model() == LSLIDAR_C8_V4
        || config_.model() == LSLIDAR_C1_V4) {
        // Find the 0 degree packet, in order to fix the 0 degree angle position
        if (scan_fill) {
            LslidarPacket *packet = scan->add_firing_pkts();
            int PKT_DATA_LENGTH = 1212;
            void *data_ptr = malloc(PKT_DATA_LENGTH);
            memcpy(data_ptr,
                   reinterpret_cast<uint8_t *>(
                           const_cast<char *>(scan_start.data().c_str())),
                   PKT_DATA_LENGTH);
            packet->set_data(data_ptr, PKT_DATA_LENGTH);
            packet->set_stamp((scan_start.stamp()));
            AINFO << "scan->firing_pkts_size(): " << scan->firing_pkts_size();
        }

        scan_fill = false;
        int i = 1;
        while (scan->firing_pkts_size() < config_.npackets()) {
            // config_.npackets()
            LslidarPacket *packet = scan->add_firing_pkts();
            while (true) {
                // keep reading until full packet received
                int rc = input_->GetPacket(packet);

                if (rc == 0) {
                    if (!config_.time_synchronization()) {
                        time_t t = time(NULL);
                        localtime_r(&t, &current_time);
                        current_time.tm_hour = current_time.tm_hour - 8;
                        packet_time_ns_ = 0;
                        gps_time = apollo::cyber::Time().Now().ToNanosecond();
                    } else {
                        uint8_t *data = reinterpret_cast<uint8_t *>(
                                const_cast<char *>(packet->data().c_str()));
                        if (config_.packet_size() == 1212) {  // 1212字节雷达
                            if (0xff == data[1200]) {         // ptp授时
                                // std::cout << "ptp";
                                basetime_
                                        = (data[1201] * pow(2, 32)
                                           + data[1202] * pow(2, 24)
                                           + data[1203] * pow(2, 16)
                                           + data[1204] * pow(2, 8)
                                           + data[1205] * pow(2, 0));
                                packet_time_ns_
                                        = (static_cast<uint16_t>(data[1206])
                                           + static_cast<uint16_t>(data[1207])
                                                   * pow(2, 8)
                                           + static_cast<uint16_t>(data[1208])
                                                   * pow(2, 16)
                                           + static_cast<uint16_t>(data[1209])
                                                   * pow(2, 24));
                            } else {  // gps授时
                                current_time.tm_year
                                        = static_cast<uint16_t>(data[1200])
                                        + 2000 - 1900;
                                current_time.tm_mon
                                        = static_cast<uint16_t>(data[1201]) - 1;
                                current_time.tm_mday
                                        = static_cast<uint16_t>(data[1202]);
                                current_time.tm_hour
                                        = static_cast<uint16_t>(data[1203]);
                                current_time.tm_min
                                        = static_cast<uint16_t>(data[1204]);

                                if (config_.model() == LSLIDAR16P
                                    || config_.model() == LSLIDAR32P) {
                                    current_time.tm_sec
                                            = static_cast<uint16_t>(data[1205])
                                            + 1;
                                    packet_time_ns_
                                            = (static_cast<uint16_t>(data[1206])
                                               + static_cast<uint16_t>(
                                                         data[1207])
                                                       * pow(2, 8)
                                               + static_cast<uint16_t>(
                                                         data[1208])
                                                       * pow(2, 16)
                                               + static_cast<uint16_t>(
                                                         data[1209])
                                                       * pow(2, 24))
                                            * 1e3;  // ns
                                } else if (
                                        config_.model() == LSLIDAR_C32_V4
                                        || config_.model() == LSLIDAR_C16_V4
                                        || config_.model() == LSLIDAR_C8_V4
                                        || config_.model() == LSLIDAR_C1_V4) {
                                    current_time.tm_sec
                                            = static_cast<uint16_t>(data[1205]);
                                    packet_time_ns_
                                            = (static_cast<uint16_t>(data[1206])
                                               + static_cast<uint16_t>(
                                                         data[1207])
                                                       * pow(2, 8)
                                               + static_cast<uint16_t>(
                                                         data[1208])
                                                       * pow(2, 16)
                                               + static_cast<uint16_t>(
                                                         data[1209])
                                                       * pow(2, 24));
                                }
                                basetime_ = static_cast<uint64_t>(
                                        timegm(&current_time));
                            }
                        } else {  // 1206字节雷达，V3.0
                            uint8_t *data = reinterpret_cast<uint8_t *>(
                                    const_cast<char *>(packet->data().c_str()));
                            packet_time_ns_
                                    = (static_cast<uint16_t>(data[1200])
                                       + static_cast<uint16_t>(data[1201])
                                               * pow(2, 8)
                                       + static_cast<uint16_t>(data[1202])
                                               * pow(2, 16)
                                       + static_cast<uint16_t>(data[1203])
                                               * pow(2, 24))
                                    * 1e3;  // ns
                            basetime_ = static_cast<uint64_t>(
                                    timegm(&current_time));
                        }
                        gps_time = basetime_ * 1000000000 + packet_time_ns_;
                    }
                    packet->set_stamp(gps_time);
                    break;
                } else if (rc < 0) {
                    return rc;
                }
            }
            packet->set_stamp(gps_time);  // 設置包的時間

            uint8_t *data = reinterpret_cast<uint8_t *>(
                    const_cast<char *>(packet->data().c_str()));
            int azi1 = 256 * static_cast<uint16_t>(data[3])
                    + static_cast<uint16_t>(data[2]);
            int azi2 = 256 * static_cast<uint16_t>(data[1103])
                    + static_cast<uint16_t>(data[1102]);

            if (((azi1 > 35000 && azi2 < 1000)
                 || (azi1 < 500 && i > config_.npackets() / 2))) {
                scan_fill = true;
                scan_start = *packet;
                break;
            }
            i++;
        }
    } else if (config_.model() == LSLIDAR_LS128S2) {  // LS128S2
        // Find the 0 degree packet, in order to fix the 0 degree angle position
        scan_fill = false;
        int i = 1;
        bool is_found_frame_header = false;
        while (scan->firing_pkts_size() < config_.npackets()
               && !is_found_frame_header) {
            LslidarPacket *packet = scan->add_firing_pkts();
            while (true) {
                // keep reading until full packet received
                int rc = input_->GetPacket(packet);
                AINFO << "[debug ] line: " << __LINE__
                      << "  file: " << __FILE__;
                if (rc == 0) {
                    if (!config_.time_synchronization()) {
                        time_t t = time(NULL);
                        localtime_r(&t, &current_time);
                        current_time.tm_hour = current_time.tm_hour - 8;
                        gps_time = apollo::cyber::Time().Now().ToNanosecond();
                    } else {
                        uint8_t *data = reinterpret_cast<uint8_t *>(
                                const_cast<char *>(packet->data().c_str()));
                        if (0xff
                            == static_cast<uint16_t>(data[1194])) {  // ptp授时
                            basetime_
                                    = (static_cast<uint16_t>(data[1195]) * 0
                                       + (static_cast<uint16_t>(data[1196])
                                          << 24)
                                       + (static_cast<uint16_t>(data[1197])
                                          << 16)
                                       + (static_cast<uint16_t>(data[1198])
                                          << 8)
                                       + static_cast<uint16_t>(data[1199])
                                               * pow(2, 0));
                            packet_time_ns_
                                    = (static_cast<uint16_t>(data[1200]) << 24)
                                    + (static_cast<uint16_t>(data[1201]) << 16)
                                    + (static_cast<uint16_t>(data[1202]) << 8)
                                    + (static_cast<uint16_t>(data[1203]));
                            gps_time = basetime_ * 1000000000 + packet_time_ns_;
                        } else {  // gps授时
                            struct tm cur_time {};
                            memset(&cur_time, 0, sizeof(cur_time));
                            cur_time.tm_sec = static_cast<uint16_t>(data[1199]);
                            cur_time.tm_min = static_cast<uint16_t>(data[1198]);
                            cur_time.tm_hour
                                    = static_cast<uint16_t>(data[1197]);
                            cur_time.tm_mday
                                    = static_cast<uint16_t>(data[1196]);
                            cur_time.tm_mon
                                    = static_cast<uint16_t>(data[1195]) - 1;
                            cur_time.tm_year = static_cast<uint16_t>(data[1194])
                                    + 2000 - 1900;
                            basetime_
                                    = static_cast<uint64_t>(timegm(&cur_time));
                            packet_time_ns_ = static_cast<uint16_t>(data[1203])
                                    + (static_cast<uint16_t>(data[1202]) << 8)
                                    + (static_cast<uint16_t>(data[1201]) << 16)
                                    + (static_cast<uint16_t>(data[1200])
                                       << 24);  // ns
                            gps_time = basetime_ * 1000000000 + packet_time_ns_;
                        }
                    }
                    break;
                } else if (rc < 0) {
                    return rc;
                }
            }
            packet->set_stamp(gps_time);  // 設置包的時間
            uint8_t *data = reinterpret_cast<uint8_t *>(
                    const_cast<char *>(packet->data().c_str()));
            int return_mode = static_cast<uint16_t>(data[1205]);

            if (return_mode == 1) {
                for (size_t point_idx = 0;
                     point_idx < LS_POINTS_PER_PACKET_SINGLE_ECHO;
                     point_idx += 8) {  // 一圈
                    if ((static_cast<uint16_t>(data[point_idx]) == 0xff)
                        && (static_cast<uint16_t>(data[point_idx + 1]) == 0xaa)
                        && (static_cast<uint16_t>(data[point_idx + 2]) == 0xbb)
                        && (static_cast<uint16_t>(data[point_idx + 3]) == 0xcc)
                        && (static_cast<uint16_t>(data[point_idx + 4])
                            == 0xdd)) {
                        scan_fill = false;
                        is_found_frame_header = true;

                        break;
                    }
                }
            } else {
                for (size_t point_idx = 0;
                     point_idx < LS_POINTS_PER_PACKET_DOUBLE_ECHO;
                     point_idx += 12) {  // 一圈
                    if ((static_cast<uint16_t>(data[point_idx]) == 0xff)
                        && (static_cast<uint16_t>(data[point_idx + 1]) == 0xaa)
                        && (static_cast<uint16_t>(data[point_idx + 2]) == 0xbb)
                        && (static_cast<uint16_t>(data[point_idx + 3]) == 0xcc)
                        && (static_cast<uint16_t>(data[point_idx + 4])
                            == 0xdd)) {
                        scan_fill = false;
                        is_found_frame_header = true;
                        AERROR << "\none circle! scan->firing_pkts_size(): "
                               << scan->firing_pkts_size();
                        break;
                    }
                }
            }
            i++;
        }
    } else {  // ch系列
        // Find the 0 degree packet, in order to fix the 0 degree angle position
        scan_fill = false;
        int i = 1;
        bool is_found_frame_header = false;
        while (scan->firing_pkts_size() < config_.npackets()
               && !is_found_frame_header) {
            LslidarPacket *packet = scan->add_firing_pkts();
            while (true) {
                // keep reading until full packet received
                int rc = input_->GetPacket(packet);
                AINFO << "[debug ] line: " << __LINE__
                      << "  file: " << __FILE__;
                if (rc == 0) {
                    if (config_.model() == LSLIDAR_CH64w
                        || config_.model() == LSLIDAR_CH120
                        || config_.model() == LSLIDAR_CH128X1
                        || config_.model() == LSLIDAR_CH32) {
                        if (!config_.time_synchronization()) {
                            time_t t = time(NULL);
                            localtime_r(&t, &current_time);
                            current_time.tm_hour = current_time.tm_hour - 8;
                            gps_time = apollo::cyber::Time()
                                               .Now()
                                               .ToNanosecond();
                        } else {
                            uint8_t *data = reinterpret_cast<uint8_t *>(
                                    const_cast<char *>(packet->data().c_str()));
                            current_time.tm_hour
                                    = static_cast<uint16_t>(data[1197]);
                            current_time.tm_min
                                    = static_cast<uint16_t>(data[1198]);
                            current_time.tm_sec
                                    = static_cast<uint16_t>(data[1199]);
                            basetime_ = static_cast<uint64_t>(
                                    timegm(&current_time));
                            if (time_service_mode == "gps") {
                                packet_time_ns_
                                        = (static_cast<uint16_t>(data[1203])
                                           + static_cast<uint16_t>(data[1202])
                                                   * pow(2, 8)
                                           + static_cast<uint16_t>(data[1201])
                                                   * pow(2, 16)
                                           + static_cast<uint16_t>(data[1200])
                                                   * pow(2, 24))
                                        * 1e3;  // ns
                            } else if (time_service_mode == "gptp") {
                                packet_time_ns_
                                        = (static_cast<uint16_t>(data[1203])
                                           + static_cast<uint16_t>(data[1202])
                                                   * pow(2, 8)
                                           + static_cast<uint16_t>(data[1201])
                                                   * pow(2, 16)
                                           + static_cast<uint16_t>(data[1200])
                                                   * pow(2, 24));  // ns
                            }
                            gps_time = basetime_ * 1000000000 + packet_time_ns_;
                        }
                    }
                    break;
                } else if (rc < 0) {
                    return rc;
                }
            }
            packet->set_stamp(gps_time);  // 設置包的時間
            uint8_t *data = reinterpret_cast<uint8_t *>(
                    const_cast<char *>(packet->data().c_str()));

            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET;
                 point_idx += 7) {  // 一圈
                if ((static_cast<uint16_t>(data[point_idx]) == 0xff)
                    && (static_cast<uint16_t>(data[point_idx + 1]) == 0xaa)
                    && (static_cast<uint16_t>(data[point_idx + 2]) == 0xbb)
                    && (static_cast<uint16_t>(data[point_idx + 3]) == 0xcc)) {
                    scan_fill = false;
                    is_found_frame_header = true;
                    AERROR << "\none circle! scan->firing_pkts_size(): "
                           << scan->firing_pkts_size();
                    break;
                }
            }
            i++;
        }
    }
    return 0;
}

void LslidarDriver::difopPoll(void) {
    while (!cyber::IsShutdown()) {
        std::shared_ptr<apollo::drivers::lslidar::LslidarScan> scans
                = std::make_shared<apollo::drivers::lslidar::LslidarScan>();
        std::shared_ptr<LslidarPacket> packet
                = std::make_shared<LslidarPacket>();
        while (!cyber::IsShutdown()) {
            int rc = positioning_input_->GetPacket(packet.get());
            if (rc == 0) {
                break;  // got a full packet
            }
            if (rc < 0) {
                return;
            }
        }
        uint8_t *data = reinterpret_cast<uint8_t *>(
                const_cast<char *>(packet->data().c_str()));
        {
            std::unique_lock<std::mutex> lock(mutex_);
            memcpy(bytes, data, FIRING_DATA_PACKET_SIZE);
        }
        if (!config_.time_synchronization()) {
            time_t t = time(NULL);
            localtime_r(&t, &current_time);
            current_time.tm_hour = current_time.tm_hour - 8;
        } else {
            if (data[0] == 0xA5 && data[1] == 0xFF && data[2] == 0x00
                && data[3] == 0x5A) {
                if (config_.model() == LSLIDAR_CH16
                    || config_.model() == LSLIDAR_CH32
                    || config_.model() == LSLIDAR_CH128
                    || config_.model() == LSLIDAR_CH64) {
                    current_time.tm_year
                            = static_cast<uint16_t>(data[36] + 100);
                    current_time.tm_mon = static_cast<uint16_t>(data[37] - 1);
                    current_time.tm_mday = static_cast<uint16_t>(data[38]);
                    current_time.tm_hour = static_cast<uint16_t>(data[39]);
                    current_time.tm_min = static_cast<uint16_t>(data[40]);
                    current_time.tm_sec = static_cast<uint16_t>(data[41]);
                } else if (config_.model() == LSLIDAR_CH64w) {
                    current_time.tm_year
                            = static_cast<uint16_t>(data[52] + 100);
                    current_time.tm_mon = static_cast<uint16_t>(data[53] - 1);
                    current_time.tm_mday = static_cast<uint16_t>(data[54]);
                    if (data[44] == 0x00) {  // gps授时
                        time_service_mode = "gps";
                    } else if (data[44] == 0x01) {  // ptp授时
                        time_service_mode = "gptp";
                    }
                } else if (config_.model() == LSLIDAR_CH120) {
                    current_time.tm_year
                            = static_cast<uint16_t>(data[36] + 100);
                    current_time.tm_mon = static_cast<uint16_t>(data[37] - 1);
                    current_time.tm_mday = static_cast<uint16_t>(data[38]);
                } else if (config_.model() == LSLIDAR_CH128X1) {
                    current_time.tm_year
                            = static_cast<uint16_t>(data[52] + 100);
                    current_time.tm_mon = static_cast<uint16_t>(data[53] - 1);
                    current_time.tm_mday = static_cast<uint16_t>(data[54]);
                    if (data[44] == 0x00) {  // gps授时
                        time_service_mode = "gps";
                    } else if (data[44] == 0x01) {  // ptp授时
                        time_service_mode = "gptp";
                    }
                } else if (
                        config_.packet_size() == 1206
                        && (config_.model() == LSLIDAR32P
                            || config_.model() == LSLIDAR16P)) {
                    current_time.tm_year
                            = static_cast<uint16_t>(data[52] + 100);
                    current_time.tm_mon = static_cast<uint16_t>(data[53] - 1);
                    current_time.tm_mday = static_cast<uint16_t>(data[54]);
                    current_time.tm_hour = static_cast<uint16_t>(data[55]);
                    current_time.tm_min = static_cast<uint16_t>(data[56]);
                    current_time.tm_sec = static_cast<uint16_t>(data[57]);
                }
            }
        }
    }
}

LslidarDriver *LslidarDriverFactory::CreateDriver(const Config &config) {
    LslidarDriver *driver = nullptr;
    driver = new LslidarDriver(config);
    return driver;
}

}  // namespace driver
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
