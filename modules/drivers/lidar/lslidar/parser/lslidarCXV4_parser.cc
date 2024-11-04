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

#include "modules/drivers/lidar/lslidar/parser/lslidar_parser.h"

namespace apollo {
namespace drivers {
namespace lslidar {
namespace parser {

LslidarCXV4Parser::LslidarCXV4Parser(const Config &config) :
        LslidarParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
    vertical_angle_ = config.vertical_angle();
    distance_unit_ = config.distance_unit();
    is_new_c32w_ = true;
    is_get_scan_altitude_ = false;
    AERROR << "vertical_angle_ " << vertical_angle_;
    if (config_.model() == LSLIDAR_C16_V4) {
        lidar_number_ = 16;
        AERROR << "lidar: c16";
        for (int i = 0; i < 16; ++i) {
            sin_scan_altitude[i] = sin(c16_vertical_angle[i] * DEG_TO_RAD);
            cos_scan_altitude[i] = cos(c16_vertical_angle[i] * DEG_TO_RAD);
        }
    } else if (config_.model() == LSLIDAR_C8_V4) {
        lidar_number_ = 8;
        AERROR << "lidar: c8";
        for (int i = 0; i < 8; ++i) {
            sin_scan_altitude[i] = sin(c8_vertical_angle[i] * DEG_TO_RAD);
            cos_scan_altitude[i] = cos(c8_vertical_angle[i] * DEG_TO_RAD);
        }
    } else if (config_.model() == LSLIDAR_C1_V4) {
        lidar_number_ = 1;
        AERROR << "lidar: c1";
        for (int i = 0; i < 8; ++i) {
            sin_scan_altitude[i] = sin(c1_vertical_angle[i] * DEG_TO_RAD);
            cos_scan_altitude[i] = cos(c1_vertical_angle[i] * DEG_TO_RAD);
        }
    } else if (config_.model() == LSLIDAR_C32_V4) {
        lidar_number_ = 32;
        if (32 == config.vertical_angle()) {
            AERROR << "Vertical angle: 32 degrees";
            for (int i = 0; i < 32; ++i) {
                sin_scan_altitude[i]
                        = sin(c32_32_vertical_angle[i] * DEG_TO_RAD);
                cos_scan_altitude[i]
                        = cos(c32_32_vertical_angle[i] * DEG_TO_RAD);
            }
        } else if (70 == config.vertical_angle()) {
            AERROR << "Vertical angle: 70 degrees";
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k]
                        = sin(c32_70_vertical_angle2[k] * DEG_TO_RAD);
                cos_scan_altitude[k]
                        = cos(c32_70_vertical_angle2[k] * DEG_TO_RAD);
            }
        } else if (90 == config.vertical_angle()) {
            AERROR << "Vertical angle: 90 degrees";
            for (int k = 0; k < 32; ++k) {
                sin_scan_altitude[k]
                        = sin(c32_90_vertical_angle[k] * DEG_TO_RAD);
                cos_scan_altitude[k]
                        = cos(c32_90_vertical_angle[k] * DEG_TO_RAD);
            }
        }
    }

    // create the sin and cos table for different azimuth values
    for (int j = 0; j < 36000; ++j) {
        double angle = static_cast<double>(j) / 100.0 * DEG_TO_RAD;
        sin_azimuth_table[j] = sin(angle);
        cos_azimuth_table[j] = cos(angle);
    }
    config_vert_angle = false;
    lslidar_type = 2;
}

void LslidarCXV4Parser::decodePacket(const RawPacket_C32 *packet) {
    // couputer azimuth angle for each firing, 12
    for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
        firings.firing_azimuth[b_idx] = packet->blocks[b_idx].rotation
                % 36000;  //* 0.01 * DEG_TO_RAD;
    }
    for (size_t block_idx = 0; block_idx < BLOCKS_PER_PACKET; ++block_idx) {
        const RawBlock &raw_block = packet->blocks[block_idx];

        int32_t azimuth_diff_b = 0;
        if (config_.return_mode() == 1) {
            if (block_idx < BLOCKS_PER_PACKET - 1) {
                azimuth_diff_b = firings.firing_azimuth[block_idx + 1]
                        - firings.firing_azimuth[block_idx];
                azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000
                                                    : azimuth_diff_b;

            } else {
                azimuth_diff_b = firings.firing_azimuth[block_idx]
                        - firings.firing_azimuth[block_idx - 1];

                azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000
                                                    : azimuth_diff_b;
            }
        } else {
            // return mode 2
            if (block_idx < BLOCKS_PER_PACKET - 2) {
                azimuth_diff_b = firings.firing_azimuth[block_idx + 2]
                        - firings.firing_azimuth[block_idx];
                azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000
                                                    : azimuth_diff_b;

            } else {
                azimuth_diff_b = firings.firing_azimuth[block_idx]
                        - firings.firing_azimuth[block_idx - 2];

                azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000
                                                    : azimuth_diff_b;
            }
        }

        // 32 scan
        for (size_t scan_fir_idx = 0; scan_fir_idx < SCANS_PER_FIRING_C32;
             ++scan_fir_idx) {
            size_t byte_idx = RAW_SCAN_SIZE * scan_fir_idx;
            // azimuth
            firings.azimuth[block_idx * 32 + scan_fir_idx]
                    = firings.firing_azimuth[block_idx]
                    + scan_fir_idx * azimuth_diff_b / FIRING_TOFFSET_C32;
            firings.azimuth[block_idx * 32 + scan_fir_idx]
                    = firings.azimuth[block_idx * 32 + scan_fir_idx] % 36000;
            // distance
            TwoBytes raw_distance{};
            raw_distance.bytes[0] = raw_block.data[byte_idx];
            raw_distance.bytes[1] = raw_block.data[byte_idx + 1];
            firings.distance[block_idx * 32 + scan_fir_idx]
                    = static_cast<double>(raw_distance.distance)
                    * DISTANCE_RESOLUTION * distance_unit_;

            // intensity
            firings.intensity[block_idx * 32 + scan_fir_idx]
                    = static_cast<double>(raw_block.data[byte_idx + 2]);
        }
    }
    return;
}

bool LslidarCXV4Parser::checkPacketValidity(const RawPacket_C32 *packet) {
    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
        if (packet->blocks[blk_idx].header != UPPER_BANK) {
            return false;
        }
    }
    return true;
}

void LslidarCXV4Parser::GeneratePointcloud(
        const std::shared_ptr<LslidarScan> &scan_msg,
        const std::shared_ptr<PointCloud> &out_msg) {
    // allocate a point cloud with same time and frame ID as raw data
    out_msg->mutable_header()->set_timestamp_sec(
            scan_msg->basetime() / 1000000000.0);
    out_msg->mutable_header()->set_module_name(
            scan_msg->header().module_name());
    out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
    out_msg->set_height(1);
    out_msg->set_measurement_time(scan_msg->basetime() / 1000000000.0);
    out_msg->mutable_header()->set_sequence_num(
            scan_msg->header().sequence_num());
    gps_base_usec_ = scan_msg->basetime();

    const unsigned char *difop_ptr
            = (const unsigned char *)scan_msg->difop_pkts(0).data().c_str();
    if (difop_ptr[0] == 0xa5 && difop_ptr[1] == 0xff && difop_ptr[2] == 0x00
        && difop_ptr[3] == 0x5a) {
        AINFO << "设备包，暂时用不上...";  // todo 暂时不用设备包
    }

    size_t packets_size = scan_msg->firing_pkts_size();
    block_num = 0;
    packet_number_ = packets_size;

    AINFO << "packets_size :" << packets_size;

    for (size_t i = 0; i < packets_size; ++i) {
        Unpack(scan_msg->firing_pkts(static_cast<int>(i)), out_msg, i);
        last_time_stamp_ = out_msg->measurement_time();
        ADEBUG << "stamp: " << std::fixed << last_time_stamp_;
    }

    if (out_msg->point().empty()) {
        // we discard this pointcloud if empty
        AERROR << "All points is NAN!Please check lslidar:" << config_.model();
    }

    // set default width
    out_msg->set_width(out_msg->point_size());
}

/** @brief convert raw packet to point cloud
 *  @param pkt raw packet to Unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void LslidarCXV4Parser::Unpack(
        const LslidarPacket &pkt,
        std::shared_ptr<PointCloud> pc,
        int packet_number) {
    double point_time;
    const RawPacket_C32 *raw_packet
            = (const RawPacket_C32 *)(pkt.data().c_str());
    uint8_t *data = reinterpret_cast<uint8_t *>(
            const_cast<char *>(pkt.data().c_str()));

    // check if the packet is valid
    if (!checkPacketValidity(raw_packet))
        return;

    // decode the packet
    decodePacket(raw_packet);

    if (!is_get_scan_altitude_
        && static_cast<uint16_t>(data[1211]) == 0x46) {  // old C32w
        AERROR << "byte[1211] == 0x46: old C32W";
        is_new_c32w_ = false;
        for (int k = 0; k < 32; ++k) {
            sin_scan_altitude[k] = sin(c32_70_vertical_angle[k] * DEG_TO_RAD);
            cos_scan_altitude[k] = cos(c32_70_vertical_angle[k] * DEG_TO_RAD);
        }
        is_get_scan_altitude_ = true;
    }

    for (size_t fir_idx = 0; fir_idx < SCANS_PER_PACKET; ++fir_idx) {
        if (!is_new_c32w_ && vertical_angle_ == 70) {
            if (fir_idx % 32 == 29 || fir_idx % 32 == 6 || fir_idx % 32 == 14
                || fir_idx % 32 == 22 || fir_idx % 32 == 30 || fir_idx % 32 == 7
                || fir_idx % 32 == 15 || fir_idx % 32 == 23) {
                firings.azimuth[fir_idx] += 389;
            }
            if (firings.azimuth[fir_idx] > 36000)
                firings.azimuth[fir_idx] -= 36000;
        }
        // convert the point to xyz coordinate
        size_t table_idx = firings.azimuth[fir_idx];
        double cos_azimuth = cos_azimuth_table[table_idx];
        double sin_azimuth = sin_azimuth_table[table_idx];
        double x_coord, y_coord, z_coord;
        bool coordinate_opt = true;
        double R1 = 0.0;
        if (vertical_angle_ == 90) {  // ch32r
            R1 = CX4_R1_90;
        } else if (is_new_c32w_) {  // new C32w
            R1 = CX4_C32W;
        } else {  // others
            R1 = CX4_R1_;
        }
        double conversionAngle = (vertical_angle_ == 90)
                ? CX4_conversionAngle_90
                : CX4_conversionAngle_;
        if (coordinate_opt) {
            x_coord = firings.distance[fir_idx]
                            * cos_scan_altitude[fir_idx % lidar_number_]
                            * cos_azimuth
                    + R1
                            * cos((conversionAngle
                                   - firings.azimuth[fir_idx] * 0.01)
                                  * DEG_TO_RAD);
            y_coord = -firings.distance[fir_idx]
                            * cos_scan_altitude[fir_idx % lidar_number_]
                            * sin_azimuth
                    + R1
                            * sin((conversionAngle
                                   - firings.azimuth[fir_idx] * 0.01)
                                  * DEG_TO_RAD);
            z_coord = firings.distance[fir_idx]
                    * sin_scan_altitude[fir_idx % lidar_number_];

        } else {
            // Y-axis correspondence 0 degree
            x_coord = firings.distance[fir_idx]
                            * cos_scan_altitude[fir_idx % lidar_number_]
                            * sin_azimuth
                    + R1
                            * sin((firings.azimuth[fir_idx] * 0.01
                                   - conversionAngle)
                                  * DEG_TO_RAD);
            y_coord = firings.distance[fir_idx]
                            * cos_scan_altitude[fir_idx % lidar_number_]
                            * cos_azimuth
                    + R1
                            * cos((firings.azimuth[fir_idx] * 0.01
                                   - conversionAngle)
                                  * DEG_TO_RAD);
            z_coord = firings.distance[fir_idx]
                    * sin_scan_altitude[fir_idx % lidar_number_];
        }

        // computer the time of the point
        if (last_packet_time > 1e-6) {
            point_time = current_packet_time
                    - (current_packet_time - previous_packet_time)
                            * (SCANS_PER_PACKET - fir_idx - 1)
                            / SCANS_PER_PACKET;
        } else {
            point_time = current_packet_time;
        }

        if (firings.distance[fir_idx] > config_.max_range()
            || firings.distance[fir_idx] < config_.min_range()) {
            PointXYZIT *point = pc->add_point();
            point->set_x(nan);
            point->set_y(nan);
            point->set_z(nan);
            point->set_timestamp(point_time);
            point->set_intensity(0);
        } else {
            if ((firings.azimuth[fir_idx] < config_.scan_start_angle())
                || (firings.azimuth[fir_idx] > config_.scan_end_angle()))
                continue;
            if (packet_number == 0) {
                if (firings.azimuth[fir_idx] > 30000) {
                    continue;
                }
            }

            if (packet_number == packet_number_ - 1) {
                if (firings.azimuth[fir_idx] < 10000) {
                    continue;
                }
            }

            PointXYZIT *point = pc->add_point();
            point->set_timestamp(point_time);

            point->set_intensity(firings.intensity[fir_idx]);
            if ((y_coord >= config_.bottom_left_x()
                 && y_coord <= config_.top_right_x())
                && (-x_coord >= config_.bottom_left_y()
                    && -x_coord <= config_.top_right_y())) {
                point->set_x(nan);
                point->set_y(nan);
                point->set_z(nan);
                point->set_timestamp(point_time);
                point->set_intensity(0);
            } else {
                point->set_x(y_coord);
                point->set_y(-x_coord);
                point->set_z(z_coord);
            }
        }
    }
    previous_packet_time = current_packet_time;
}

void LslidarCXV4Parser::Order(std::shared_ptr<PointCloud> cloud) {
    int width = 32;
    cloud->set_width(width);
    int height = cloud->point_size() / cloud->width();
    cloud->set_height(height);

    std::shared_ptr<PointCloud> cloud_origin = std::make_shared<PointCloud>();
    cloud_origin->CopyFrom(*cloud);

    for (int i = 0; i < width; ++i) {
        int col = ORDER_32[i];

        for (int j = 0; j < height; ++j) {
            // make sure offset is initialized, should be init at setup() just
            // once
            int target_index = j * width + i;
            int origin_index = j * width + col;
            cloud->mutable_point(target_index)
                    ->CopyFrom(cloud_origin->point(origin_index));
        }
    }
}

}  // namespace parser
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
