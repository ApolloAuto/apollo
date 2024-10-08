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

float R1_ = 0.04376;
float R2_ = 0.010875;

Lslidar16Parser::Lslidar16Parser(const Config &config) :
        LslidarParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
    need_two_pt_correction_ = false;
    config_vert_angle = false;
    previous_packet_time = 0;

    if (2 == config_.degree_mode()) {
        for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
            cos_scan_altitude_caliration[i]
                    = std::cos(scan_altitude_original_2[i]);
            sin_scan_altitude_caliration[i]
                    = std::sin(scan_altitude_original_2[i]);
            scan_altitude[i] = scan_altitude_original_2[i];
        }
    } else if (1 == config_.degree_mode()) {
        for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
            cos_scan_altitude_caliration[i]
                    = std::cos(scan_altitude_original_1[i]);
            sin_scan_altitude_caliration[i]
                    = std::sin(scan_altitude_original_1[i]);
            scan_altitude[i] = scan_altitude_original_1[i];
        }
    }

    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation = ROTATION_RESOLUTION
                * static_cast<float>(rot_index * M_PI) / 180.0f;
        cos_azimuth_table[rot_index] = cosf(rotation);
        sin_azimuth_table[rot_index] = sinf(rotation);
    }
}

void Lslidar16Parser::GeneratePointcloud(
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
    uint8_t version_data = static_cast<uint8_t>(difop_ptr[1202]);

    if (config_.config_vert()) {
        if (2 == version_data) {
            for (int i = 0; i < 16; i++) {
                uint16_t vert_angle = static_cast<uint16_t>(
                        difop_ptr[234 + 2 * i] * 256
                        + difop_ptr[234 + 2 * i + 1]);

                if (vert_angle > 32767) {
                    vert_angle = vert_angle - 65535;
                }

                scan_altitude[i]
                        = (static_cast<float>(vert_angle) / 100.f) * DEG_TO_RAD;
                if (2 == config_.degree_mode()) {
                    if (scan_altitude[i] != 0) {
                        if (fabs(scan_altitude_original_2[i] - scan_altitude[i])
                                    * RAD_TO_DEG
                            > 1.5) {
                            scan_altitude[i] = scan_altitude_original_2[i];
                        }
                    } else {
                        scan_altitude[i] = scan_altitude_original_2[i];
                    }
                } else if (1 == config_.degree_mode()) {
                    if (scan_altitude[i] != 0) {
                        if (fabs(scan_altitude_original_1[i] - scan_altitude[i])
                                    * RAD_TO_DEG
                            > 1.5) {
                            scan_altitude[i] = scan_altitude_original_1[i];
                        }
                    } else {
                        scan_altitude[i] = scan_altitude_original_1[i];
                    }
                }
                config_vert_angle = true;
            }
        } else {
            for (int i = 0; i < 16; i++) {
                uint16_t vert_angle = static_cast<uint16_t>(
                        difop_ptr[245 + 2 * i] * 256
                        + difop_ptr[245 + 2 * i + 1]);

                if (vert_angle > 32767) {
                    vert_angle = vert_angle - 65535;
                }

                scan_altitude[i]
                        = (static_cast<float>(vert_angle) / 100.f) * DEG_TO_RAD;

                if (2 == config_.degree_mode()) {
                    if (scan_altitude[i] != 0) {
                        if (fabs(scan_altitude_original_2[i] - scan_altitude[i])
                                    * RAD_TO_DEG
                            > 1.5) {
                            scan_altitude[i] = scan_altitude_original_2[i];
                        }
                    } else {
                        scan_altitude[i] = scan_altitude_original_2[i];
                    }
                } else if (1 == config_.degree_mode()) {
                    if (scan_altitude[i] != 0) {
                        if (fabs(scan_altitude_original_1[i] - scan_altitude[i])
                                    * RAD_TO_DEG
                            > 1.5) {
                            scan_altitude[i] = scan_altitude_original_1[i];
                        }
                    } else {
                        scan_altitude[i] = scan_altitude_original_1[i];
                    }
                }
                config_vert_angle = true;
            }
        }
    }

    size_t packets_size = scan_msg->firing_pkts_size();
    packet_number_ = packets_size;
    block_num = 0;

    for (size_t i = 0; i < packets_size; ++i) {
        Unpack(scan_msg->firing_pkts(static_cast<int>(i)), out_msg, i);
        last_time_stamp_ = out_msg->measurement_time();
        ADEBUG << "***************stamp: " << std::fixed << last_time_stamp_;
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
void Lslidar16Parser::Unpack(
        const LslidarPacket &pkt,
        std::shared_ptr<PointCloud> pc,
        int packet_number) {
    float x, y, z;
    float azimuth = 0.0f;
    uint32_t intensity = 0;
    float azimuth_diff = 0.0f;
    float azimuth_corrected_f = 0.0f;
    int azimuth_corrected = 0;
    uint64_t point_time;
    uint64_t packet_end_time;

    if (config_vert_angle) {
        for (int i = 0; i < LSC16_SCANS_PER_FIRING; i++) {
            cos_scan_altitude_caliration[i] = std::cos(scan_altitude[i]);
            sin_scan_altitude_caliration[i] = std::sin(scan_altitude[i]);
        }
        config_vert_angle = false;
    }
    const raw_packet_t *raw = (const raw_packet_t *)pkt.data().c_str();

    if (!config_.time_synchronization()) {
        packet_end_time = pkt.stamp();
    } else {
        packet_end_time = pkt.stamp();
    }
    current_packet_time = packet_end_time;
    int point_count = -1;
    for (int block = 0; block < BLOCKS_PER_PACKET; block++, block_num++) {
        if (UPPER_BANK != raw->blocks[block].header)
            break;

        azimuth = static_cast<float>(
                256 * raw->blocks[block].rotation_2
                + raw->blocks[block].rotation_1);

        if (2 == config_.return_mode()) {
            if (block < (BLOCKS_PER_PACKET - 2)) {
                int azi1, azi2;
                azi1 = 256 * raw->blocks[block + 2].rotation_2
                        + raw->blocks[block + 2].rotation_1;
                azi2 = 256 * raw->blocks[block].rotation_2
                        + raw->blocks[block].rotation_1;
                azimuth_diff
                        = static_cast<float>((36000 + azi1 - azi2) % 36000);
            } else {
                int azi1, azi2;
                azi1 = 256 * raw->blocks[block].rotation_2
                        + raw->blocks[block].rotation_1;
                azi2 = 256 * raw->blocks[block - 2].rotation_2
                        + raw->blocks[block - 2].rotation_1;
                azimuth_diff
                        = static_cast<float>((36000 + azi1 - azi2) % 36000);
            }
        } else {
            if (block < (BLOCKS_PER_PACKET - 1)) {
                int azi1, azi2;
                azi1 = 256 * raw->blocks[block + 1].rotation_2
                        + raw->blocks[block + 1].rotation_1;
                azi2 = 256 * raw->blocks[block].rotation_2
                        + raw->blocks[block].rotation_1;
                azimuth_diff
                        = static_cast<float>((36000 + azi1 - azi2) % 36000);
            } else {
                int azi1, azi2;
                azi1 = 256 * raw->blocks[block].rotation_2
                        + raw->blocks[block].rotation_1;
                azi2 = 256 * raw->blocks[block - 1].rotation_2
                        + raw->blocks[block - 1].rotation_1;
                azimuth_diff
                        = static_cast<float>((36000 + azi1 - azi2) % 36000);
            }
        }

        float cos_azimuth;
        float sin_azimuth;

        for (int firing = 0, k = 0; firing < LSC16_FIRINGS_PER_BLOCK;
             ++firing) {
            for (int dsr = 0; dsr < LSC16_SCANS_PER_FIRING;
                 ++dsr, k += RAW_SCAN_SIZE) {
                point_count++;
                LaserCorrection &corrections
                        = calibration_.laser_corrections_[dsr];

                /** correct for the laser rotation as a function of timing
                 * during the firings **/
                azimuth_corrected_f = azimuth
                        + azimuth_diff / (LSC16_SCANS_PER_FIRING * 2)
                                * (LSC16_SCANS_PER_FIRING * firing + dsr);

                azimuth_corrected = static_cast<int>(
                        round(fmod(azimuth_corrected_f, 36000.0)));

                cos_azimuth = cos_azimuth_table[azimuth_corrected];
                sin_azimuth = sin_azimuth_table[azimuth_corrected];

                // distance
                union two_bytes tmp;
                tmp.bytes[0] = raw->blocks[block].data[k];
                tmp.bytes[1] = raw->blocks[block].data[k + 1];
                int distance = tmp.uint;

                // read intensity
                intensity = raw->blocks[block].data[k + 2];

                float distance2 = (distance * DISTANCE_RESOLUTION)
                        * config_.distance_unit();

                if (config_.calibration())
                    distance2 = distance2 + corrections.dist_correction;

                // The offset calibration
                float arg_horiz = static_cast<float>(
                        azimuth_corrected_f * ROTATION_RESOLUTION);
                arg_horiz = arg_horiz > 360 ? (arg_horiz - 360) : arg_horiz;
                float arg_horiz_orginal = (14.67 - arg_horiz) * M_PI / 180;

                if (2 == config_.return_mode()) {
                    // modify
                    if (previous_packet_time > 1e-6) {
                        point_time = current_packet_time
                                - (SCANS_PER_PACKET - point_count - 1)
                                        * (current_packet_time
                                           - previous_packet_time)
                                        / (SCANS_PER_PACKET);
                    } else {  // fist packet
                        point_time = current_packet_time;
                    }

                } else {
                    // modify
                    if (previous_packet_time > 1e-6) {
                        point_time = current_packet_time
                                - (SCANS_PER_PACKET - point_count - 1)
                                        * (current_packet_time
                                           - previous_packet_time)
                                        / (SCANS_PER_PACKET);
                    } else {  // fist packet
                        point_time = current_packet_time;
                    }
                }

                if ((azimuth_corrected < config_.scan_start_angle())
                    || (azimuth_corrected > config_.scan_end_angle()))
                    continue;
                if (packet_number == 0) {
                    if (azimuth_corrected > 30000) {
                        continue;
                    }
                }

                if (packet_number == packet_number_ - 1) {
                    if (azimuth_corrected < 10000) {
                        continue;
                    }
                }

                if (distance2 > config_.max_range()
                    || distance2 < config_.min_range()) {
                    PointXYZIT *point = pc->add_point();
                    point->set_x(nan);
                    point->set_y(nan);
                    point->set_z(nan);
                    point->set_timestamp(point_time);
                    point->set_intensity(0);
                } else {
                    PointXYZIT *point = pc->add_point();
                    point->set_timestamp(point_time);

                    point->set_intensity(intensity);

                    if (config_.calibration()) {
                        ComputeCoords(
                                distance2,
                                &corrections,
                                static_cast<uint16_t>(azimuth_corrected),
                                point);

                    } else {
                        x = distance2 * cos_scan_altitude_caliration[dsr]
                                        * cos_azimuth
                                + R1_ * cos(arg_horiz_orginal);
                        y = -distance2 * cos_scan_altitude_caliration[dsr]
                                        * sin_azimuth
                                + R1_ * sin(arg_horiz_orginal);
                        z = distance2 * sin_scan_altitude_caliration[dsr]
                                + 0.426 / 100.f;

                        if ((y >= config_.bottom_left_x()
                             && y <= config_.top_right_x())
                            && (-x >= config_.bottom_left_y()
                                && -x <= config_.top_right_y())) {
                            point->set_x(nan);
                            point->set_y(nan);
                            point->set_z(nan);
                            point->set_timestamp(point_time);
                            point->set_intensity(0);
                        } else {
                            point->set_x(y);
                            point->set_y(-x);
                            point->set_z(z);
                        }
                    }
                }
            }
        }
    }
    previous_packet_time = current_packet_time;
}

void Lslidar16Parser::Order(std::shared_ptr<PointCloud> cloud) {
    int width = 16;
    cloud->set_width(width);
    int height = cloud->point_size() / cloud->width();
    cloud->set_height(height);

    std::shared_ptr<PointCloud> cloud_origin = std::make_shared<PointCloud>();
    cloud_origin->CopyFrom(*cloud);

    for (int i = 0; i < width; ++i) {
        int col = ORDER_16[i];

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
