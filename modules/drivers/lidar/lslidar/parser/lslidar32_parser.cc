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

Lslidar32Parser::Lslidar32Parser(const Config& config) :
        LslidarParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
    adjust_angle = 0;
    adjust_angle_two = 0;
    adjust_angle_three = 0;
    adjust_angle_four = 0;
    read_difop_ = true;
    config_vert_angle = false;
    lslidar_type = 2;
}

void Lslidar32Parser::GeneratePointcloud(
        const std::shared_ptr<LslidarScan>& scan_msg,
        const std::shared_ptr<PointCloud>& out_msg) {
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

    double scan_altitude_original_degree33[32];
    double scan_altitude_original_degree1[32];
    int startpos = 0;

    const unsigned char* difop_ptr
            = (const unsigned char*)scan_msg->difop_pkts(0).data().c_str();
    if (difop_ptr[0] == 0xa5 && difop_ptr[1] == 0xff && difop_ptr[2] == 0x00
        && difop_ptr[3] == 0x5a) {
        if ((difop_ptr[1202] >= 0x02 && difop_ptr[1203] >= 0x80)
            || difop_ptr[1202] == 0x03) {
            lslidar_type = 3;
            startpos = 245;
            // Horizontal correction Angle
            adjust_angle
                    = (difop_ptr[186] * 256
                       + difop_ptr[187]);  // Angle correction A1
            if (adjust_angle > 32767) {
                adjust_angle = adjust_angle - 65535;
            }
            adjust_angle_two
                    = (difop_ptr[190] * 256
                       + difop_ptr[191]);  // Angle correction A2
            if (adjust_angle_two > 32767) {
                adjust_angle_two = adjust_angle_two - 65535;
            }
            adjust_angle_three
                    = (difop_ptr[188] * 256
                       + difop_ptr[189]);  // Angle correction A3
            if (adjust_angle_three > 32767) {
                adjust_angle_three = adjust_angle_three - 65535;
            }
            adjust_angle_four
                    = (difop_ptr[192] * 256
                       + difop_ptr[193]);  // Angle correction A4
            if (adjust_angle_four > 32767) {
                adjust_angle_four = adjust_angle_four - 65535;
            }
            memcpy(scan_altitude_original_degree1,
                   scan_altitude_original_A3,
                   32 * 8);
            memcpy(scan_altitude_original_degree33,
                   scan_altitude_original_C3,
                   32 * 8);

            if (difop_ptr[185] == 0 || difop_ptr[185] == 1) {
                int return_mode = difop_ptr[185] + 1;
                config_.set_return_mode(return_mode);

                if (difop_ptr[1195] == 0x21)
                    config_.set_degree_mode(2);
                else
                    config_.set_degree_mode(1);

                config_.set_distance_unit(0.4f);

                for (int i = 0; i < LSC32_SCANS_PER_FIRING; i++) {
                    // 均匀1度校准两列
                    if (1 == config_.degree_mode()) {
                        cos_scan_altitude_caliration[i] = std::cos(
                                static_cast<float>(
                                        scan_altitude_original_A3[i] * M_PI)
                                / 180.0f);
                        sin_scan_altitude_caliration[i] = std::sin(
                                static_cast<float>(
                                        scan_altitude_original_A3[i] * M_PI)
                                / 180.0f);
                        scan_altitude_A[i] = scan_altitude_original_A3[i];
                    }
                    // 0.33度校准四列
                    if (2 == config_.degree_mode()) {
                        cos_scan_altitude_caliration[i] = std::cos(
                                static_cast<float>(
                                        scan_altitude_original_C3[i] * M_PI)
                                / 180.0f);
                        sin_scan_altitude_caliration[i] = std::sin(
                                static_cast<float>(
                                        scan_altitude_original_C3[i] * M_PI)
                                / 180.0f);
                        scan_altitude_C[i] = scan_altitude_original_C3[i];
                    }
                }
            }
        } else {
            lslidar_type = 2;
            startpos = 882;
            // Horizontal correction Angle
            adjust_angle
                    = (difop_ptr[34] * 256
                       + difop_ptr[35]); /*Angle correction A1*/
            if (adjust_angle > 32767) {
                adjust_angle = adjust_angle - 65535;
            }
            adjust_angle_two
                    = (difop_ptr[42] * 256
                       + difop_ptr[43]); /*Angle correction A2*/
            if (adjust_angle_two > 32767) {
                adjust_angle_two = adjust_angle_two - 65535;
            }
            adjust_angle_three
                    = (difop_ptr[66] * 256
                       + difop_ptr[67]); /*Angle correction A3*/
            if (adjust_angle_three > 32767) {
                adjust_angle_three = adjust_angle_three - 65535;
            }
            adjust_angle_four
                    = (difop_ptr[68] * 256
                       + difop_ptr[69]); /*Angle correction A4*/
            if (adjust_angle_four > 32767) {
                adjust_angle_four = adjust_angle_four - 65535;
            }
            AWARN << "Load config failed, config file";
            // Vertical Angle Calibration for device package
            for (int i = 0; i < LSC32_SCANS_PER_FIRING; i++) {
                // 均匀1度校准两列
                if (1 == config_.degree_mode()) {
                    cos_scan_altitude_caliration[i] = std::cos(
                            static_cast<float>(
                                    scan_altitude_original_A[i] * M_PI)
                            / 180.0f);
                    sin_scan_altitude_caliration[i] = std::sin(
                            static_cast<float>(
                                    scan_altitude_original_A[i] * M_PI)
                            / 180.0f);
                    scan_altitude_A[i] = scan_altitude_original_A[i];
                }
                // 0.33度校准四列
                if (2 == config_.degree_mode()) {
                    cos_scan_altitude_caliration[i] = std::cos(
                            static_cast<float>(
                                    scan_altitude_original_C[i] * M_PI)
                            / 180.0f);
                    sin_scan_altitude_caliration[i] = std::sin(
                            static_cast<float>(
                                    scan_altitude_original_C[i] * M_PI)
                            / 180.0f);
                    scan_altitude_C[i] = scan_altitude_original_C[i];
                }
            }
            memcpy(scan_altitude_original_degree1,
                   scan_altitude_original_A,
                   32 * 8);
            memcpy(scan_altitude_original_degree33,
                   scan_altitude_original_C,
                   32 * 8);
        }
    }

    // Vertical Angle parse
    if (config_.config_vert()) {
        for (int i = 0; i < LSC32_SCANS_PER_FIRING; i++) {
            uint8_t data1 = difop_ptr[startpos + 2 * i];
            uint8_t data2 = difop_ptr[startpos + 2 * i + 1];
            int vert_angle = data1 * 256 + data2;
            if (vert_angle > 32767) {
                vert_angle = vert_angle - 65535;
            }
            // 均匀1度校准两列
            if (1 == config_.degree_mode()) {
                scan_altitude_A[i]
                        = static_cast<double>(vert_angle * ROTATION_RESOLUTION);
                if (fabs(scan_altitude_original_degree1[i] - scan_altitude_A[i])
                    > 1.5) {
                    scan_altitude_A[i] = scan_altitude_original_degree1[i];
                }
                config_vert_angle = true;
            }
            // 0.33度校准四列
            if (2 == config_.degree_mode()) {
                scan_altitude_C[i]
                        = static_cast<double>(vert_angle * ROTATION_RESOLUTION);
                if (fabs(scan_altitude_original_degree33[i]
                         - scan_altitude_C[i])
                    > 1.5) {
                    scan_altitude_C[i] = scan_altitude_original_degree33[i];
                }
                config_vert_angle = true;
            }
        }
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
void Lslidar32Parser::Unpack(
        const LslidarPacket& pkt,
        std::shared_ptr<PointCloud> pc,
        int packet_number) {
    float x, y, z;
    float azimuth = 0.0f;
    uint32_t intensity = 0;
    float azimuth_diff = 0.0f;
    float azimuth_corrected_f = 0.0f;
    float azimuth_corrected_offset_f = 0.0f;
    uint64_t point_time;
    uint64_t packet_end_time;

    if (config_vert_angle) {
        for (int i = 0; i < LSC32_SCANS_PER_FIRING; i++) {
            // 均匀1度校准两列
            if (1 == config_.degree_mode()) {
                cos_scan_altitude_caliration[i] = std::cos(
                        static_cast<float>(scan_altitude_A[i] * M_PI) / 180.0f);
                sin_scan_altitude_caliration[i] = std::sin(
                        static_cast<float>(scan_altitude_A[i] * M_PI) / 180.0f);
            }

            // 0.33度校准四列
            if (2 == config_.degree_mode()) {
                cos_scan_altitude_caliration[i] = std::cos(
                        static_cast<float>(scan_altitude_C[i] * M_PI) / 180.0f);
                sin_scan_altitude_caliration[i] = std::sin(
                        static_cast<float>(scan_altitude_C[i] * M_PI) / 180.0f);
            }
        }
        config_vert_angle = false;
    }

    const raw_packet_t* raw = (const raw_packet_t*)pkt.data().c_str();
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
            azimuth_diff
                    = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
            azimuth_diff = azimuth_diff > 36000 ? azimuth_diff - 36000
                                                : azimuth_diff;
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
            azimuth_diff
                    = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
            azimuth_diff = azimuth_diff > 36000 ? azimuth_diff - 36000
                                                : azimuth_diff;
        }

        for (int firing = 0, k = 0; firing < LSC32_FIRINGS_PER_BLOCK;
             ++firing) {
            for (int dsr = 0; dsr < LSC32_SCANS_PER_FIRING;
                 ++dsr, k += RAW_SCAN_SIZE) {
                point_count++;
                /** correct for the laser rotation as a function of timing
                 * during the firings **/
                LaserCorrection& corrections
                        = calibration_.laser_corrections_[dsr];

                azimuth_corrected_f
                        = azimuth + azimuth_diff / LSC32_SCANS_PER_FIRING * dsr;
                azimuth_corrected_f = azimuth_corrected_f < 0
                        ? azimuth_corrected_f + 36000
                        : azimuth_corrected_f;
                azimuth_corrected_f = azimuth_corrected_f > 36000
                        ? azimuth_corrected_f - 36000
                        : azimuth_corrected_f;

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

                if (2 == config_.return_mode()) {
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

                if (distance2 > config_.max_range()
                    || distance2 < config_.min_range()) {
                    PointXYZIT* point = pc->add_point();
                    point->set_x(nan);
                    point->set_y(nan);
                    point->set_z(nan);
                    point->set_timestamp(point_time);
                    point->set_intensity(0);
                } else {
                    // 均匀1度校准两列
                    if (1 == config_.degree_mode()) {
                        double adjust_diff = adjust_angle_two - adjust_angle;
                        if (adjust_diff > 300 && adjust_diff < 460) {
                            // v2.7 calibrtation
                            if (lslidar_type == 3) {
                                if (1 >= dsr % 4) {
                                    azimuth_corrected_f += adjust_angle_two;
                                } else {
                                    azimuth_corrected_f += adjust_angle;
                                }
                            } else {
                                if (0 == dsr % 2) {
                                    azimuth_corrected_f += adjust_angle_two;
                                } else {
                                    azimuth_corrected_f += adjust_angle;
                                }
                            }
                        } else {
                            // v2.6 calibrtation
                            if (0 == dsr % 2) {
                                azimuth_corrected_f += adjust_angle;
                            } else {
                                azimuth_corrected_f -= adjust_angle;
                            }
                        }
                    }

                    // 0.33度校准四列
                    if (2 == config_.degree_mode()) {
                        double adjust_diff_one
                                = adjust_angle_two - adjust_angle;
                        double adjust_diff_two
                                = adjust_angle_four - adjust_angle_three;
                        if (lslidar_type == 3) {
                            // v3.0 calibrtation
                            if (0 == dsr || 1 == dsr || 4 == dsr || 8 == dsr
                                || 9 == dsr || 12 == dsr || 16 == dsr
                                || 17 == dsr || 21 == dsr || 24 == dsr
                                || 25 == dsr || 29 == dsr)
                                azimuth_corrected_f += adjust_angle_four;  // A4

                            if (2 == dsr || 3 == dsr || 6 == dsr || 10 == dsr
                                || 11 == dsr || 14 == dsr || 18 == dsr
                                || 19 == dsr || 23 == dsr || 26 == dsr
                                || 27 == dsr || 31 == dsr)
                                azimuth_corrected_f
                                        += adjust_angle_three;  // A3

                            if (5 == dsr || 13 == dsr || 20 == dsr || 28 == dsr)
                                azimuth_corrected_f += adjust_angle_two;  // A2

                            if (7 == dsr || 15 == dsr || 22 == dsr || 30 == dsr)
                                azimuth_corrected_f += adjust_angle;  // A1
                        } else if (
                                adjust_diff_one > 500 && adjust_diff_one < 660
                                && adjust_diff_two > 150
                                && adjust_diff_two < 350) {
                            // v2.7 calibrtation
                            if (10 == dsr || 14 == dsr || 18 == dsr
                                || 22 == dsr)
                                azimuth_corrected_f += adjust_angle_four;  // A4

                            if (11 == dsr || 15 == dsr || 19 == dsr
                                || 23 == dsr)
                                azimuth_corrected_f
                                        += adjust_angle_three;  // A3

                            if (0 == dsr || 2 == dsr || 4 == dsr || 6 == dsr
                                || 8 == dsr || 12 == dsr || 16 == dsr
                                || 20 == dsr || 24 == dsr || 26 == dsr
                                || 28 == dsr || 30 == dsr)
                                azimuth_corrected_f += adjust_angle_two;  // A2

                            if (1 == dsr || 3 == dsr || 5 == dsr || 7 == dsr
                                || 9 == dsr || 13 == dsr || 17 == dsr
                                || 21 == dsr || 25 == dsr || 27 == dsr
                                || 29 == dsr || 31 == dsr)
                                azimuth_corrected_f += adjust_angle;  // A1
                        } else {
                            // v2.6 calibrtation
                            if (10 == dsr || 14 == dsr || 18 == dsr
                                || 22 == dsr)
                                azimuth_corrected_f += adjust_angle;

                            if (11 == dsr || 15 == dsr || 19 == dsr
                                || 23 == dsr)
                                azimuth_corrected_f -= adjust_angle;

                            if (0 == dsr || 2 == dsr || 4 == dsr || 6 == dsr
                                || 8 == dsr || 12 == dsr || 16 == dsr
                                || 20 == dsr || 24 == dsr || 26 == dsr
                                || 28 == dsr || 30 == dsr)
                                azimuth_corrected_f += adjust_angle_two;

                            if (1 == dsr || 3 == dsr || 5 == dsr || 7 == dsr
                                || 9 == dsr || 13 == dsr || 17 == dsr
                                || 21 == dsr || 25 == dsr || 27 == dsr
                                || 29 == dsr || 31 == dsr)
                                azimuth_corrected_f -= adjust_angle_two;
                        }
                    }

                    azimuth_corrected_f = azimuth_corrected_f < 0
                            ? azimuth_corrected_f + 36000
                            : azimuth_corrected_f;
                    azimuth_corrected_f = azimuth_corrected_f > 36000
                            ? azimuth_corrected_f - 36000
                            : azimuth_corrected_f;

                    if ((azimuth_corrected_f < config_.scan_start_angle())
                        || (azimuth_corrected_f > config_.scan_end_angle()))
                        continue;
                    if (packet_number == 0) {
                        if (azimuth_corrected_f > 30000) {
                            continue;
                        }
                    }

                    if (packet_number == packet_number_ - 1) {
                        if (azimuth_corrected_f < 10000) {
                            continue;
                        }
                    }

                    // 以结构为中心
                    float rotation_azimuth = ROTATION_RESOLUTION
                            * static_cast<float>(azimuth_corrected_f * M_PI)
                            / 180.0f;
                    azimuth_corrected_offset_f
                            = azimuth_corrected_f * ROTATION_RESOLUTION
                            - LSC32_AZIMUTH_TOFFSET;
                    float rotation_azimuth_offset
                            = static_cast<float>(
                                      azimuth_corrected_offset_f * M_PI)
                            / 180.0f;

                    PointXYZIT* point = pc->add_point();
                    point->set_timestamp(point_time);
                    point->set_intensity(intensity);
                    point->set_intensity(intensity);

                    if (config_.calibration()) {
                        ComputeCoords(
                                distance2,
                                &corrections,
                                static_cast<uint16_t>(azimuth_corrected_f),
                                point);
                    } else {
                        x = distance2 * cos_scan_altitude_caliration[dsr]
                                        * cosf(rotation_azimuth)
                                + (LSC32_DISTANCE_TOFFSET
                                   * cosf(rotation_azimuth_offset))
                                        * DISTANCE_RESOLUTION;
                        y = -(distance2 * cos_scan_altitude_caliration[dsr]
                                      * sinf(rotation_azimuth)
                              + (LSC32_DISTANCE_TOFFSET
                                 * sinf(rotation_azimuth_offset))
                                      * DISTANCE_RESOLUTION);
                        z = distance2 * sin_scan_altitude_caliration[dsr];

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

void Lslidar32Parser::Order(std::shared_ptr<PointCloud> cloud) {
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
