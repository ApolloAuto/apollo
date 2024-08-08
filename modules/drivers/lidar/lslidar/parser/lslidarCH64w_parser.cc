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

LslidarCH64wParser::LslidarCH64wParser(const Config& config) :
        LslidarParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
    point_time_diff = 1 / (60.0 * 1000 * 32);  // s
    for (int i = 0; i < 4; ++i) {
        prism_angle64[i] = i * 0.35;
    }
    for (int j = 0; j < 128; ++j) {
        if (j / 4 % 2 == 0) {
            theat1_s[j] = sin((-25 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_s[j] = sin(prism_angle64[j % 4] * M_PI / 180);
            theat1_c[j] = cos((-25 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_c[j] = cos(prism_angle64[j % 4] * M_PI / 180);
        } else {
            theat1_s[j] = sin((-24 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_s[j] = sin(prism_angle64[j % 4] * M_PI / 180);
            theat1_c[j] = cos((-24 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_c[j] = cos(prism_angle64[j % 4] * M_PI / 180);
        }
    }
}

void LslidarCH64wParser::GeneratePointcloud(
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

    packets_size = scan_msg->firing_pkts_size();
    AINFO << "packets_size :" << packets_size;
    for (size_t i = 0; i < packets_size; ++i) {
        Unpack(static_cast<int>(i),
               scan_msg->firing_pkts(static_cast<int>(i)),
               out_msg);
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
void LslidarCH64wParser::Unpack(
        int num,
        const LslidarPacket& pkt,
        std::shared_ptr<PointCloud> pc) {
    float x, y, z;
    uint64_t point_time;
    uint64_t packet_end_time;
    time_last = 0;
    const RawPacket* raw = (const RawPacket*)pkt.data().c_str();
    double cos_xita;
    // 垂直角度
    double sin_theat;
    double cos_theat;
    double _R_;
    // 水平角度
    double cos_H_xita;
    double sin_H_xita;
    double cos_xita_F;
    double sin_xita_F;

    packet_end_time = pkt.stamp();

    current_packet_time = packet_end_time;

    for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {
        firings[point_idx].vertical_line = raw->points[point_idx].vertical_line;
        two_bytes point_amuzith;
        point_amuzith.bytes[0] = raw->points[point_idx].azimuth_2;
        point_amuzith.bytes[1] = raw->points[point_idx].azimuth_1;
        firings[point_idx].azimuth
                = static_cast<double>(point_amuzith.uint) * 0.01 * DEG_TO_RAD;
        four_bytes point_distance;
        point_distance.bytes[0] = raw->points[point_idx].distance_3;
        point_distance.bytes[1] = raw->points[point_idx].distance_2;
        point_distance.bytes[2] = raw->points[point_idx].distance_1;
        point_distance.bytes[3] = 0;
        firings[point_idx].distance = static_cast<double>(point_distance.uint)
                * DISTANCE_RESOLUTION2;
        firings[point_idx].intensity = raw->points[point_idx].intensity;
    }

    for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {
        LaserCorrection& corrections
                = calibration_
                          .laser_corrections_[firings[point_idx].vertical_line];

        if (config_.calibration())
            firings[point_idx].distance
                    = firings[point_idx].distance + corrections.dist_correction;

        if (firings[point_idx].distance > config_.max_range()
            || firings[point_idx].distance < config_.min_range())
            continue;

        int line_num = firings[point_idx].vertical_line;
        if (line_num / 4 % 2 == 0) {
            cos_xita
                    = cos((firings[point_idx].azimuth * RAD_TO_DEG / 2.0 + 22.5)
                          * M_PI / 180);
        } else {
            cos_xita = cos(
                    (-firings[point_idx].azimuth * RAD_TO_DEG / 2.0 + 112.5)
                    * M_PI / 180);
        }

        _R_ = theat2_c[line_num] * theat1_c[line_num] * cos_xita
                - theat2_s[line_num] * theat1_s[line_num];
        sin_theat = theat1_s[line_num] + 2 * _R_ * theat2_s[line_num];
        cos_theat = sqrt(1 - pow(sin_theat, 2));
        cos_H_xita
                = (2 * _R_ * theat2_c[line_num] * cos_xita - theat1_c[line_num])
                / cos_theat;
        sin_H_xita = sqrt(1 - pow(cos_H_xita, 2));

        if (line_num / 4 % 2 == 0) {
            cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt(0.5);
            sin_xita_F = sqrt(1 - pow(cos_xita_F, 2));
        } else {
            cos_xita_F = (cos_H_xita + sin_H_xita) * (-sqrt(0.5));
            sin_xita_F = sqrt(1 - pow(cos_xita_F, 2));
        }

        x = firings[point_idx].distance * cos_theat * cos_xita_F;
        y = firings[point_idx].distance * cos_theat * sin_xita_F;
        z = firings[point_idx].distance * sin_theat;

        if ((y >= config_.bottom_left_x() && y <= config_.top_right_x())
            && (-x >= config_.bottom_left_y() && -x <= config_.top_right_y()))
            return;
        // modify
        if (previous_packet_time > 1e-6) {
            point_time = current_packet_time
                    - (POINTS_PER_PACKET - point_idx - 1)
                            * (current_packet_time - previous_packet_time)
                            / (POINTS_PER_PACKET);
        } else {  // fist packet
            point_time = current_packet_time;
        }

        PointXYZIT* point = pc->add_point();
        point->set_timestamp(point_time);
        point->set_intensity(firings[point_idx].intensity);

        if (config_.calibration()) {
            ComputeCoords2(
                    firings[point_idx].vertical_line,
                    CH64w,
                    firings[point_idx].distance,
                    &corrections,
                    firings[point_idx].azimuth,
                    point);

        } else {
            if ((y >= config_.bottom_left_x() && y <= config_.top_right_x())
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
    previous_packet_time = current_packet_time;
}

void LslidarCH64wParser::Order(std::shared_ptr<PointCloud> cloud) {}

}  // namespace parser
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
