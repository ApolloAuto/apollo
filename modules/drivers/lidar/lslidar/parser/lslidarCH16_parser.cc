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

LslidarCH16Parser::LslidarCH16Parser(const Config& config) :
        LslidarParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {}

void LslidarCH16Parser::GeneratePointcloud(
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

    const unsigned char* difop_ptr
            = (const unsigned char*)scan_msg->difop_pkts(0).data().c_str();

    two_bytes angle_1, angle_2, angle_3, angle_4;
    angle_1.bytes[0] = difop_ptr[663];
    angle_1.bytes[1] = difop_ptr[662];
    prism_angle[0] = angle_1.uint * 0.01;

    angle_2.bytes[0] = difop_ptr[665];
    angle_2.bytes[1] = difop_ptr[664];
    prism_angle[1] = angle_2.uint * 0.01;

    angle_3.bytes[0] = difop_ptr[667];
    angle_3.bytes[1] = difop_ptr[666];
    prism_angle[2] = angle_3.uint * 0.01;

    angle_4.bytes[0] = difop_ptr[669];
    angle_4.bytes[1] = difop_ptr[668];
    prism_angle[3] = angle_4.uint * 0.01;
    packets_size = scan_msg->firing_pkts_size();

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
void LslidarCH16Parser::Unpack(
        int num,
        const LslidarPacket& pkt,
        std::shared_ptr<PointCloud> pc) {
    float x, y, z;
    uint64_t packet_end_time;
    double z_sin_altitude = 0.0;
    double z_cos_altitude = 0.0;
    time_last = 0;
    const RawPacket* raw = (const RawPacket*)pkt.data().c_str();

    packet_end_time = pkt.stamp();

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
                / 256.0 * DISTANCE_RESOLUTION;
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

        // Convert the point to xyz coordinate
        double cos_azimuth_half = cos(firings[point_idx].azimuth * 0.5);

        if (abs(prism_angle[0]) < 1e-6 && abs(prism_angle[1]) < 1e-6
            && abs(prism_angle[2]) < 1e-6 && abs(prism_angle[3]) < 1e-6) {
            z_sin_altitude = sin_scan_laser_altitude
                                     [firings[point_idx].vertical_line / 4 + 2]
                    + 2 * cos_azimuth_half
                            * sin_scan_mirror_altitude
                                    [firings[point_idx].vertical_line % 4];
        } else {
            z_sin_altitude = sin_scan_laser_altitude
                                     [firings[point_idx].vertical_line / 4 + 2]
                    + 2 * cos_azimuth_half
                            * sin(prism_angle
                                          [firings[point_idx].vertical_line % 4]
                                  * M_PI / 180);
        }

        z_cos_altitude = sqrt(1 - z_sin_altitude * z_sin_altitude);

        x = firings[point_idx].distance * z_cos_altitude
                * cos(firings[point_idx].azimuth);
        y = firings[point_idx].distance * z_cos_altitude
                * sin(firings[point_idx].azimuth);
        z = firings[point_idx].distance * z_sin_altitude;

        // Compute the time of the point
        uint64_t point_time
                = packet_end_time - 1665 * (POINTS_PER_PACKET - point_idx - 1);
        if (time_last < point_time && time_last > 0) {
            point_time = time_last + 1665;
        }
        time_last = point_time;

        PointXYZIT* point = pc->add_point();
        point->set_timestamp(point_time / 1000000000.0);
        point->set_intensity(firings[point_idx].intensity);

        if (config_.calibration()) {
            ComputeCoords2(
                    firings[point_idx].vertical_line,
                    CH16,
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
}

void LslidarCH16Parser::Order(std::shared_ptr<PointCloud> cloud) {
    int width = 16;
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
