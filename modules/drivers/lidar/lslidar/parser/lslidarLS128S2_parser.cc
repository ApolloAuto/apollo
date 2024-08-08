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
static float g_fDistanceAcc = 0.1 * 0.01;
static double cos30 = std::cos(DEG2RAD(30));
static double sin30 = std::sin(DEG2RAD(30));
static double sin60 = std::sin(DEG2RAD(60));

LslidarLS128S2Parser::LslidarLS128S2Parser(const Config &config) :
        LslidarParser(config), previous_packet_stamp_(0), gps_base_usec_(0) {
    // create the sin and cos table for different azimuth and vertical values
    for (int j = 0; j < 36000; ++j) {
        double angle = static_cast<double>(j) / 100.0 * M_PI / 180.0;
        sin_table[j] = sin(angle);
        cos_table[j] = cos(angle);
    }

    double mirror_angle[4]
            = {0, -2, -1, -3};  // 摆镜角度   //根据通道不同偏移角度不同
    for (int i = 0; i < 4; ++i) {
        cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
        sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
    }
    cur_pc.reset(new PointCloud());
    pre_pc.reset(new PointCloud());
}

void LslidarLS128S2Parser::GeneratePointcloud(
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

    frame_count++;
    const unsigned char *difop_ptr
            = (const unsigned char *)scan_msg->difop_pkts(0).data().c_str();

    if (difop_ptr[0] == 0x00 || difop_ptr[0] == 0xa5) {
        if (difop_ptr[1] == 0xff && difop_ptr[2] == 0x00
            && difop_ptr[3] == 0x5a) {
            if (difop_ptr[231] == 64 || difop_ptr[231] == 65) {
                is_add_frame_ = true;
            }

            int majorVersion = difop_ptr[1202];
            int minorVersion1 = difop_ptr[1203] / 16;

            // v1.1 :0.01   //v1.2以后  ： 0.0025
            if (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) {
                g_fAngleAcc_V = 0.0025;
            } else {
                g_fAngleAcc_V = 0.01;
            }

            float fInitAngle_V = difop_ptr[188] * 256 + difop_ptr[189];
            if (fInitAngle_V > 32767) {
                fInitAngle_V = fInitAngle_V - 65536;
            }
            this->prism_angle[0] = fInitAngle_V * g_fAngleAcc_V;

            fInitAngle_V = difop_ptr[190] * 256 + difop_ptr[191];
            if (fInitAngle_V > 32767) {
                fInitAngle_V = fInitAngle_V - 65536;
            }
            this->prism_angle[1] = fInitAngle_V * g_fAngleAcc_V;

            fInitAngle_V = difop_ptr[192] * 256 + difop_ptr[193];
            if (fInitAngle_V > 32767) {
                fInitAngle_V = fInitAngle_V - 65536;
            }
            this->prism_angle[2] = fInitAngle_V * g_fAngleAcc_V;

            fInitAngle_V = difop_ptr[194] * 256 + difop_ptr[195];
            if (fInitAngle_V > 32767) {
                fInitAngle_V = fInitAngle_V - 65536;
            }
            this->prism_angle[3] = fInitAngle_V * g_fAngleAcc_V;
        }
    }

    packets_size = scan_msg->firing_pkts_size();
    packet_number_ = packets_size;

    for (size_t i = 0; i < packets_size; ++i) {
        Unpack(static_cast<int>(i),
               scan_msg->firing_pkts(static_cast<int>(i)),
               out_msg);
    }

    if (is_add_frame_) {
        if (frame_count >= 2) {
            // out_msg = std::move(cur_pc);
            for (int j = 0; j < cur_pc->point_size(); ++j) {
                PointXYZIT *point3 = out_msg->add_point();
                point3->set_timestamp(cur_pc->point(j).timestamp());
                point3->set_intensity(cur_pc->point(j).intensity());
                point3->set_x(cur_pc->point(j).x());
                point3->set_y(cur_pc->point(j).y());
                point3->set_z(cur_pc->point(j).z());
            }
        }
        cur_pc = pre_pc;
        pre_pc.reset(new PointCloud());
    } else {
        //                    out_msg = cur_pc;
        for (int j = 0; j < cur_pc->point_size(); ++j) {
            PointXYZIT *point3 = out_msg->add_point();
            point3->set_timestamp(cur_pc->point(j).timestamp());
            point3->set_intensity(cur_pc->point(j).intensity());
            point3->set_x(cur_pc->point(j).x());
            point3->set_y(cur_pc->point(j).y());
            point3->set_z(cur_pc->point(j).z());
        }
        cur_pc.reset(new PointCloud());
        pre_pc.reset(new PointCloud());
    }
    AINFO << "line: " << __LINE__ << "out_msg size: " << out_msg->point_size();
    AINFO << "packets_size :" << packets_size;
    if (out_msg->point().empty()) {
        // we discard this pointcloud if empty
        AERROR << "All points is NAN!Please check lslidar:" << config_.model();
    }

    // set default width
    out_msg->set_width(out_msg->point_size());
    out_msg->set_height(1);
}

/** @brief convert raw packet to point cloud
 *  @param pkt raw packet to Unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void LslidarLS128S2Parser::Unpack(
        int num,
        const LslidarPacket &pkt,
        std::shared_ptr<PointCloud> out_msg) {
    struct Firing_LS128S2 lidardata {};
    uint64_t packet_end_time;
    const unsigned char *msop_ptr = (const unsigned char *)pkt.data().c_str();

    packet_end_time = pkt.stamp();
    current_packet_time = packet_end_time;
    if (msop_ptr[1205] == 0x02) {
        return_mode = 2;
    }

    if (return_mode == 1) {
        double packet_interval_time = (current_packet_time - last_packet_time)
                / (POINTS_PER_PACKET_SINGLE_ECHO / 8.0);
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO;
             point_idx += 8) {
            if ((msop_ptr[point_idx] == 0xff)
                && (msop_ptr[point_idx + 1] == 0xaa)
                && (msop_ptr[point_idx + 2] == 0xbb)
                && (msop_ptr[point_idx + 3] == 0xcc)
                && (msop_ptr[point_idx + 4] == 0xdd)) {
                continue;
            } else {
                // Compute the time of the point
                double point_time;
                if (last_packet_time > 1e-6) {
                    point_time = packet_end_time
                            - packet_interval_time
                                    * ((POINTS_PER_PACKET_SINGLE_ECHO
                                        - point_idx)
                                               / 8
                                       - 1);
                } else {
                    point_time = current_packet_time;
                }

                memset(&lidardata, 0, sizeof(lidardata));
                // 水平角度
                double fAngle_H
                        = msop_ptr[point_idx + 1] + (msop_ptr[point_idx] << 8);
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                lidardata.azimuth = fAngle_H * 0.01;
                // 垂直角度+通道号
                int iTempAngle = msop_ptr[point_idx + 2];
                int iChannelNumber = iTempAngle >> 6;  // 左移六位 通道号
                int iSymmbol = (iTempAngle >> 5) & 0x01;  // 左移五位 符号位
                double fAngle_V = 0.0;
                if (1 == iSymmbol) {  // 符号位 0：正数 1：负数
                    int iAngle_V = msop_ptr[point_idx + 3]
                            + (msop_ptr[point_idx + 2] << 8);

                    fAngle_V = iAngle_V | 0xc000;
                    if (fAngle_V > 32767) {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } else {
                    int iAngle_Hight = iTempAngle & 0x3f;
                    fAngle_V = msop_ptr[point_idx + 3] + (iAngle_Hight << 8);
                }

                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance
                        = ((msop_ptr[point_idx + 4] << 16)
                           + (msop_ptr[point_idx + 5] << 8)
                           + msop_ptr[point_idx + 6]);
                lidardata.intensity = msop_ptr[point_idx + 7];
                lidardata.time = point_time;
                lidardata.azimuth = fAngle_H * 0.01;
                convertCoordinate(lidardata);
            }
        }
    } else {
        double packet_interval_time = (current_packet_time - last_packet_time)
                / (POINTS_PER_PACKET_DOUBLE_ECHO / 12.0);
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO;
             point_idx += 12) {
            if ((msop_ptr[point_idx] == 0xff)
                && (msop_ptr[point_idx + 1] == 0xaa)
                && (msop_ptr[point_idx + 2] == 0xbb)
                && (msop_ptr[point_idx + 3] == 0xcc)
                && (msop_ptr[point_idx + 4] == 0xdd)) {
                continue;
            } else {
                // Compute the time of the point
                double point_time;
                if (last_packet_time > 1e-6) {
                    point_time = packet_end_time
                            - packet_interval_time
                                    * ((POINTS_PER_PACKET_DOUBLE_ECHO
                                        - point_idx)
                                               / 12
                                       - 1);
                } else {
                    point_time = current_packet_time;
                }
                memset(&lidardata, 0, sizeof(lidardata));
                // 水平角度
                double fAngle_H
                        = msop_ptr[point_idx + 1] + (msop_ptr[point_idx] << 8);
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                lidardata.azimuth = fAngle_H * 0.01;

                // 垂直角度+通道号
                int iTempAngle = msop_ptr[point_idx + 2];
                int iChannelNumber = iTempAngle >> 6;  // 左移六位 通道号
                int iSymmbol = (iTempAngle >> 5) & 0x01;  // 左移五位 符号位
                double fAngle_V = 0.0;
                if (1 == iSymmbol) {  // 符号位 0：正数 1：负数
                    int iAngle_V = msop_ptr[point_idx + 3]
                            + (msop_ptr[point_idx + 2] << 8);

                    fAngle_V = iAngle_V | 0xc000;
                    if (fAngle_V > 32767) {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } else {
                    int iAngle_Hight = iTempAngle & 0x3f;
                    fAngle_V = msop_ptr[point_idx + 3] + (iAngle_Hight << 8);
                }

                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance
                        = ((msop_ptr[point_idx + 4] << 16)
                           + (msop_ptr[point_idx + 5] << 8)
                           + msop_ptr[point_idx + 6]);
                lidardata.intensity = msop_ptr[point_idx + 7];
                lidardata.time = point_time;
                convertCoordinate(lidardata);  // 第一个点

                lidardata.distance
                        = ((msop_ptr[point_idx + 8] << 16)
                           + (msop_ptr[point_idx + 9] << 8)
                           + msop_ptr[point_idx + 10]);
                lidardata.intensity = msop_ptr[point_idx + 11];
                lidardata.time = point_time;
                convertCoordinate(lidardata);  // 第二个点
            }
        }
        last_packet_time = packet_end_time;
    }
}

int LslidarLS128S2Parser::convertCoordinate(
        const struct Firing_LS128S2 &lidardata) {
    if (lidardata.distance * g_fDistanceAcc > config_.max_range()
        || lidardata.distance * g_fDistanceAcc < config_.min_range()) {
        return -1;
    }

    if ((lidardata.azimuth < config_.scan_start_angle())
        || (lidardata.azimuth > config_.scan_end_angle())) {
        return -1;
    }

    double fAngle_H = 0.0;  // 水平角度
    double fAngle_V = 0.0;  // 垂直角度
    fAngle_H = lidardata.azimuth;
    fAngle_V = lidardata.vertical_angle;

    // 加畸变
    double fSinV_angle = 0;
    double fCosV_angle = 0;

    // 振镜偏移角度 = 实际垂直角度 / 2  - 偏移值
    double fGalvanometrtAngle = 0;
    fGalvanometrtAngle = fAngle_V + 7.26;

    while (fGalvanometrtAngle < 0.0) {
        fGalvanometrtAngle += 360.0;
    }
    while (fAngle_H < 0.0) {
        fAngle_H += 360.0;
    }

    int table_index_V = static_cast<int>(fGalvanometrtAngle * 100) % 36000;
    int table_index_H = static_cast<int>(fAngle_H * 100) % 36000;

    double fAngle_R0 = cos30 * cos_mirror_angle[lidardata.channel_number % 4]
                    * cos_table[table_index_V]
            - sin_table[table_index_V]
                    * sin_mirror_angle[lidardata.channel_number % 4];

    fSinV_angle = 2 * fAngle_R0 * sin_table[table_index_V]
            + sin_mirror_angle[lidardata.channel_number % 4];
    fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

    double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * sin30
                       - cos_mirror_angle[lidardata.channel_number % 4] * sin60)
            / fCosV_angle;
    double fCosCite = sqrt(1 - pow(fSinCite, 2));

    double fSinCite_H = sin_table[table_index_H] * fCosCite
            + cos_table[table_index_H] * fSinCite;
    double fCosCite_H = cos_table[table_index_H] * fCosCite
            - sin_table[table_index_H] * fSinCite;

    double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
    x_coord = (lidardata.distance * fCosV_angle * fSinCite_H) * g_fDistanceAcc;
    y_coord = (lidardata.distance * fCosV_angle * fCosCite_H) * g_fDistanceAcc;
    z_coord = (lidardata.distance * fSinV_angle) * g_fDistanceAcc;

    if ((y_coord >= config_.bottom_left_x() && y_coord <= config_.top_right_x())
        && (-x_coord >= config_.bottom_left_y()
            && -x_coord <= config_.top_right_y()))
        return -1;

    PointXYZIT *point1 = cur_pc->add_point();
    point1->set_timestamp(lidardata.time);
    point1->set_intensity(lidardata.intensity);
    point1->set_x(y_coord);
    point1->set_y(-x_coord);
    point1->set_z(z_coord);

    PointXYZIT *point2 = pre_pc->add_point();
    point2->set_timestamp(lidardata.time);
    point2->set_intensity(lidardata.intensity);
    point2->set_x(y_coord);
    point2->set_y(-x_coord);
    point2->set_z(z_coord);
    return 0;
}

int LslidarLS128S2Parser::convertCoordinate(
        const struct Firing_LS128S2 &lidardata,
        std::shared_ptr<PointCloud> out_cloud) {
    if (lidardata.distance * g_fDistanceAcc > config_.max_range()
        || lidardata.distance * g_fDistanceAcc < config_.min_range()) {
        return -1;
    }

    if ((lidardata.azimuth < config_.scan_start_angle())
        || (lidardata.azimuth > config_.scan_end_angle())) {
        return -1;
    }

    double fAngle_H = 0.0;  // 水平角度
    double fAngle_V = 0.0;  // 垂直角度
    fAngle_H = lidardata.azimuth;
    fAngle_V = lidardata.vertical_angle;

    // 加畸变
    double fSinV_angle = 0;
    double fCosV_angle = 0;

    // 振镜偏移角度 = 实际垂直角度 / 2  - 偏移值
    double fGalvanometrtAngle = 0;
    fGalvanometrtAngle = fAngle_V + 7.26;

    while (fGalvanometrtAngle < 0.0) {
        fGalvanometrtAngle += 360.0;
    }
    while (fAngle_H < 0.0) {
        fAngle_H += 360.0;
    }

    int table_index_V = static_cast<int>(fGalvanometrtAngle * 100) % 36000;
    int table_index_H = static_cast<int>(fAngle_H * 100) % 36000;

    double fAngle_R0 = cos30 * cos_mirror_angle[lidardata.channel_number % 4]
                    * cos_table[table_index_V]
            - sin_table[table_index_V]
                    * sin_mirror_angle[lidardata.channel_number % 4];

    fSinV_angle = 2 * fAngle_R0 * sin_table[table_index_V]
            + sin_mirror_angle[lidardata.channel_number % 4];
    fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

    double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * sin30
                       - cos_mirror_angle[lidardata.channel_number % 4] * sin60)
            / fCosV_angle;
    double fCosCite = sqrt(1 - pow(fSinCite, 2));

    double fSinCite_H = sin_table[table_index_H] * fCosCite
            + cos_table[table_index_H] * fSinCite;
    double fCosCite_H = cos_table[table_index_H] * fCosCite
            - sin_table[table_index_H] * fSinCite;

    double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
    x_coord = (lidardata.distance * fCosV_angle * fSinCite_H) * g_fDistanceAcc;
    y_coord = (lidardata.distance * fCosV_angle * fCosCite_H) * g_fDistanceAcc;
    z_coord = (lidardata.distance * fSinV_angle) * g_fDistanceAcc;

    if ((y_coord >= config_.bottom_left_x() && y_coord <= config_.top_right_x())
        && (-x_coord >= config_.bottom_left_y()
            && -x_coord <= config_.top_right_y()))
        return -1;

    PointXYZIT *point1 = cur_pc->add_point();
    point1->set_timestamp(lidardata.time);
    point1->set_intensity(lidardata.intensity);
    point1->set_x(y_coord);
    point1->set_y(-x_coord);
    point1->set_z(z_coord);

    PointXYZIT *point2 = pre_pc->add_point();
    point2->set_timestamp(lidardata.time);
    point2->set_intensity(lidardata.intensity);
    point2->set_x(y_coord);
    point2->set_y(-x_coord);
    point2->set_z(z_coord);

    PointXYZIT *point3 = out_cloud->add_point();
    point3->set_timestamp(lidardata.time);
    point3->set_intensity(lidardata.intensity);
    point3->set_x(y_coord);
    point3->set_y(-x_coord);
    point3->set_z(z_coord);
    return 0;
}

void LslidarLS128S2Parser::Order(std::shared_ptr<PointCloud> cloud) {}

}  // namespace parser
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
