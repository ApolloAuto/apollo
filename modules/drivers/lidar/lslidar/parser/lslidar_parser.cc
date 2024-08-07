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

LslidarParser::LslidarParser(const Config &config) :
        last_time_stamp_(0), config_(config) {
    for (int i = 0; i < 4; ++i) {
        prism_angle64[i] = i * 0.35;
    }
    for (int j = 0; j < 128; ++j) {
        // 右边
        if (j / 4 % 2 == 0) {
            theat1_s[j] = sin((-25 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_s[j] = sin(prism_angle64[j % 4] * M_PI / 180);
            theat1_c[j] = cos((-25 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_c[j] = cos(prism_angle64[j % 4] * M_PI / 180);
        } else {  // 左边
            theat1_s[j] = sin((-24 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_s[j] = sin(prism_angle64[j % 4] * M_PI / 180);
            theat1_c[j] = cos((-24 + floor(j / 8) * 2.5) * M_PI / 180);
            theat2_c[j] = cos(prism_angle64[j % 4] * M_PI / 180);
        }
    }
}

bool LslidarParser::is_scan_valid(int rotation, float range) {
    // check range first
    if (range < config_.min_range() || range > config_.max_range())
        return false;
    else
        return true;
}

/** Set up for on-line operation. */
void LslidarParser::setup() {
    if (config_.calibration()) {
        calibration_.read(config_.calibration_file());

        if (!calibration_.initialized_) {
            AFATAL << "Unable to open calibration file: "
                   << config_.calibration_file();
        }
    }
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
        float rotation_f = ROTATION_RESOLUTION
                * static_cast<float>(rot_index * M_PI) / 180.0f;
        cos_azimuth_table[rot_index] = cosf(rotation_f);
        sin_azimuth_table[rot_index] = sinf(rotation_f);
    }
}

void LslidarParser::ComputeCoords(
        const float &raw_distance,
        LaserCorrection *corrections,
        const uint16_t rotation,
        PointXYZIT *point) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double distance_corr_x = 0;
    double distance_corr_y = 0;

    corrections->cos_rot_correction = cosf(corrections->rot_correction);
    corrections->sin_rot_correction = sinf(corrections->rot_correction);
    corrections->cos_vert_correction = cosf(corrections->vert_correction);
    corrections->sin_vert_correction = sinf(corrections->vert_correction);

    double distance = raw_distance + corrections->dist_correction;

    double cos_rot_angle
            = cos_azimuth_table[rotation] * corrections->cos_rot_correction
            + sin_azimuth_table[rotation] * corrections->sin_rot_correction;
    double sin_rot_angle
            = sin_azimuth_table[rotation] * corrections->cos_rot_correction
            - cos_azimuth_table[rotation] * corrections->sin_rot_correction;

    // Compute the distance in the xy plane (w/o accounting for rotation)
    double xy_distance = distance * corrections->cos_vert_correction;

    // Get 2points calibration values,Linear interpolation to get distance
    // correction for X and Y, that means distance correction use
    // different value at different distance
    distance_corr_x = distance_corr_y = corrections->dist_correction;

    double distance_x = raw_distance + distance_corr_x;
    xy_distance = distance_x * corrections->cos_vert_correction;

    x = xy_distance * sin_rot_angle
            - corrections->horiz_offset_correction * cos_rot_angle;

    double distance_y = raw_distance + distance_corr_y;
    xy_distance = distance_y * corrections->cos_vert_correction;

    y = xy_distance * cos_rot_angle
            + corrections->horiz_offset_correction * sin_rot_angle;
    z = distance * corrections->sin_vert_correction
            + corrections->vert_offset_correction;

    /** Use standard ROS coordinate system (right-hand rule) */
    point->set_x(static_cast<float>(-y));
    point->set_y(static_cast<float>(-x));
    point->set_z(static_cast<float>(z));
}

void LslidarParser::ComputeCoords2(
        int Laser_ring,
        int Type,
        const float &raw_distance,
        LaserCorrection *corrections,
        const double rotation,
        PointXYZIT *point) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double distance_corr_x = 0;
    double distance_corr_y = 0;
    double sinTheta_1[128] = {0};
    double cosTheta_1[128] = {0};
    double sinTheta_2[128] = {0};
    double cosTheta_2[128] = {0};
    double cos_xita;
    // 垂直角度
    double sin_theat;
    double cos_theat;
    double _R_;
    // 水平角度
    double cos_H_xita;
    double sin_H_xita;
    double cos_xita_F;

    for (int i = 0; i < 128; i++) {
        sinTheta_1[i] = sin(big_angle[i / 4] * M_PI / 180);
        cosTheta_1[i] = cos(big_angle[i / 4] * M_PI / 180);

        if (abs(prism_angle[0]) < 1e-6 && abs(prism_angle[1]) < 1e-6
            && abs(prism_angle[2]) < 1e-6 && abs(prism_angle[3]) < 1e-6) {
            sinTheta_2[i] = sin((i % 4) * (-0.17) * M_PI / 180);
            cosTheta_2[i] = cos((i % 4) * (-0.17) * M_PI / 180);
        } else {
            sinTheta_2[i] = sin(prism_angle[i % 4] * M_PI / 180);
            cosTheta_2[i] = cos(prism_angle[i % 4] * M_PI / 180);
        }
    }

    corrections->cos_rot_correction = cosf(corrections->rot_correction);
    corrections->sin_rot_correction = sinf(corrections->rot_correction);

    double cos_azimuth_half = cos(rotation * 0.5);

    if (Type == CH16) {
        if (abs(prism_angle[0]) < 1e-6 && abs(prism_angle[1]) < 1e-6
            && abs(prism_angle[2]) < 1e-6 && abs(prism_angle[3]) < 1e-6) {
            corrections->sin_vert_correction
                    = sin_scan_laser_altitude[Laser_ring / 4 + 2]
                    + 2 * cos_azimuth_half
                            * sin_scan_mirror_altitude[Laser_ring % 4];
        } else {
            corrections->sin_vert_correction
                    = sin_scan_laser_altitude[Laser_ring / 4 + 2]
                    + 2 * cos_azimuth_half
                            * sin(prism_angle[Laser_ring % 4] * M_PI / 180);
        }
    } else if (Type == CH32) {
        corrections->sin_vert_correction
                = sin_scan_laser_altitude[Laser_ring / 4]
                + 2 * cos_azimuth_half
                        * sin_scan_mirror_altitude[Laser_ring % 4];
    } else if (Type == CH64) {
        if (Laser_ring % 8 == 0 || Laser_ring % 8 == 1 || Laser_ring % 8 == 2
            || Laser_ring % 8 == 3) {
            corrections->sin_vert_correction
                    = sin(-13.33 * DEG_TO_RAD
                          + floor(Laser_ring / 4) * 1.33 * DEG_TO_RAD)
                    + 2 * cos(rotation / 2 + 1.05 * DEG_TO_RAD)
                            * sin((Laser_ring % 4) * 0.33 * DEG_TO_RAD);
        } else if (
                Laser_ring % 8 == 4 || Laser_ring % 8 == 5
                || Laser_ring % 8 == 6 || Laser_ring % 8 == 7) {
            corrections->sin_vert_correction
                    = sin(-13.33 * DEG_TO_RAD
                          + floor(Laser_ring / 4) * 1.33 * DEG_TO_RAD)
                    + 2 * cos(rotation / 2 - 1.05 * DEG_TO_RAD)
                            * sin((Laser_ring % 4) * 0.33 * DEG_TO_RAD);
        }
    } else if (Type == CH64w) {
        if (Laser_ring / 4 % 2 == 0) {
            cos_xita = cos((rotation / 2.0 + 22.5) * M_PI / 180);
        } else {
            cos_xita = cos((-rotation / 2.0 + 112.5) * M_PI / 180);
        }
        _R_ = theat2_c[Laser_ring] * theat1_c[Laser_ring] * cos_xita
                - theat2_s[Laser_ring] * theat1_s[Laser_ring];
        sin_theat = theat1_s[Laser_ring] + 2 * _R_ * theat2_s[Laser_ring];
        cos_theat = sqrt(1 - pow(sin_theat, 2));
        cos_H_xita = (2 * _R_ * theat2_c[Laser_ring] * cos_xita
                      - theat1_c[Laser_ring])
                / cos_theat;
        sin_H_xita = sqrt(1 - pow(cos_H_xita, 2));

        if (Laser_ring / 4 % 2 == 0) {
            cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt(0.5);
            corrections->sin_vert_correction = sqrt(1 - pow(cos_xita_F, 2));
        } else {
            cos_xita_F = (cos_H_xita + sin_H_xita) * (-sqrt(0.5));
            corrections->sin_vert_correction = sqrt(1 - pow(cos_xita_F, 2));
        }
    } else if (Type == CH120) {
        corrections->sin_vert_correction = sinf(corrections->vert_correction);
    } else if (Type == CH128) {
        if (Laser_ring / 4 % 2 == 0) {
            cos_azimuth_half = sin((rotation - 15 * DEG_TO_RAD) * 0.5);
        } else {
            cos_azimuth_half = cos((rotation + 15 * DEG_TO_RAD) * 0.5);
        }
        corrections->sin_vert_correction
                = sin_scan_laser_altitude_ch128[Laser_ring / 4]
                + 2 * cos_azimuth_half * sinTheta_2[Laser_ring];
    } else if (Type == CH128X1) {
        double _R_ = cosTheta_2[Laser_ring] * cosTheta_1[Laser_ring]
                        * cos_azimuth_half
                - sinTheta_2[Laser_ring] * sinTheta_1[Laser_ring];
        corrections->sin_vert_correction
                = sinTheta_1[Laser_ring] + 2 * _R_ * sinTheta_2[Laser_ring];
    }
    corrections->cos_vert_correction
            = sqrt(1 - pow(corrections->sin_vert_correction, 2));

    double distance = raw_distance + corrections->dist_correction;

    double cos_rot_angle = cos(rotation) * corrections->cos_rot_correction
            + sin(rotation) * corrections->sin_rot_correction;
    double sin_rot_angle = sin(rotation) * corrections->cos_rot_correction
            - cos(rotation) * corrections->sin_rot_correction;

    // Compute the distance in the xy plane (w/o accounting for rotation)
    double xy_distance = distance * corrections->cos_vert_correction;

    // Get 2points calibration values,Linear interpolation to get distance
    // correction for X and Y, that means distance correction use
    // different value at different distance
    distance_corr_x = distance_corr_y = corrections->dist_correction;

    double distance_x = raw_distance + distance_corr_x;
    xy_distance = distance_x * corrections->cos_vert_correction;

    x = xy_distance * sin_rot_angle
            - corrections->horiz_offset_correction * cos_rot_angle;

    double distance_y = raw_distance + distance_corr_y;
    xy_distance = distance_y * corrections->cos_vert_correction;

    y = xy_distance * cos_rot_angle
            + corrections->horiz_offset_correction * sin_rot_angle;
    z = distance * corrections->sin_vert_correction
            + corrections->vert_offset_correction;

    /** Use standard ROS coordinate system (right-hand rule) */
    point->set_x(static_cast<float>(-y));
    point->set_y(static_cast<float>(-x));
    point->set_z(static_cast<float>(z));
}

LslidarParser *LslidarParserFactory::CreateParser(Config source_config) {
    Config config = source_config;

    if (config.model() == LSLIDAR16P) {
        return new Lslidar16Parser(config);
    } else if (config.model() == LSLIDAR32P) {
        return new Lslidar32Parser(config);
    } else if (
            config.model() == LSLIDAR_C32_V4 || config.model() == LSLIDAR_C16_V4
            || config.model() == LSLIDAR_C8_V4
            || config.model() == LSLIDAR_C1_V4) {
        return new LslidarCXV4Parser(config);
    } else if (config.model() == LSLIDAR_CH16) {
        return new LslidarCH16Parser(config);
    } else if (config.model() == LSLIDAR_CH32) {
        return new LslidarCH32Parser(config);
    } else if (config.model() == LSLIDAR_CH64) {
        return new LslidarCH64Parser(config);
    } else if (config.model() == LSLIDAR_CH64w) {
        return new LslidarCH64wParser(config);
    } else if (config.model() == LSLIDAR_CH120) {
        return new LslidarCH120Parser(config);
    } else if (config.model() == LSLIDAR_CH128) {
        return new LslidarCH128Parser(config);
    } else if (config.model() == LSLIDAR_CH128X1) {
        return new LslidarCH128X1Parser(config);
    } else if (config.model() == LSLIDAR_LS128S2) {
        return new LslidarLS128S2Parser(config);
    } else {
        AERROR << "invalid model";
        return nullptr;
    }
}

}  // namespace parser
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
