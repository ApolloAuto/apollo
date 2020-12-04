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

#ifndef LIDAR_HESAI_CONST_VAR_H_
#define LIDAR_HESAI_CONST_VAR_H_

namespace apollo {
namespace drivers {
namespace hesai {

// hesai 40
const int SOB_ANGLE_SIZE = 4;
const int RAW_MEASURE_SIZE = 3;
const int LASER_COUNT = 40;
const int BLOCKS_PER_PACKET = 10;
const int BLOCK_SIZE = RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE;
const int HESAI40_ONE_PACKET_POINTS = LASER_COUNT * BLOCKS_PER_PACKET;
const int HESAI40_MAX_PACKETS = 200;
const int HESAI40_MIN_PACKETS = 90;

const int TIMESTAMP_SIZE = 4;
const int FACTORY_INFO_SIZE = 1;
const int ECHO_SIZE = 1;
const int RESERVE_SIZE = 8;
const int REVOLUTION_SIZE = 2;

const int INFO_SIZE = (TIMESTAMP_SIZE + FACTORY_INFO_SIZE + ECHO_SIZE +
                       RESERVE_SIZE + REVOLUTION_SIZE);
const int UTC_TIME = 6;
const int UDP_SEQUENCE_SIZE = 4;
const int PACKET_SIZE = (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME);
const int PACKET_SIZE_WITH_UDPSEQ = (PACKET_SIZE + UDP_SEQUENCE_SIZE);
const double LASER_RETURN_TO_DISTANCE_RATE = 0.004;

// hesai 64
const int HEAD_SIZE = 8;
const int BLOCKS_PER_PACKET_L64 = 6;
const int BLOCK_AZIMUTH_SIZE = 2;
const int LASER_COUNT_L64 = 64;
const int HS_LIDAR_L64_UNIT_SIZE = 3;
const int BLOCK_SIZE_L64 =
    (HS_LIDAR_L64_UNIT_SIZE * LASER_COUNT_L64 + BLOCK_AZIMUTH_SIZE);
const int ENGINE_VELOCITY_SIZE = 2;
const int PACKET_SIZE_L64 =
    HEAD_SIZE + BLOCK_SIZE_L64 * BLOCKS_PER_PACKET_L64 + INFO_SIZE + UTC_TIME;
const int PACKET_SIZE_L64_WITH_UDPSEQ = (PACKET_SIZE_L64 + UDP_SEQUENCE_SIZE);
const int HESAI64_ONE_PACKET_POINTS = LASER_COUNT_L64 * BLOCKS_PER_PACKET_L64;
const int HESAI64_MAX_PACKETS = 350;
const int HESAI64_MIN_PACKETS = 150;

const int ETHERNET_MTU = 1500;

// gps
const int GPS_PACKET_SIZE = 512;
const int GPS_PACKET_FLAG_SIZE = 2;
const int GPS_PACKET_YEAR_SIZE = 2;
const int GPS_PACKET_MONTH_SIZE = 2;
const int GPS_PACKET_DAY_SIZE = 2;
const int GPS_PACKET_HOUR_SIZE = 2;
const int GPS_PACKET_MINUTE_SIZE = 2;
const int GPS_PACKET_SECOND_SIZE = 2;
const int GPS_ITEM_NUM = 7;

const double PI = 3.14159265358979323846;

static const double pandar40p_elev_angle_map[] = {
    6.96,   5.976,  4.988,   3.996,   2.999,   2.001,  1.667,   1.333,
    1.001,  0.667,  0.333,   0,       -0.334,  -0.667, -1.001,  -1.334,
    -1.667, -2.001, -2.331,  -2.667,  -3,      -3.327, -3.663,  -3.996,
    -4.321, -4.657, -4.986,  -5.311,  -5.647,  -5.974, -6.957,  -7.934,
    -8.908, -9.871, -10.826, -11.772, -12.705, -13.63, -14.543, -15.444};

static const double pandarGeneral_elev_angle_map[] = {
    14.882, 11.032, 8.059,   5.057,   3.04,    2.028,  1.86,    1.688,
    1.522,  1.351,  1.184,   1.013,   0.846,   0.675,  0.508,   0.337,
    0.169,  0,      -0.169,  -0.337,  -0.508,  -0.675, -0.845,  -1.013,
    -1.184, -1.351, -1.522,  -1.688,  -1.86,   -2.028, -2.198,  -2.365,
    -2.536, -2.7,   -2.873,  -3.04,   -3.21,   -3.375, -3.548,  -3.712,
    -3.884, -4.05,  -4.221,  -4.385,  -4.558,  -4.72,  -4.892,  -5.057,
    -5.229, -5.391, -5.565,  -5.726,  -5.898,  -6.061, -7.063,  -8.059,
    -9.06,  -9.885, -11.032, -12.006, -12.974, -13.93, -18.889, -24.897};

static const double pandar40p_horizatal_azimuth_offset_map[] = {
    0.005,  0.006,  0.006,  0.006,  -2.479, -2.479, 2.491,  -4.953,
    -2.479, 2.492,  -4.953, -2.479, 2.492,  -4.953, 0.007,  2.491,
    -4.953, 0.006,  4.961,  -2.479, 0.006,  4.96,   -2.478, 0.006,
    4.958,  -2.478, 2.488,  4.956,  -2.477, 2.487,  2.485,  2.483,
    0.004,  0.004,  0.003,  0.003,  -2.466, -2.463, -2.46,  -2.457};

static const double pandarGeneral_horizatal_azimuth_offset_map[] = {
    -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, 1.042,  3.125,
    5.208,  -5.208, -3.125, -1.042, 1.042,  3.125,  5.208,  -5.208,
    -3.125, -1.042, 1.042,  3.125,  5.208,  -5.208, -3.125, -1.042,
    1.042,  3.125,  5.208,  -5.208, -3.125, -1.042, 1.042,  3.125,
    5.208,  -5.208, -3.125, -1.042, 1.042,  3.125,  5.208,  -5.208,
    -3.125, -1.042, 1.042,  3.125,  5.208,  -5.208, -3.125, -1.042,
    1.042,  3.125,  5.208,  -5.208, -3.125, -1.042, -1.042, -1.042,
    -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042, -1.042};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif
