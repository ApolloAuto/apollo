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

/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *  \ingroup lslidar
 *
 *  These classes Unpack raw Lslidar LIDAR packets into several
 *  useful formats.
 *
 *     lslidar::Data -- virtual base class for unpacking data into
 *                      various formats
 *
 *     lslidar::DataScans -- derived class, unpacks into vector of
 *                      individual laser scans
 *
 *     lslidar::DataXYZ -- derived class, unpacks into XYZ format
 *
 *  \todo make a separate header for each class?
 *
 *  \author Yaxin Liu
 *  \author Patrick Beeson
 *  \author Jack O'Quin
 */

#pragma once

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/lslidar/proto/config.pb.h"
#include "modules/drivers/lidar/lslidar/proto/lslidar.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/lidar/lslidar/parser/calibration.h"

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#define CH16 1
#define CH32 2
#define CH64 3
#define CH64w 4
#define CH120 5
#define CH128 6
#define CH128X1 7

namespace apollo {
namespace drivers {
namespace lslidar {
namespace parser {

using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::lslidar::LSLIDAR16P;
using apollo::drivers::lslidar::LslidarPacket;
using apollo::drivers::lslidar::LslidarScan;

/**
 * Raw Lslidar packet constants and structures.
 */
#define DEG2RAD(x) ((x) * 0.017453293)
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f; /**< degrees */
static const uint16_t ROTATION_MAX_UNITS = 36000;

static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;
static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

/** According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
 *  valid packets with readings up to 200.0. */
static const float DISTANCE_MAX = 200.0f; /**< meters */
static const float DISTANCE_MIN = 0.2f;
static const float DISTANCE_RESOLUTION = 0.01f; /**< meters */
static const double DISTANCE_RESOLUTION2 = 0.0000390625;
static const float DISTANCE_RESOLUTION_NEW = 0.005f;
static const float DISTANCE_MAX_UNITS
        = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);

// laser_block_id
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int LSC16_FIRINGS_PER_BLOCK = 2;
static const int LSC16_SCANS_PER_FIRING = 16;
static const float LSC16_BLOCK_TDURATION = 100.0f;  // [µs]
static const float LSC16_DSR_TOFFSET = 3.125f;      // [µs]
static const float LSC16_FIRING_TOFFSET = 50.0f;    // [µs]

static const int LSC32_FIRINGS_PER_BLOCK = 1;
static const int LSC32_SCANS_PER_FIRING = 32;
static const float LSC32_FIRING_TOFFSET = 100.0f;  // [µs]
static const float LSC32_AZIMUTH_TOFFSET = 12.98f; /**< meters */
static const float LSC32_DISTANCE_TOFFSET = 4.94f; /**< meters */

static const int SCANS_PER_FIRING_C32 = 32;
static const int FIRING_TOFFSET_C32 = 32;
static const double R32_1_ = 0.0361;
static const double CX4_R1_ = 0.0361;    //
static const double CX4_C32W = 0.03416;  // 新C32W   byte[1211] = 0x47
static const double CX4_R1_90 = 0.0209;  // 90度
static const double CX4_conversionAngle_ = 20.25;
static const double CX4_conversionAngle_90 = 27.76;  // 90度

/** \brief Raw Lslidar data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use cstdint types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block {
    uint16_t header;  ///< UPPER_BANK or LOWER_BANK
    uint8_t rotation_1;
    uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get
                         /// 0-35999, divide by 100 to get degrees
    uint8_t data[BLOCK_DATA_SIZE];  // 96
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
    int16_t uint;
    int8_t bytes[2];
};

union four_bytes {
    uint32_t uint;
    uint8_t bytes[4];
};
static const int PACKET_SIZE = 1212;
static const int POINTS_PER_PACKET = 171;
static const int LS_POINTS_PER_PACKET = 149;  // LS128S2 149 points
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

const int ORDER_16[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const int ORDER_32[32]
        = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
           16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
// 1,v2.6
static const double scan_altitude_original_A[32] = {
        -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
        0,   1,   2,   3,   4,   5,   6,   7,  8,  9,  10, 11, 12, 13, 14, 15};

// 1,v3.0
static const double scan_altitude_original_A3[32]
        = {-16, 0, -15, 1, -14, 2,  -13, 3,  -12, 4,  -11, 5,  -10, 6,  -9, 7,
           -8,  8, -7,  9, -6,  10, -5,  11, -4,  12, -3,  13, -2,  14, -1, 15};

// 0.33,v2.6
static const double scan_altitude_original_C[32]
        = {-18,  -15,   -12,  -10,   -8,    -7, -6,    -5, -4,    -3.33, -2.66,
           -3,   -2.33, -2,   -1.33, -1.66, -1, -0.66, 0,  -0.33, 0.33,  0.66,
           1.33, 1,     1.66, 2,     3,     4,  6,     8,  11,    14};

// 0.33,v3.0
static const double scan_altitude_original_C3[32]
        = {-18,   -1, -15,   -0.66, -12,  -0.33, -10,   0,     -8,    0.33, -7,
           0.66,  -6, 1,     -5,    1.33, -4,    1.66,  -3.33, 2,     -3,   3,
           -2.66, 4,  -2.33, 6,     -2,   8,     -1.66, 11,    -1.33, 14};

// CX  4.0
// Pre-compute the sine and cosine for the altitude angles.
// c32 32度
static const double c32_32_vertical_angle[32]
        = {-16, -8, 0, 8,  -15, -7, 1, 9,  -14, -6, 2, 10, -13, -5, 3, 11,
           -12, -4, 4, 12, -11, -3, 5, 13, -10, -2, 6, 14, -9,  -1, 7, 15};
// c32 70度 0x46
static const double c32_70_vertical_angle[32]
        = {-54,   -31, -8,  2.66, -51.5, -28, -6.66, 4,     -49,  -25, -5.33,
           5.5,   -46, -22, -4,   7,     -43, -18.5, -2.66, 9,    -40, -15,
           -1.33, 11,  -37, -12,  0,     13,  -34,   -10,   1.33, 15};

// c32 70度  0x47
static const double c32_70_vertical_angle2[32]
        = {-54.7, -31, -9,  3,    -51.5, -28, -7.5,  4.5,   -49, -25, -6.0,
           6.0,   -46, -22, -4.5, 7.5,   -43, -18.5, -3.0,  9,   -40, -15,
           -1.5,  11,  -37, -12,  0,     13,  -34,   -10.5, 1.5, 15};

// c32 90度
static const double c32_90_vertical_angle[32]
        = {2.487,  25.174, 47.201, 68.819, 5.596,  27.811, 49.999, 71.525,
           8.591,  30.429, 52.798, 74.274, 11.494, 33.191, 55.596, 77.074,
           14.324, 36.008, 58.26,  79.938, 17.096, 38.808, 60.87,  82.884,
           19.824, 41.603, 63.498, 85.933, 22.513, 44.404, 66.144, 89.105};

static const double c16_vertical_angle[16]
        = {-16, 0, -14, 2, -12, 4, -10, 6, -8, 8, -6, 10, -4, 12, -2, 14};

static const double c8_vertical_angle[8] = {-12, 4, -8, 8, -4, 10, 0, 12};
static const double c1_vertical_angle[8] = {0, 0, 0, 0, 0, 0, 0, 0};

static const double scan_altitude_original_2[16]
        = {-0.2617993877991494,
           0.017453292519943295,
           -0.22689280275926285,
           0.05235987755982989,
           -0.19198621771937624,
           0.08726646259971647,
           -0.15707963267948966,
           0.12217304763960307,
           -0.12217304763960307,
           0.15707963267948966,
           -0.08726646259971647,
           0.19198621771937624,
           -0.05235987755982989,
           0.22689280275926285,
           -0.017453292519943295,
           0.2617993877991494};
static const double scan_altitude_original_1[16]
        = {-0.17453292519943295,
           0.01160643952576229,
           -0.15123277968530863,
           0.03490658503988659,
           -0.12793263417118436,
           0.05811946409141117,
           -0.10471975511965978,
           0.08141960960553547,
           -0.08141960960553547,
           0.10471975511965978,
           -0.05811946409141117,
           0.12793263417118436,
           -0.03490658503988659,
           0.15123277968530863,
           -0.01160643952576229,
           0.17453292519943295};

static const double scan_laser_altitude[8] = {
        -0.11641346110802178,
        -0.09302604913129776,
        -0.06981317007977318,
        -0.04642575810304917,
        -0.023212879051524585,
        -0.0,
        0.023212879051524585,
        0.04642575810304917,
};

static const double scan_laser_altitude_ch32[8] = {
        -0.24434609527920614,
        -0.19198621771937624,
        -0.13962634015954636,
        -0.08726646259971647,
        -0.03490658503988659,
        -0.0,
        0.03490658503988659,
        0.08726646259971647,
};

static const double big_angle[32] = {
        -17, -16,   -15,    -14,   -13,    -12, -11,    -10, -9,     -8, -7,
        -6,  -5,    -4.125, -4,    -3.125, -3,  -2.125, -2,  -1.125, -1, -0.125,
        0,   0.875, 1,      1.875, 2,      3,   4,      5,   6,      7};

static const double sin_scan_laser_altitude[8] = {
        std::sin(scan_laser_altitude[0]),
        std::sin(scan_laser_altitude[1]),
        std::sin(scan_laser_altitude[2]),
        std::sin(scan_laser_altitude[3]),
        std::sin(scan_laser_altitude[4]),
        std::sin(scan_laser_altitude[5]),
        std::sin(scan_laser_altitude[6]),
        std::sin(scan_laser_altitude[7]),
};

static const double sin_scan_laser_altitude_ch32[8] = {
        std::sin(scan_laser_altitude_ch32[0]),
        std::sin(scan_laser_altitude_ch32[1]),
        std::sin(scan_laser_altitude_ch32[2]),
        std::sin(scan_laser_altitude_ch32[3]),
        std::sin(scan_laser_altitude_ch32[4]),
        std::sin(scan_laser_altitude_ch32[5]),
        std::sin(scan_laser_altitude_ch32[6]),
        std::sin(scan_laser_altitude_ch32[7]),
};

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_laser_altitude_ch128[32] = {
        -0.29670597283903605, -0.2792526803190927,   -0.2617993877991494,
        -0.24434609527920614, -0.22689280275926285,  -0.20943951023931956,
        -0.19198621771937624, -0.17453292519943295,  -0.15707963267948966,
        -0.13962634015954636, -0.12217304763960307,  -0.10471975511965978,
        -0.08726646259971647, -0.06981317007977318,  -0.05235987755982989,
        -0.03490658503988659, -0.017453292519943295, 0.0,
        0.017453292519943295, 0.03490658503988659,   0.05235987755982989,
        0.06981317007977318,  0.08726646259971647,   0.10471975511965978,
        0.12217304763960307,  0.13962634015954636,   0.15707963267948966,
        0.17453292519943295,  0.19198621771937624,   0.20943951023931956,
        0.22689280275926285,  0.24434609527920614,
};

static const double sin_scan_laser_altitude_ch128[32] = {
        std::sin(scan_laser_altitude_ch128[0]),
        std::sin(scan_laser_altitude_ch128[1]),
        std::sin(scan_laser_altitude_ch128[2]),
        std::sin(scan_laser_altitude_ch128[3]),
        std::sin(scan_laser_altitude_ch128[4]),
        std::sin(scan_laser_altitude_ch128[5]),
        std::sin(scan_laser_altitude_ch128[6]),
        std::sin(scan_laser_altitude_ch128[7]),
        std::sin(scan_laser_altitude_ch128[8]),
        std::sin(scan_laser_altitude_ch128[9]),
        std::sin(scan_laser_altitude_ch128[10]),
        std::sin(scan_laser_altitude_ch128[11]),
        std::sin(scan_laser_altitude_ch128[12]),
        std::sin(scan_laser_altitude_ch128[13]),
        std::sin(scan_laser_altitude_ch128[14]),
        std::sin(scan_laser_altitude_ch128[15]),
        std::sin(scan_laser_altitude_ch128[16]),
        std::sin(scan_laser_altitude_ch128[17]),
        std::sin(scan_laser_altitude_ch128[18]),
        std::sin(scan_laser_altitude_ch128[19]),
        std::sin(scan_laser_altitude_ch128[20]),
        std::sin(scan_laser_altitude_ch128[21]),
        std::sin(scan_laser_altitude_ch128[22]),
        std::sin(scan_laser_altitude_ch128[23]),
        std::sin(scan_laser_altitude_ch128[24]),
        std::sin(scan_laser_altitude_ch128[25]),
        std::sin(scan_laser_altitude_ch128[26]),
        std::sin(scan_laser_altitude_ch128[27]),
        std::sin(scan_laser_altitude_ch128[28]),
        std::sin(scan_laser_altitude_ch128[29]),
        std::sin(scan_laser_altitude_ch128[30]),
        std::sin(scan_laser_altitude_ch128[31]),
};

static const double scan_mirror_altitude[4] = {
        -0.0,
        0.005759586531581287,
        0.011693705988362009,
        0.017453292519943295,
};

static const double sin_scan_mirror_altitude[4] = {
        std::sin(scan_mirror_altitude[0]),
        std::sin(scan_mirror_altitude[1]),
        std::sin(scan_mirror_altitude[2]),
        std::sin(scan_mirror_altitude[3]),
};

static const double scan_mirror_altitude_ch128[4] = {
        0.0,
        0.004363323129985824,
        0.008726646259971648,
        0.013089969389957472,
};

static const double sin_scan_mirror_altitude_ch128[4] = {
        std::sin(scan_mirror_altitude_ch128[0]),
        std::sin(scan_mirror_altitude_ch128[1]),
        std::sin(scan_mirror_altitude_ch128[2]),
        std::sin(scan_mirror_altitude_ch128[3]),
};

static const int POINTS_PER_PACKET_SINGLE_ECHO = 1192;  // modify
static const int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;  // modify

/** \brief Raw Lslidar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct FiringC32 {
    uint16_t firing_azimuth[BLOCKS_PER_PACKET];
    uint16_t azimuth[SCANS_PER_PACKET];
    double distance[SCANS_PER_PACKET];
    double intensity[SCANS_PER_PACKET];
} FiringC32;

union TwoBytes {
    uint16_t distance;
    uint8_t bytes[2];
};

struct RawBlock {
    uint16_t header;
    uint16_t rotation;  // 0-35999
    uint8_t data[BLOCK_DATA_SIZE];
};

struct RawPacket_C32 {
    RawBlock blocks[BLOCKS_PER_PACKET];
    uint32_t time_stamp;
    uint8_t factory[2];
};

typedef struct raw_packet {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint8_t gps_timestamp[10];  // gps时间 1200-1209
    uint8_t status_type;
    uint8_t status_value;
} raw_packet_t;

union vertical_point {
    uint8_t uint[2];
    uint16_t value;
};

struct Point {
    uint8_t vertical_line;  // 0-127
    uint8_t azimuth_1;
    uint8_t azimuth_2;
    uint8_t distance_1;
    uint8_t distance_2;
    uint8_t distance_3;
    uint8_t intensity;
};

struct RawPacket {
    Point points[POINTS_PER_PACKET];
    uint8_t nodata[3];
    uint32_t time_stamp;
    uint8_t factory[2];
};

struct Firing {
    // double vertical_angle;
    int vertical_line;
    double azimuth;
    double distance;
    int intensity;
};

struct Firing_LS128S2 {
    double vertical_angle;
    //        int vertical_line;
    double azimuth;
    double distance;
    float intensity;
    double time;
    int channel_number;
};

struct Point_LS128S2 {
    uint8_t vertical_line;  // 0-127
    uint8_t azimuth_1;
    uint8_t azimuth_2;
    uint8_t distance_1;
    uint8_t distance_2;
    uint8_t distance_3;
    uint8_t intensity;
};

struct RawPacket_LS128S2 {
    Point points[POINTS_PER_PACKET];
    uint8_t nodata[3];
    uint32_t time_stamp;
    uint8_t factory[2];
};

static const float nan = std::numeric_limits<float>::signaling_NaN();

/** \brief Lslidar data conversion class */
class LslidarParser {
 public:
    LslidarParser() {}
    explicit LslidarParser(const Config& config);
    virtual ~LslidarParser() {}

    /** \brief Set up for data processing.
     *  Perform initializations needed before data processing can
     *  begin:
     *  - read device-specific angles calibration
     *
     *  @param private_nh private node handle for ROS parameters
     *  @returns 0 if successful;
     *           errno value for failure
     */
    virtual void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg)
            = 0;
    virtual void setup();

    // Order point cloud fod IDL by lslidar model
    virtual void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud) = 0;

    const double get_last_timestamp() {
        return last_time_stamp_;
    }

 protected:
    double scan_altitude[16];
    float sin_azimuth_table[ROTATION_MAX_UNITS];
    float cos_azimuth_table[ROTATION_MAX_UNITS];
    double cos_scan_altitude_caliration[LSC32_SCANS_PER_FIRING];
    double sin_scan_altitude_caliration[LSC32_SCANS_PER_FIRING];
    double last_time_stamp_;
    double theat1_s[128];
    double theat2_s[128];
    double theat1_c[128];
    double theat2_c[128];
    double prism_angle[4];
    double prism_angle64[4];
    Config config_;
    Calibration calibration_;
    bool need_two_pt_correction_;
    bool config_vert_angle;
    uint64_t current_packet_time = 0;
    uint64_t previous_packet_time = 0;

    bool is_scan_valid(int rotation, float distance);

    void ComputeCoords(
            const float& raw_distance,
            LaserCorrection* corrections,
            const uint16_t rotation,
            PointXYZIT* point);

    void ComputeCoords2(
            int Laser_ring,
            int Type,
            const float& raw_distance,
            LaserCorrection* corrections,
            const double rotation,
            PointXYZIT* point);
};  // class LslidarParser

class Lslidar16Parser : public LslidarParser {
 public:
    explicit Lslidar16Parser(const Config& config);
    ~Lslidar16Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<apollo::drivers::lslidar::LslidarScan>&
                    scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

 private:
    void Unpack(
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc,
            int packet_number);
    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint8_t difop_data[PACKET_SIZE];
    int block_num;
    int packet_number_;
    bool read_difop_;
};  // class Lslidar16Parser

class Lslidar32Parser : public LslidarParser {
 public:
    explicit Lslidar32Parser(const Config& config);
    ~Lslidar32Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<apollo::drivers::lslidar::LslidarScan>&
                    scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

 private:
    void Unpack(
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc,
            int packet_number);
    double adjust_angle;
    double adjust_angle_two;
    double adjust_angle_three;
    double adjust_angle_four;

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    uint8_t difop_data[PACKET_SIZE];
    int block_num;
    int lslidar_type;
    int packet_number_;
    bool read_difop_;

    double scan_altitude_A[32] = {0};
    double scan_altitude_C[32] = {0};
};  // class Lslidar32Parser

class LslidarCXV4Parser : public LslidarParser {
 public:
    explicit LslidarCXV4Parser(const Config& config);
    ~LslidarCXV4Parser() {}

    void decodePacket(const RawPacket_C32* packet);

    void GeneratePointcloud(
            const std::shared_ptr<apollo::drivers::lslidar::LslidarScan>&
                    scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

    bool checkPacketValidity(const RawPacket_C32* packet);

 private:
    void Unpack(
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc,
            int packet_number);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    uint8_t difop_data[PACKET_SIZE];
    int block_num;
    int lslidar_type;
    int packet_number_;
    bool read_difop_;

    // double c32_vertical_angle[32] = {0};
    double cos_scan_altitude[32] = {0};
    double sin_scan_altitude[32] = {0};
    double cos_azimuth_table[36000];
    double sin_azimuth_table[36000];
    int vertical_angle_;  // 雷达垂直角度： 32度，70度，90度
    FiringC32 firings;
    double distance_unit_;
    int lidar_number_;
    bool is_new_c32w_;
    bool is_get_scan_altitude_;
};  // class LslidarCXV4Parser

class Lslidar401Parser : public LslidarParser {
 public:
    explicit Lslidar401Parser(const Config& config);
    ~Lslidar401Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

 private:
    void decodePacket(const raw_packet_t* packet);
    void Unpack(
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint8_t difop_data[PACKET_SIZE];
    int block_num;
};  // class Lslidar401Parser

class LslidarCH16Parser : public LslidarParser {
 public:
    explicit LslidarCH16Parser(const Config& config);
    ~LslidarCH16Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    uint64_t time_last;
    uint8_t difop_data[PACKET_SIZE];
    Firing firings[POINTS_PER_PACKET];
};  // class LslidarCH16Parser

class LslidarCH32Parser : public LslidarParser {
 public:
    explicit LslidarCH32Parser(const Config& config);
    ~LslidarCH32Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    uint64_t time_last;
    uint8_t difop_data[PACKET_SIZE];
    Firing firings[POINTS_PER_PACKET];
};  // class LslidarCH32Parser

class LslidarCH64Parser : public LslidarParser {
 public:
    explicit LslidarCH64Parser(const Config& config);
    ~LslidarCH64Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    uint64_t time_last;
    uint8_t difop_data[PACKET_SIZE];
    Firing firings[POINTS_PER_PACKET];
};  // class LslidarCH64Parser

class LslidarCH64wParser : public LslidarParser {
 public:
    explicit LslidarCH64wParser(const Config& config);
    ~LslidarCH64wParser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
    void Order(std::shared_ptr<apollo::drivers::PointCloud> cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<apollo::drivers::PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    uint64_t time_last;
    double theat1_s[128];
    double theat2_s[128];
    double theat1_c[128];
    double theat2_c[128];
    double point_time_diff;
    uint8_t difop_data[PACKET_SIZE];
    Firing firings[POINTS_PER_PACKET];
};  // class LslidarCH64Parser

class LslidarCH120Parser : public LslidarParser {
 public:
    explicit LslidarCH120Parser(const Config& config);
    ~LslidarCH120Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<PointCloud>& out_msg);
    void Order(std::shared_ptr<PointCloud> cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    uint64_t time_last;
    uint8_t difop_data[PACKET_SIZE];
    Firing firings[POINTS_PER_PACKET];
};  // class LslidarCH120Parser

class LslidarCH128Parser : public LslidarParser {
 public:
    explicit LslidarCH128Parser(const Config& config);
    ~LslidarCH128Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<PointCloud>& out_msg);
    void Order(std::shared_ptr<PointCloud> cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    double sinTheta_2[128];
    double prism_angle128[4];
    uint64_t time_last;
    uint8_t difop_data[PACKET_SIZE];
    Firing firings[POINTS_PER_PACKET];
};  // class LslidarCH128Parser

class LslidarCH128X1Parser : public LslidarParser {
 public:
    explicit LslidarCH128X1Parser(const Config& config);
    ~LslidarCH128X1Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<PointCloud>& out_msg);
    void Order(std::shared_ptr<PointCloud> cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    uint8_t difop_data[PACKET_SIZE];
    Firing firings[POINTS_PER_PACKET];
};  // class LslidarCH128X1Parser

class LslidarLS128S2Parser : public LslidarParser {
 public:
    explicit LslidarLS128S2Parser(const Config& config);
    ~LslidarLS128S2Parser() {}

    void GeneratePointcloud(
            const std::shared_ptr<LslidarScan>& scan_msg,
            const std::shared_ptr<PointCloud>& out_msg);
    void Order(std::shared_ptr<PointCloud> cloud);

    int convertCoordinate(const struct Firing_LS128S2& lidardata);

    int convertCoordinate(
            const struct Firing_LS128S2& lidardata,
            std::shared_ptr<PointCloud> out_cloud);

 private:
    void Unpack(
            int num,
            const LslidarPacket& pkt,
            std::shared_ptr<PointCloud> pc);

    // Previous Lslidar packet time stamp. (offset to the top hour)
    double previous_packet_stamp_;
    uint64_t gps_base_usec_;  // full time
    uint64_t last_packet_time;
    uint64_t last_point_time;
    size_t packets_size;
    uint8_t difop_data[PACKET_SIZE];
    Firing_LS128S2 firings[LS_POINTS_PER_PACKET];
    int frame_count = 0;  // LS系列，叠帧模式会用到

    double cos_table[36000]{};
    double sin_table[36000]{};
    double cos_mirror_angle[4]{};
    double sin_mirror_angle[4]{};
    double g_fAngleAcc_V = 0.01;
    bool is_add_frame_ = false;
    int return_mode = 1;
    std::shared_ptr<PointCloud> cur_pc;
    std::shared_ptr<PointCloud> pre_pc;
    int packet_number_ = 0;
};  // class LslidarLS128S2Parser

class LslidarParserFactory {
 public:
    static LslidarParser* CreateParser(Config config);
};

}  // namespace parser
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
