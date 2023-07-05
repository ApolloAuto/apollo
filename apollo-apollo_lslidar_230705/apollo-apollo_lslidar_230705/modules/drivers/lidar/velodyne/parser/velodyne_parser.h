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
 *
 *  Velodyne HDL-64E 3D LIDAR data accessors
 *
 *  \ingroup velodyne
 *
 *  These classes Unpack raw Velodyne LIDAR packets into several
 *  useful formats.
 *
 *     velodyne::Data -- virtual base class for unpacking data into
 *                      various formats
 *
 *     velodyne::DataScans -- derived class, unpacks into vector of
 *                      individual laser scans
 *
 *     velodyne::DataXYZ -- derived class, unpacks into XYZ format
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

#include <boost/format.hpp>

#include "modules/drivers/lidar/proto/velodyne.pb.h"
#include "modules/drivers/lidar/proto/velodyne_config.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/drivers/lidar/velodyne/parser/calibration.h"
#include "modules/drivers/lidar/velodyne/parser/const_variables.h"
#include "modules/drivers/lidar/velodyne/parser/online_calibration.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::velodyne::DUAL;
using apollo::drivers::velodyne::HDL32E;
using apollo::drivers::velodyne::HDL64E_S2;
using apollo::drivers::velodyne::HDL64E_S3D;
using apollo::drivers::velodyne::HDL64E_S3S;
using apollo::drivers::velodyne::LAST;
using apollo::drivers::velodyne::STRONGEST;
using apollo::drivers::velodyne::VelodynePacket;
using apollo::drivers::velodyne::VelodyneScan;
using apollo::drivers::velodyne::VLP16;
using apollo::drivers::velodyne::VLP32C;
using apollo::drivers::velodyne::VLS128;

/**
 * Raw Velodyne packet constants and structures.
 */
static const int BLOCK_SIZE = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f; /**< degrees */
// static const uint16_t ROTATION_MAX_UNITS = 36000; [>*< hundredths of degrees
// <]
// because angle_rang is [0, 36000], so the size is 36001
static const uint16_t ROTATION_MAX_UNITS = 36001; /**< hundredths of degrees */

/** According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
 *  valid packets with readings up to 130.0. */
static const float DISTANCE_MAX = 130.0f;        /**< meters */
static const float DISTANCE_RESOLUTION = 0.002f; /**< meters */
static const float DISTANCE_MAX_UNITS =
    (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);

// laser_block_id
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

static const float ANGULAR_RESOLUTION = 0.00300919f;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;
static const float VLP16_DSR_TOFFSET = 2.304f;
static const float VLP16_FIRING_TOFFSET = 55.296f;

static const float VLS128_CHANNEL_TDURATION = 2.304f;
static const float VLS128_SEQ_TDURATION = 55.296f;

static const float VLP32_DISTANCE_RESOLUTION = 0.004f;
static const float VSL128_DISTANCE_RESOLUTION = 0.004f;
static const float CHANNEL_TDURATION = 2.304f;
static const float SEQ_TDURATION = 55.296f;

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use cstdint types, so things work with both 64 and 32-bit machines
 */
struct RawBlock {
  uint16_t laser_block_id;  ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;        ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
};

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union RawDistance {
  uint16_t raw_distance;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

enum StatusType {
  HOURS = 72,
  MINUTES = 77,
  SECONDS = 83,
  DATE = 68,
  MONTH = 78,
  YEAR = 89,
  GPS_STATUS = 71
};

/** \brief Raw Velodyne packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  status has either a temperature encoding or the microcode level
 */
struct RawPacket {
  RawBlock blocks[BLOCKS_PER_PACKET];
  // uint16_t revolution;
  // uint8_t status[PACKET_STATUS_SIZE];
  unsigned int gps_timestamp;
  unsigned char status_type;
  unsigned char status_value;
};

/** Special Definition for VLS128 support **/
// static const int VLS128_NUM_CHANS_PER_BLOCK = 128;
// static const int VLS128_BLOCK_DATA_SIZE = VLS128_NUM_CHANS_PER_BLOCK *
// RAW_SCAN_SIZE;
//
// struct Vls128RawBlock {
//   uint16_t header;
//   uint16_t rotation;
//   uint8_t data[VLS128_BLOCK_DATA_SIZE];
// };
//
// struct Vls128RawPacket {
//   Vls128RawBlock blocks[3];
//   uint16_t revolution;
//   uint8_t status[PACKET_STATUS_SIZE];
// };

// Convert related config, get value from private_nh param server, used by
// velodyne raw data
// struct Config {
//  double max_range;  ///< maximum range to publish
//  double min_range;  ///< minimum range to publish
//  double max_angle;
//  double min_angle;
//  double view_direction;
//  double view_width;
//  bool calibration_online;
//  std::string calibration_file;
//  std::string model;  // VLP16,32E, 64E_32
//  bool organized;     // is point cloud Order
//};

// enum Mode { STRONGEST, LAST, DUAL };

static const float nan = std::numeric_limits<float>::signaling_NaN();

/** \brief Velodyne data conversion class */
class VelodyneParser {
 public:
  VelodyneParser() {}
  explicit VelodyneParser(const Config& config);
  virtual ~VelodyneParser() {}

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns 0 if successful;
   *           errno value for failure
   */
  virtual void GeneratePointcloud(const std::shared_ptr<VelodyneScan>& scan_msg,
                                  std::shared_ptr<PointCloud> out_msg) = 0;
  virtual void setup();
  // Order point cloud fod IDL by velodyne model
  virtual void Order(std::shared_ptr<PointCloud> cloud) = 0;

  const Calibration& get_calibration() { return calibration_; }
  const double get_last_timestamp() { return last_time_stamp_; }

 protected:
  const float (*inner_time_)[12][32];

  Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];
  double last_time_stamp_;
  Config config_;
  // Last Velodyne packet time stamp. (Full time)
  bool need_two_pt_correction_;
  Mode mode_;

  PointXYZIT get_nan_point(uint64_t timestamp);
  void init_angle_params(double view_direction, double view_width);
  /**
   * \brief Compute coords with the data in block
   *
   * @param tmp A two bytes union store the value of laser distance information
   * @param index The index of block
   */
  void ComputeCoords(const float& raw_distance,
                     const LaserCorrection& corrections,
                     const uint16_t rotation, PointXYZIT* point);

  bool is_scan_valid(int rotation, float distance);

  /**
   * \brief Unpack velodyne packet
   *
   */
  virtual void Unpack(const VelodynePacket& pkt,
                      std::shared_ptr<PointCloud> pc) = 0;

  uint64_t GetGpsStamp(double current_stamp, double* previous_stamp,
                       uint64_t* gps_base_usec);

  virtual uint64_t GetTimestamp(double base_time, float time_offset,
                                uint16_t laser_block_id) = 0;
};  // class VelodyneParser

class Velodyne64Parser : public VelodyneParser {
 public:
  explicit Velodyne64Parser(const Config& config);
  ~Velodyne64Parser() {}

  void GeneratePointcloud(const std::shared_ptr<VelodyneScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  void Order(std::shared_ptr<PointCloud> cloud);
  void setup() override;

 private:
  void SetBaseTimeFromPackets(const VelodynePacket& pkt);
  void CheckGpsStatus(const VelodynePacket& pkt);
  uint64_t GetTimestamp(double base_time, float time_offset,
                        uint16_t laser_block_id);
  void Unpack(const VelodynePacket& pkt, std::shared_ptr<PointCloud> pc);
  void InitOffsets();
  int IntensityCompensate(const LaserCorrection& corrections,
                          const uint16_t raw_distance, int intensity);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double previous_packet_stamp_[4];
  uint64_t gps_base_usec_[4];  // full time
  bool is_s2_;
  int offsets_[64];

  OnlineCalibration online_calibration_;
};  // class Velodyne64Parser

class Velodyne32Parser : public VelodyneParser {
 public:
  explicit Velodyne32Parser(const Config& config);
  ~Velodyne32Parser() {}

  void GeneratePointcloud(const std::shared_ptr<VelodyneScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  void Order(std::shared_ptr<PointCloud> cloud);

 private:
  uint64_t GetTimestamp(double base_time, float time_offset,
                        uint16_t laser_block_id);
  void Unpack(const VelodynePacket& pkt, std::shared_ptr<PointCloud> pc);
  void UnpackVLP32C(const VelodynePacket& pkt, std::shared_ptr<PointCloud> pc);
  // Previous laser firing time stamp. (offset to the top hour)
  double previous_firing_stamp_;
  uint64_t gps_base_usec_;  // full time
};                          // class Velodyne32Parser

class Velodyne16Parser : public VelodyneParser {
 public:
  explicit Velodyne16Parser(const Config& config);
  ~Velodyne16Parser() {}

  void GeneratePointcloud(const std::shared_ptr<VelodyneScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  void Order(std::shared_ptr<PointCloud> cloud);

 private:
  uint64_t GetTimestamp(double base_time, float time_offset,
                        uint16_t laser_block_id);
  void Unpack(const VelodynePacket& pkt, std::shared_ptr<PointCloud> pc);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double previous_packet_stamp_;
  uint64_t gps_base_usec_;  // full time
};                          // class Velodyne16Parser

class Velodyne128Parser : public VelodyneParser {
 public:
  explicit Velodyne128Parser(const Config& config);
  ~Velodyne128Parser() {}

  void GeneratePointcloud(const std::shared_ptr<VelodyneScan>& scan_msg,
                          std::shared_ptr<PointCloud> out_msg);
  void Order(std::shared_ptr<PointCloud> cloud);

 private:
  uint64_t GetTimestamp(double base_time, float time_offset,
                        uint16_t laser_block_id);
  void Unpack(const VelodynePacket& pkt, std::shared_ptr<PointCloud> pc);
  int IntensityCompensate(const LaserCorrection& corrections,
                          const uint16_t raw_distance, int intensity);
  // Previous Velodyne packet time stamp. (offset to the top hour)
  double previous_packet_stamp_;
  uint64_t gps_base_usec_;  // full time
};                          // class Velodyne128Parser

class VelodyneParserFactory {
 public:
  static VelodyneParser* CreateParser(Config config);
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
