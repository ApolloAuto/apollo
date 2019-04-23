/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

// An parser for decoding binary messages from a Starneto Newton-M2 receiver.
// The following messages must be logged in order for this parser to work
// properly.
//

#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/novatel_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace newtonm2 {

constexpr size_t BUFFER_SIZE = 256;

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD_M2 = M_PI / 180.0;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

// The NewtonM2's orientation covariance matrix is pitch, roll, and yaw. We use
// the index array below
// to convert it to the orientation covariance matrix with order roll, pitch,
// and yaw.
constexpr int INDEX[] = {4, 3, 5, 1, 0, 2, 7, 6, 8};
static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}

// CRC algorithm from the NewtonM2 document.
inline uint32_t crc32_word(uint32_t word) {
  for (int j = 0; j < 8; ++j) {
    if (word & 1) {
      word = (word >> 1) ^ 0xEDB88320;
    } else {
      word >>= 1;
    }
  }
  return word;
}

inline uint32_t crc32_block(const uint8_t* buffer, size_t length) {
  uint32_t word = 0;
  while (length--) {
    uint32_t t1 = (word >> 8) & 0xFFFFFF;
    uint32_t t2 = crc32_word((word ^ *buffer++) & 0xFF);
    word = t1 ^ t2;
  }
  return word;
}

// Converts NewtonM2's azimuth (north = 0, east = 90) to FLU yaw (east = 0,
// north = pi/2).
constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
  return (90.0 - azimuth) * DEG_TO_RAD_M2;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU
// measurements.
inline void rfu_to_flu(double r, double f, double u,
                       ::apollo::common::Point3D* flu) {
  flu->set_x(f);
  flu->set_y(-r);
  flu->set_z(u);
}

}  // namespace newtonm2

class NewtonM2Parser : public Parser {
 public:
  NewtonM2Parser();
  explicit NewtonM2Parser(const config::Config& config);

  virtual MessageType GetMessage(MessagePtr* message_ptr);

 private:
  bool check_crc();

  Parser::MessageType PrepareMessage(MessagePtr* message_ptr);

  // The handle_xxx functions return whether a message is ready.
  bool HandleBestPos(const novatel::BestPos* pos, uint16_t gps_week,
                     uint32_t gps_millisecs);

  bool HandleGnssBestpos(const novatel::BestPos* pos, uint16_t gps_week,
                         uint32_t gps_millisecs);

  bool HandleBestVel(const novatel::BestVel* vel, uint16_t gps_week,
                     uint32_t gps_millisecs);

  bool HandleCorrImuData(const novatel::CorrImuData* imu);

  bool HandleInsCov(const novatel::InsCov* cov);

  bool HandleInsPva(const novatel::InsPva* pva);

  bool HandleInsPvax(const novatel::InsPvaX* pvax, uint16_t gps_week,
                     uint32_t gps_millisecs);

  bool HandleRawImuX(const novatel::RawImuX* imu);

  bool HandleRawImu(const novatel::RawImu* imu);

  bool HandleBdsEph(const novatel::BDS_Ephemeris* bds_emph);

  bool HandleGpsEph(const novatel::GPS_Ephemeris* gps_emph);

  bool HandleGloEph(const novatel::GLO_Ephemeris* glo_emph);

  void SetObservationTime();

  bool DecodeGnssObservation(const uint8_t* obs_data,
                             const uint8_t* obs_data_end);

  bool HandleHeading(const novatel::Heading* heading, uint16_t gps_week,
                     uint32_t gps_millisecs);
  double gyro_scale_ = 0.0;

  double accel_scale_ = 0.0;

  float imu_measurement_span_ = 1.0f / 200.0f;
  float imu_measurement_hz_ = 200.0f;

  int imu_frame_mapping_ = 5;

  double imu_measurement_time_previous_ = -1.0;

  std::vector<uint8_t> buffer_;

  size_t header_length_ = 0;

  size_t total_length_ = 0;

  config::ImuType imu_type_ = config::ImuType::CPT_XW5651;

  // NONE is an unused value.
  novatel::SolutionStatus solution_status_ =
      static_cast<novatel::SolutionStatus>(novatel::SolutionStatus::NONE);
  novatel::SolutionType position_type_ =
      static_cast<novatel::SolutionType>(novatel::SolutionType::NONE);
  novatel::SolutionType velocity_type_ =
      static_cast<novatel::SolutionType>(novatel::SolutionType::NONE);
  novatel::InsStatus ins_status_ =
      static_cast<novatel::InsStatus>(novatel::InsStatus::NONE);

  raw_t raw_;  // used for observation data

  ::apollo::drivers::gnss::Gnss gnss_;
  ::apollo::drivers::gnss::GnssBestPose bestpos_;
  ::apollo::drivers::gnss::Imu imu_;
  ::apollo::drivers::gnss::Ins ins_;
  ::apollo::drivers::gnss::InsStat ins_stat_;
  ::apollo::drivers::gnss::GnssEphemeris gnss_ephemeris_;
  ::apollo::drivers::gnss::EpochObservation gnss_observation_;
  ::apollo::drivers::gnss::Heading heading_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
