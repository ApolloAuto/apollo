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

// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include "gnss/parser.h"
#include "novatel_messages.h"
#include "proto/gnss.pb.h"
#include "proto/gnss_best_pose.pb.h"
#include "proto/gnss_raw_observation.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"
#include "rtcm/rtcm_decode.h"
#include "util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr size_t BUFFER_SIZE = 256;

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

// The NovAtel's orientation covariance matrix is pitch, roll, and yaw. We use
// the index array below
// to convert it to the orientation covariance matrix with order roll, pitch,
// and yaw.
constexpr int INDEX[] = {4, 3, 5, 1, 0, 2, 7, 6, 8};
static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}

// CRC algorithm from the NovAtel document.
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

// Converts NovAtel's azimuth (north = 0, east = 90) to FLU yaw (east = 0, north
// = pi/2).
constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
  return (90.0 - azimuth) * DEG_TO_RAD;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU
// measurements.
inline void rfu_to_flu(double r, double f, double u,
                       ::apollo::common::Point3D* flu) {
  flu->set_x(f);
  flu->set_y(-r);
  flu->set_z(u);
}

}  // namespace

class NovatelParser : public Parser {
 public:
  NovatelParser();

  virtual MessageType get_message(MessagePtr& message_ptr);

 private:
  bool check_crc();

  Parser::MessageType prepare_message(MessagePtr& message_ptr);

  // The handle_xxx functions return whether a message is ready.
  bool handle_best_pos(const novatel::BestPos* pos, uint16_t gps_week,
                       uint32_t gps_millisecs);

  bool handle_gnss_bestpos(const novatel::BestPos* pos, uint16_t gps_week,
                           uint32_t gps_millisecs);

  bool handle_best_vel(const novatel::BestVel* vel, uint16_t gps_week,
                       uint32_t gps_millisecs);

  bool handle_corr_imu_data(const novatel::CorrImuData* imu);

  bool handle_ins_cov(const novatel::InsCov* cov);

  bool handle_ins_pva(const novatel::InsPva* pva);

  bool handle_ins_pvax(const novatel::InsPvaX* pvax, uint16_t gps_week,
                       uint32_t gps_millisecs);

  bool handle_raw_imu_x(const novatel::RawImuX* imu);

  bool handle_raw_imu(const novatel::RawImu* imu);

  bool handle_bds_eph(const novatel::BDS_Ephemeris* bds_emph);

  bool handle_gps_eph(const novatel::GPS_Ephemeris* gps_emph);

  bool handle_glo_eph(const novatel::GLO_Ephemeris* glo_emph);

  void set_observation_time();

  bool decode_gnss_observation(const uint8_t* obs_data,
                               const uint8_t* obs_data_end);

  double _gyro_scale = 0.0;

  double _accel_scale = 0.0;

  float _imu_measurement_span = 1.0 / 200.0;
  float _imu_measurement_hz = 200.0;

  // TODO: Get mapping from configuration file.
  int _imu_frame_mapping = 5;

  double _imu_measurement_time_previous = -1.0;

  std::vector<uint8_t> _buffer;

  size_t _header_length = 0;

  size_t _total_length = 0;

  // -1 is an unused value.
  novatel::SolutionStatus _solution_status =
      static_cast<novatel::SolutionStatus>(-1);
  novatel::SolutionType _position_type = static_cast<novatel::SolutionType>(-1);
  novatel::SolutionType _velocity_type = static_cast<novatel::SolutionType>(-1);
  novatel::InsStatus _ins_status = static_cast<novatel::InsStatus>(-1);

  raw_t _raw;  // used for observation data

  ::apollo::drivers::gnss::Gnss _gnss;
  ::apollo::drivers::gnss::GnssBestPose _bestpos;
  ::apollo::drivers::gnss::Imu _imu;
  ::apollo::drivers::gnss::Ins _ins;
  ::apollo::drivers::gnss::InsStat _ins_stat;
  ::apollo::drivers::gnss::GnssEphemeris _gnss_ephemeris;
  ::apollo::drivers::gnss::EpochObservation _gnss_observation;
};

Parser* Parser::create_novatel() {
  return new NovatelParser();
}

NovatelParser::NovatelParser() {
  _buffer.reserve(BUFFER_SIZE);
  _ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  _ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  _ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  if (1 != init_raw(&_raw)) {
    ROS_FATAL_STREAM("memory allocation error for observation data structure.");
  }
}

Parser::MessageType NovatelParser::get_message(MessagePtr& message_ptr) {
  if (_data == nullptr) {
    return MessageType::NONE;
  }

  while (_data < _data_end) {
    if (_buffer.size() == 0) {  // Looking for SYNC0
      if (*_data == novatel::SYNC_0) {
        _buffer.push_back(*_data);
      }
      ++_data;
    } else if (_buffer.size() == 1) {  // Looking for SYNC1
      if (*_data == novatel::SYNC_1) {
        _buffer.push_back(*_data++);
      } else {
        _buffer.clear();
      }
    } else if (_buffer.size() == 2) {  // Looking for SYNC2
      switch (*_data) {
        case novatel::SYNC_2_LONG_HEADER:
          _buffer.push_back(*_data++);
          _header_length = sizeof(novatel::LongHeader);
          break;
        case novatel::SYNC_2_SHORT_HEADER:
          _buffer.push_back(*_data++);
          _header_length = sizeof(novatel::ShortHeader);
          break;
        default:
          _buffer.clear();
      }
    } else if (_header_length > 0) {  // Working on header.
      if (_buffer.size() < _header_length) {
        _buffer.push_back(*_data++);
      } else {
        if (_header_length == sizeof(novatel::LongHeader)) {
          _total_length = _header_length + novatel::CRC_LENGTH +
                          reinterpret_cast<novatel::LongHeader*>(_buffer.data())
                              ->message_length;
        } else if (_header_length == sizeof(novatel::ShortHeader)) {
          _total_length =
              _header_length + novatel::CRC_LENGTH +
              reinterpret_cast<novatel::ShortHeader*>(_buffer.data())
                  ->message_length;
        } else {
          ROS_ERROR("Incorrect _header_length. Should never reach here.");
          _buffer.clear();
        }
        _header_length = 0;
      }
    } else if (_total_length > 0) {
      if (_buffer.size() < _total_length) {  // Working on body.
        _buffer.push_back(*_data++);
        continue;
      }
      MessageType type = prepare_message(message_ptr);
      _buffer.clear();
      _total_length = 0;
      if (type != MessageType::NONE) {
        return type;
      }
    }
  }
  return MessageType::NONE;
}

bool NovatelParser::check_crc() {
  size_t l = _buffer.size() - novatel::CRC_LENGTH;
  return crc32_block(_buffer.data(), l) ==
         *reinterpret_cast<uint32_t*>(_buffer.data() + l);
}

Parser::MessageType NovatelParser::prepare_message(MessagePtr& message_ptr) {
  if (!check_crc()) {
    ROS_ERROR("CRC check failed.");
    return MessageType::NONE;
  }

  uint8_t* message = nullptr;
  novatel::MessageId message_id;
  uint16_t message_length;
  uint16_t gps_week;
  uint32_t gps_millisecs;
  if (_buffer[2] == novatel::SYNC_2_LONG_HEADER) {
    auto header = reinterpret_cast<const novatel::LongHeader*>(_buffer.data());
    message = _buffer.data() + sizeof(novatel::LongHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
  } else {
    auto header = reinterpret_cast<const novatel::ShortHeader*>(_buffer.data());
    message = _buffer.data() + sizeof(novatel::ShortHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
  }
  switch (message_id) {
    case novatel::BESTGNSSPOS:
      if (message_length != sizeof(novatel::BestPos)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      if (handle_gnss_bestpos(reinterpret_cast<novatel::BestPos*>(message),
                              gps_week, gps_millisecs)) {
        message_ptr = &_bestpos;
        return MessageType::BEST_GNSS_POS;
      }
      break;

    case novatel::BESTPOS:
    case novatel::PSRPOS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::BestPos), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::BestPos)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      if (handle_best_pos(reinterpret_cast<novatel::BestPos*>(message),
                          gps_week, gps_millisecs)) {
        message_ptr = &_gnss;
        return MessageType::GNSS;
      }
      break;

    case novatel::BESTGNSSVEL:
    case novatel::BESTVEL:
    case novatel::PSRVEL:
      // ROS_ERROR_COND(message_length != sizeof(novatel::BestVel), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::BestVel)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }
      if (handle_best_vel(reinterpret_cast<novatel::BestVel*>(message),
                          gps_week, gps_millisecs)) {
        message_ptr = &_gnss;
        return MessageType::GNSS;
      }
      break;

    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::CorrImuData),
      // "Incorrect message_length");
      if (message_length != sizeof(novatel::CorrImuData)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }

      if (handle_corr_imu_data(
              reinterpret_cast<novatel::CorrImuData*>(message))) {
        message_ptr = &_ins;
        return MessageType::INS;
      }
      break;

    case novatel::INSCOV:
    case novatel::INSCOVS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::InsCov), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::InsCov)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }

      if (handle_ins_cov(reinterpret_cast<novatel::InsCov*>(message))) {
        message_ptr = &_ins;
        return MessageType::INS;
      }
      break;

    case novatel::INSPVA:
    case novatel::INSPVAS:
      // ROS_ERROR_COND(message_length != sizeof(novatel::InsPva), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::InsPva)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }

      if (handle_ins_pva(reinterpret_cast<novatel::InsPva*>(message))) {
        message_ptr = &_ins;
        return MessageType::INS;
      }
      break;

    case novatel::RAWIMUX:
    case novatel::RAWIMUSX:
      // ROS_ERROR_COND(message_length != sizeof(novatel::RawImuX), "Incorrect
      // message_length");
      if (message_length != sizeof(novatel::RawImuX)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }

      if (handle_raw_imu_x(reinterpret_cast<novatel::RawImuX*>(message))) {
        message_ptr = &_imu;
        return MessageType::IMU;
      }
      break;

    case novatel::RAWIMU:
    case novatel::RAWIMUS:
      if (message_length != sizeof(novatel::RawImu)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }

      if (handle_raw_imu(reinterpret_cast<novatel::RawImu*>(message))) {
        message_ptr = &_imu;
        return MessageType::IMU;
      }
      break;

    case novatel::INSPVAX:
      if (message_length != sizeof(novatel::InsPvaX)) {
        ROS_ERROR("Incorrect message_length");
        break;
      }

      if (handle_ins_pvax(reinterpret_cast<novatel::InsPvaX*>(message),
                          gps_week, gps_millisecs)) {
        message_ptr = &_ins_stat;
        return MessageType::INS_STAT;
      }
      break;

    case novatel::BDSEPHEMERIS:
      if (message_length != sizeof(novatel::BDS_Ephemeris)) {
        ROS_ERROR("Incorrect BDSEPHEMERIS message_length");
        break;
      }
      if (handle_bds_eph(reinterpret_cast<novatel::BDS_Ephemeris*>(message))) {
        message_ptr = &_gnss_ephemeris;
        return MessageType::BDSEPHEMERIDES;
      }
      break;

    case novatel::GPSEPHEMERIS:
      if (message_length != sizeof(novatel::GPS_Ephemeris)) {
        ROS_ERROR("Incorrect GPSEPHEMERIS message_length");
        break;
      }
      if (handle_gps_eph(reinterpret_cast<novatel::GPS_Ephemeris*>(message))) {
        message_ptr = &_gnss_ephemeris;
        return MessageType::GPSEPHEMERIDES;
      }
      break;

    case novatel::GLOEPHEMERIS:
      if (message_length != sizeof(novatel::GLO_Ephemeris)) {
        ROS_ERROR("Incorrect GLOEPHEMERIS message length");
        break;
      }
      if (handle_glo_eph(reinterpret_cast<novatel::GLO_Ephemeris*>(message))) {
        message_ptr = &_gnss_ephemeris;
        return MessageType::GLOEPHEMERIDES;
      }
      break;

    case novatel::RANGE:
      if (decode_gnss_observation(_buffer.data(),
                                  _buffer.data() + _buffer.size())) {
        message_ptr = &_gnss_observation;
        return MessageType::OBSERVATION;
      }
      break;

    default:
      break;
  }
  return MessageType::NONE;
}

bool NovatelParser::handle_gnss_bestpos(const novatel::BestPos* pos,
                                        uint16_t gps_week,
                                        uint32_t gps_millisecs) {
  _bestpos.set_sol_status(
      static_cast<apollo::drivers::gnss::SolutionStatus>(pos->solution_status));
  _bestpos.set_sol_type(
      static_cast<apollo::drivers::gnss::SolutionType>(pos->position_type));
  _bestpos.set_latitude(pos->latitude);
  _bestpos.set_longitude(pos->longitude);
  _bestpos.set_height_msl(pos->height_msl);
  _bestpos.set_undulation(pos->undulation);
  _bestpos.set_datum_id(
      static_cast<apollo::drivers::gnss::DatumId>(pos->datum_id));
  _bestpos.set_latitude_std_dev(pos->latitude_std_dev);
  _bestpos.set_longitude_std_dev(pos->longitude_std_dev);
  _bestpos.set_height_std_dev(pos->height_std_dev);
  _bestpos.set_base_station_id(pos->base_station_id);
  _bestpos.set_differential_age(pos->differential_age);
  _bestpos.set_solution_age(pos->solution_age);
  _bestpos.set_num_sats_tracked(pos->num_sats_tracked);
  _bestpos.set_num_sats_in_solution(pos->num_sats_in_solution);
  _bestpos.set_num_sats_l1(pos->num_sats_l1);
  _bestpos.set_num_sats_multi(pos->num_sats_multi);
  _bestpos.set_extended_solution_status(pos->extended_solution_status);
  _bestpos.set_galileo_beidou_used_mask(pos->galileo_beidou_used_mask);
  _bestpos.set_gps_glonass_used_mask(pos->gps_glonass_used_mask);

  double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  _bestpos.set_measurement_time(seconds);
  ROS_INFO_STREAM("Best gnss pose:\r\n" << _bestpos.DebugString());
  return true;
}

bool NovatelParser::handle_best_pos(const novatel::BestPos* pos,
                                    uint16_t gps_week, uint32_t gps_millisecs) {
  _gnss.mutable_position()->set_lon(pos->longitude);
  _gnss.mutable_position()->set_lat(pos->latitude);
  _gnss.mutable_position()->set_height(pos->height_msl + pos->undulation);
  _gnss.mutable_position_std_dev()->set_x(pos->longitude_std_dev);
  _gnss.mutable_position_std_dev()->set_y(pos->latitude_std_dev);
  _gnss.mutable_position_std_dev()->set_z(pos->height_std_dev);
  _gnss.set_num_sats(pos->num_sats_in_solution);
  if (_solution_status != pos->solution_status) {
    _solution_status = pos->solution_status;
    ROS_INFO_STREAM("Solution status: " << static_cast<int>(_solution_status));
  }
  if (_position_type != pos->position_type) {
    _position_type = pos->position_type;
    ROS_INFO_STREAM("Position type: " << static_cast<int>(_position_type));
  }
  _gnss.set_solution_status(static_cast<uint32_t>(pos->solution_status));
  if (pos->solution_status == novatel::SolutionStatus::SOL_COMPUTED) {
    _gnss.set_position_type(static_cast<uint32_t>(pos->position_type));
    switch (pos->position_type) {
      case novatel::SolutionType::SINGLE:
      case novatel::SolutionType::INS_PSRSP:
        _gnss.set_type(apollo::drivers::gnss::Gnss::SINGLE);
        break;
      case novatel::SolutionType::PSRDIFF:
      case novatel::SolutionType::WAAS:
      case novatel::SolutionType::INS_SBAS:
        _gnss.set_type(apollo::drivers::gnss::Gnss::PSRDIFF);
        break;
      case novatel::SolutionType::FLOATCONV:
      case novatel::SolutionType::L1_FLOAT:
      case novatel::SolutionType::IONOFREE_FLOAT:
      case novatel::SolutionType::NARROW_FLOAT:
      case novatel::SolutionType::RTK_DIRECT_INS:
      case novatel::SolutionType::INS_RTKFLOAT:
        _gnss.set_type(apollo::drivers::gnss::Gnss::RTK_FLOAT);
        break;
      case novatel::SolutionType::WIDELANE:
      case novatel::SolutionType::NARROWLANE:
      case novatel::SolutionType::L1_INT:
      case novatel::SolutionType::WIDE_INT:
      case novatel::SolutionType::NARROW_INT:
      case novatel::SolutionType::INS_RTKFIXED:
        _gnss.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
        break;
      case novatel::SolutionType::OMNISTAR:
      case novatel::SolutionType::INS_OMNISTAR:
      case novatel::SolutionType::INS_OMNISTAR_HP:
      case novatel::SolutionType::INS_OMNISTAR_XP:
      case novatel::SolutionType::OMNISTAR_HP:
      case novatel::SolutionType::OMNISTAR_XP:
      case novatel::SolutionType::PPP_CONVERGING:
      case novatel::SolutionType::PPP:
      case novatel::SolutionType::INS_PPP_CONVERGING:
      case novatel::SolutionType::INS_PPP:
        _gnss.set_type(apollo::drivers::gnss::Gnss::PPP);
        break;
      case novatel::SolutionType::PROPOGATED:
        _gnss.set_type(apollo::drivers::gnss::Gnss::PROPAGATED);
        break;
      default:
        _gnss.set_type(apollo::drivers::gnss::Gnss::INVALID);
    }
  } else {
    _gnss.set_type(apollo::drivers::gnss::Gnss::INVALID);
    _gnss.set_position_type(0);
  }
  if (pos->datum_id != novatel::DatumId::WGS84) {
    ROS_ERROR_STREAM_THROTTLE(
        5, "Unexpected Datum Id: " << static_cast<int>(pos->datum_id));
  }

  double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  if (_gnss.measurement_time() != seconds) {
    _gnss.set_measurement_time(seconds);
    return false;
  }
  return true;
}

bool NovatelParser::handle_best_vel(const novatel::BestVel* vel,
                                    uint16_t gps_week, uint32_t gps_millisecs) {
  if (_velocity_type != vel->velocity_type) {
    _velocity_type = vel->velocity_type;
    ROS_INFO_STREAM("Velocity type: " << static_cast<int>(_velocity_type));
  }
  if (!_gnss.has_velocity_latency() ||
      _gnss.velocity_latency() != vel->latency) {
    ROS_INFO_STREAM("Velocity latency: " << static_cast<int>(vel->latency));
    _gnss.set_velocity_latency(vel->latency);
  }
  double yaw = azimuth_deg_to_yaw_rad(vel->track_over_ground);
  _gnss.mutable_linear_velocity()->set_x(vel->horizontal_speed * cos(yaw));
  _gnss.mutable_linear_velocity()->set_y(vel->horizontal_speed * sin(yaw));
  _gnss.mutable_linear_velocity()->set_z(vel->vertical_speed);

  double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  if (_gnss.measurement_time() != seconds) {
    _gnss.set_measurement_time(seconds);
    return false;
  }
  return true;
}

bool NovatelParser::handle_corr_imu_data(const novatel::CorrImuData* imu) {
  rfu_to_flu(imu->x_velocity_change * _imu_measurement_hz,
             imu->y_velocity_change * _imu_measurement_hz,
             imu->z_velocity_change * _imu_measurement_hz,
             _ins.mutable_linear_acceleration());
  rfu_to_flu(imu->x_angle_change * _imu_measurement_hz,
             imu->y_angle_change * _imu_measurement_hz,
             imu->z_angle_change * _imu_measurement_hz,
             _ins.mutable_angular_velocity());

  double seconds = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
  if (_ins.measurement_time() != seconds) {
    _ins.set_measurement_time(seconds);
    return false;
  }
  _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  return true;
}

bool NovatelParser::handle_ins_cov(const novatel::InsCov* cov) {
  for (int i = 0; i < 9; ++i) {
    _ins.set_position_covariance(i, cov->position_covariance[i]);
    _ins.set_euler_angles_covariance(
        INDEX[i], (DEG_TO_RAD * DEG_TO_RAD) * cov->attitude_covariance[i]);
    _ins.set_linear_velocity_covariance(i, cov->velocity_covariance[i]);
  }
  return false;
}

bool NovatelParser::handle_ins_pva(const novatel::InsPva* pva) {
  if (_ins_status != pva->status) {
    _ins_status = pva->status;
    ROS_INFO_STREAM("INS status: " << static_cast<int>(_ins_status));
  }
  _ins.mutable_position()->set_lon(pva->longitude);
  _ins.mutable_position()->set_lat(pva->latitude);
  _ins.mutable_position()->set_height(pva->height);
  _ins.mutable_euler_angles()->set_x(pva->roll * DEG_TO_RAD);
  _ins.mutable_euler_angles()->set_y(-pva->pitch * DEG_TO_RAD);
  _ins.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(pva->azimuth));
  _ins.mutable_linear_velocity()->set_x(pva->east_velocity);
  _ins.mutable_linear_velocity()->set_y(pva->north_velocity);
  _ins.mutable_linear_velocity()->set_z(pva->up_velocity);

  switch (pva->status) {
    case novatel::InsStatus::ALIGNMENT_COMPLETE:
    case novatel::InsStatus::SOLUTION_GOOD:
      _ins.set_type(apollo::drivers::gnss::Ins::GOOD);
      break;
    case novatel::InsStatus::ALIGNING:
    case novatel::InsStatus::HIGH_VARIANCE:
    case novatel::InsStatus::SOLUTION_FREE:
      _ins.set_type(apollo::drivers::gnss::Ins::CONVERGING);
      break;
    default:
      _ins.set_type(apollo::drivers::gnss::Ins::INVALID);
  }

  double seconds = pva->gps_week * SECONDS_PER_WEEK + pva->gps_seconds;
  if (_ins.measurement_time() != seconds) {
    _ins.set_measurement_time(seconds);
    return false;
  }

  _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  return true;
}

bool NovatelParser::handle_ins_pvax(const novatel::InsPvaX* pvax,
                                    uint16_t gps_week, uint32_t gps_millisecs) {
  double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  double unix_sec = apollo::drivers::util::gps2unix(seconds);
  _ins_stat.mutable_header()->set_timestamp_sec(unix_sec);
  _ins_stat.set_ins_status(pvax->ins_status);
  _ins_stat.set_pos_type(pvax->pos_type);
  return true;
}

bool NovatelParser::handle_raw_imu_x(const novatel::RawImuX* imu) {
  if (imu->imu_error != 0) {
    ROS_WARN_STREAM("IMU error. Status: " << std::hex << std::showbase
                                          << imu->imuStatus);
  }
  if (is_zero(_gyro_scale)) {
    novatel::ImuParameter param = novatel::get_imu_parameter(imu->imu_type);
    ROS_INFO_STREAM("IMU type: " << static_cast<unsigned>(imu->imu_type) << "; "
                                 << "Gyro scale: " << param.gyro_scale << "; "
                                 << "Accel scale: " << param.accel_scale << "; "
                                 << "Sampling rate: " << param.sampling_rate_hz
                                 << ".");

    if (is_zero(param.sampling_rate_hz)) {
      ROS_ERROR_STREAM_THROTTLE(
          5, "Unsupported IMU type: " << static_cast<int>(imu->imu_type));
      return false;
    }
    _gyro_scale = param.gyro_scale * param.sampling_rate_hz;
    _accel_scale = param.accel_scale * param.sampling_rate_hz;
    _imu_measurement_hz = param.sampling_rate_hz;
    _imu_measurement_span = 1.0 / param.sampling_rate_hz;
    _imu.set_measurement_span(_imu_measurement_span);
  }

  double time = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
  if (_imu_measurement_time_previous > 0.0 &&
      fabs(time - _imu_measurement_time_previous - _imu_measurement_span) >
          1e-4) {
    ROS_WARN_STREAM("Unexpected delay between two IMU measurements at: "
                    << time - _imu_measurement_time_previous);
  }
  _imu.set_measurement_time(time);
  switch (_imu_frame_mapping) {
    case 5:  // Default mapping.
      rfu_to_flu(imu->x_velocity_change * _accel_scale,
                 -imu->y_velocity_change_neg * _accel_scale,
                 imu->z_velocity_change * _accel_scale,
                 _imu.mutable_linear_acceleration());
      rfu_to_flu(imu->x_angle_change * _gyro_scale,
                 -imu->y_angle_change_neg * _gyro_scale,
                 imu->z_angle_change * _gyro_scale,
                 _imu.mutable_angular_velocity());
      break;
    case 6:
      rfu_to_flu(-imu->y_velocity_change_neg * _accel_scale,
                 imu->x_velocity_change * _accel_scale,
                 -imu->z_velocity_change * _accel_scale,
                 _imu.mutable_linear_acceleration());
      rfu_to_flu(-imu->y_angle_change_neg * _gyro_scale,
                 imu->x_angle_change * _gyro_scale,
                 -imu->z_angle_change * _gyro_scale,
                 _imu.mutable_angular_velocity());
      break;
    default:
      ROS_ERROR_STREAM_THROTTLE(
          5, "Unsupported IMU frame mapping: " << _imu_frame_mapping);
  }
  _imu_measurement_time_previous = time;
  return true;
}

bool NovatelParser::handle_raw_imu(const novatel::RawImu* imu) {
  double gyro_scale = 0.0;
  double accel_scale = 0.0;
  float imu_measurement_span = 1.0 / 200.0;

  if (is_zero(_gyro_scale)) {
    novatel::ImuParameter param = novatel::get_imu_parameter(
                                    novatel::ImuType::ADIS16488);
    //ROS_INFO_STREAM("IMU type: " << static_cast<unsigned>(imu->imu_type) << "; "
    //                             << "Gyro scale: " << param.gyro_scale << "; "
    //                             << "Accel scale: " << param.accel_scale << "; "
    //                             << "Sampling rate: " << param.sampling_rate_hz
    //                             << ".");

    if (is_zero(param.sampling_rate_hz)) {
      ROS_ERROR_STREAM_THROTTLE(
          5, "Unsupported IMU type ADUS16488.");
      return false;
    }
    gyro_scale = param.gyro_scale * param.sampling_rate_hz;
    accel_scale = param.accel_scale * param.sampling_rate_hz;
    imu_measurement_span = 1.0 / param.sampling_rate_hz;
    _imu.set_measurement_span(imu_measurement_span);
  } else {
    gyro_scale = _gyro_scale;
    accel_scale = _accel_scale;
    imu_measurement_span = _imu_measurement_span;
  }

  double time = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
  if (_imu_measurement_time_previous > 0.0 &&
      fabs(time - _imu_measurement_time_previous - imu_measurement_span) >
          1e-4) {
    ROS_WARN_STREAM("Unexpected delay between two IMU measurements at: "
                    << time - _imu_measurement_time_previous);
  }

  _imu.set_measurement_time(time);
  switch (_imu_frame_mapping) {
    case 5:  // Default mapping.
      rfu_to_flu(imu->x_velocity_change * accel_scale,
                 -imu->y_velocity_change_neg * accel_scale,
                 imu->z_velocity_change * accel_scale,
                 _imu.mutable_linear_acceleration());
      rfu_to_flu(imu->x_angle_change * gyro_scale,
                 -imu->y_angle_change_neg * gyro_scale,
                 imu->z_angle_change * gyro_scale,
                 _imu.mutable_angular_velocity());
      break;
    case 6:
      rfu_to_flu(-imu->y_velocity_change_neg * accel_scale,
                 imu->x_velocity_change * accel_scale,
                 -imu->z_velocity_change * accel_scale,
                 _imu.mutable_linear_acceleration());
      rfu_to_flu(-imu->y_angle_change_neg * gyro_scale,
                 imu->x_angle_change * gyro_scale,
                 -imu->z_angle_change * gyro_scale,
                 _imu.mutable_angular_velocity());
      break;
    default:
      ROS_ERROR_STREAM_THROTTLE(
          5, "Unsupported IMU frame mapping: " << _imu_frame_mapping);
  }
  _imu_measurement_time_previous = time;
  return true;
}

bool NovatelParser::handle_gps_eph(const novatel::GPS_Ephemeris* gps_emph) {
  _gnss_ephemeris.set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      _gnss_ephemeris.mutable_keppler_orbit();

  keppler_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);
  keppler_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::GPS_TIME);
  keppler_orbit->set_sat_prn(gps_emph->prn);
  keppler_orbit->set_week_num(gps_emph->week);
  keppler_orbit->set_af0(gps_emph->af0);
  keppler_orbit->set_af1(gps_emph->af1);
  keppler_orbit->set_af2(gps_emph->af2);
  keppler_orbit->set_iode(gps_emph->iode1);
  keppler_orbit->set_deltan(gps_emph->delta_A);
  keppler_orbit->set_m0(gps_emph->M_0);
  keppler_orbit->set_e(gps_emph->ecc);
  keppler_orbit->set_roota(sqrt(gps_emph->A));
  keppler_orbit->set_toe(gps_emph->toe);
  keppler_orbit->set_toc(gps_emph->toc);
  keppler_orbit->set_cic(gps_emph->cic);
  keppler_orbit->set_crc(gps_emph->crc);
  keppler_orbit->set_cis(gps_emph->cis);
  keppler_orbit->set_crs(gps_emph->crs);
  keppler_orbit->set_cuc(gps_emph->cuc);
  keppler_orbit->set_cus(gps_emph->cus);
  keppler_orbit->set_omega0(gps_emph->omega_0);
  keppler_orbit->set_omega(gps_emph->omega);
  keppler_orbit->set_i0(gps_emph->I_0);
  keppler_orbit->set_omegadot(gps_emph->dot_omega);
  keppler_orbit->set_idot(gps_emph->dot_I);
  keppler_orbit->set_accuracy(sqrt(gps_emph->ura));
  keppler_orbit->set_health(gps_emph->health);
  keppler_orbit->set_tgd(gps_emph->tgd);
  keppler_orbit->set_iodc(gps_emph->iodc);
  return true;
}

bool NovatelParser::handle_bds_eph(const novatel::BDS_Ephemeris* bds_emph) {
  _gnss_ephemeris.set_gnss_type(apollo::drivers::gnss::GnssType::BDS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      _gnss_ephemeris.mutable_keppler_orbit();

  keppler_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::BDS_SYS);
  keppler_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::BDS_TIME);
  keppler_orbit->set_sat_prn(bds_emph->satellite_id);
  keppler_orbit->set_week_num(bds_emph->week);
  keppler_orbit->set_af0(bds_emph->a0);
  keppler_orbit->set_af1(bds_emph->a1);
  keppler_orbit->set_af2(bds_emph->a2);
  keppler_orbit->set_iode(bds_emph->aode);
  keppler_orbit->set_deltan(bds_emph->delta_N);
  keppler_orbit->set_m0(bds_emph->M0);
  keppler_orbit->set_e(bds_emph->ecc);
  keppler_orbit->set_roota(bds_emph->rootA);
  keppler_orbit->set_toe(bds_emph->toe);
  keppler_orbit->set_toc(bds_emph->toc);
  keppler_orbit->set_cic(bds_emph->cic);
  keppler_orbit->set_crc(bds_emph->crc);
  keppler_orbit->set_cis(bds_emph->cis);
  keppler_orbit->set_crs(bds_emph->crs);
  keppler_orbit->set_cuc(bds_emph->cuc);
  keppler_orbit->set_cus(bds_emph->cus);
  keppler_orbit->set_omega0(bds_emph->omega0);
  keppler_orbit->set_omega(bds_emph->omega);
  keppler_orbit->set_i0(bds_emph->inc_angle);
  keppler_orbit->set_omegadot(bds_emph->rra);
  keppler_orbit->set_idot(bds_emph->idot);
  keppler_orbit->set_accuracy(bds_emph->ura);
  keppler_orbit->set_health(bds_emph->health1);
  keppler_orbit->set_tgd(bds_emph->tdg1);
  keppler_orbit->set_iodc(bds_emph->aodc);
  return true;
}

bool NovatelParser::handle_glo_eph(const novatel::GLO_Ephemeris* glo_emph) {
  _gnss_ephemeris.set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);

  apollo::drivers::gnss::GlonassOrbit* glonass_orbit =
      _gnss_ephemeris.mutable_glonass_orbit();
  glonass_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);
  glonass_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::GLO_TIME);
  glonass_orbit->set_slot_prn(glo_emph->sloto - 37);
  glonass_orbit->set_toe(glo_emph->e_time / 1000);
  glonass_orbit->set_frequency_no(glo_emph->freqo - 7);
  glonass_orbit->set_week_num(glo_emph->e_week);
  glonass_orbit->set_week_second_s(glo_emph->e_time / 1000);
  glonass_orbit->set_tk(glo_emph->Tk);
  glonass_orbit->set_clock_offset(-glo_emph->tau_n);
  glonass_orbit->set_clock_drift(glo_emph->gamma);

  if (glo_emph->health <= 3) {
    glonass_orbit->set_health(0);  // 0 means good.
  } else {
    glonass_orbit->set_health(1);  // 1 means bad.
  }
  glonass_orbit->set_position_x(glo_emph->pos_x);
  glonass_orbit->set_position_y(glo_emph->pos_y);
  glonass_orbit->set_position_z(glo_emph->pos_z);

  glonass_orbit->set_velocity_x(glo_emph->vel_x);
  glonass_orbit->set_velocity_y(glo_emph->vel_y);
  glonass_orbit->set_velocity_z(glo_emph->vel_z);

  glonass_orbit->set_accelerate_x(glo_emph->acc_x);
  glonass_orbit->set_accelerate_y(glo_emph->acc_y);
  glonass_orbit->set_accelerate_z(glo_emph->acc_z);

  glonass_orbit->set_infor_age(glo_emph->age);

  return true;
}

void NovatelParser::set_observation_time() {
  int week = 0;
  double second = 0.0;

  second = time2gpst(_raw.time, &week);
  _gnss_observation.set_gnss_time_type(apollo::drivers::gnss::GPS_TIME);
  _gnss_observation.set_gnss_week(week);
  _gnss_observation.set_gnss_second_s(second);
}

bool NovatelParser::decode_gnss_observation(const uint8_t* obs_data,
                                            const uint8_t* obs_data_end) {
  int status = 0;
  while (obs_data < obs_data_end) {
    status = input_oem4(&_raw, *obs_data++);
    switch (status) {
      case 1:  // observation data
        if (_raw.obs.n == 0) {
          ROS_WARN("Obs is zero");
        }

        _gnss_observation.Clear();
        _gnss_observation.set_receiver_id(0);
        set_observation_time();
        _gnss_observation.set_sat_obs_num(_raw.obs.n);
        for (int i = 0; i < _raw.obs.n; ++i) {
          int prn = 0;
          int sys = 0;

          sys = satsys(_raw.obs.data[i].sat, &prn);
          ROS_INFO("sys %d, prn %d", sys, prn);

          apollo::drivers::gnss::GnssType gnss_type;
          if (!gnss_sys_type(sys, gnss_type)) {
            break;
          }

          auto sat_obs = _gnss_observation.add_sat_obs();  // create obj
          sat_obs->set_sat_prn(prn);
          sat_obs->set_sat_sys(gnss_type);

          int j = 0;
          for (j = 0; j < NFREQ + NEXOBS; ++j) {
            if (is_zero(_raw.obs.data[i].L[j])) {
              break;
            }

            apollo::drivers::gnss::GnssBandID baud_id;
            if (!gnss_baud_id(gnss_type, j, baud_id)) {
              break;
            }

            double freq = 0;
            gnss_frequence(baud_id, freq);
            auto band_obs = sat_obs->add_band_obs();
            if (_raw.obs.data[i].code[i] == CODE_L1C) {
              band_obs->set_pseudo_type(
                  apollo::drivers::gnss::PseudoType::CORSE_CODE);
            } else if (_raw.obs.data[i].code[i] == CODE_L1P) {
              band_obs->set_pseudo_type(
                  apollo::drivers::gnss::PseudoType::PRECISION_CODE);
            } else {
              ROS_INFO("Code %d, in seq %d, gnss type %d.",
                       _raw.obs.data[i].code[i], j,
                       static_cast<int>(gnss_type));
            }

            band_obs->set_band_id(baud_id);
            band_obs->set_frequency_value(freq);
            band_obs->set_pseudo_range(_raw.obs.data[i].P[j]);
            band_obs->set_carrier_phase(_raw.obs.data[i].L[j]);
            band_obs->set_loss_lock_index(_raw.obs.data[i].SNR[j]);
            band_obs->set_doppler(_raw.obs.data[i].D[j]);
            band_obs->set_snr(_raw.obs.data[i].SNR[j]);
            band_obs->set_snr(_raw.obs.data[i].SNR[j]);
          }
          ROS_INFO("Baud obs num %d.", j);
          sat_obs->set_band_obs_num(j);
        }
        ROS_INFO_STREAM("Observation debuginfo:\r\n"
                        << _gnss_observation.DebugString());
        return true;

      default:
        break;
    }
  }
  return false;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
