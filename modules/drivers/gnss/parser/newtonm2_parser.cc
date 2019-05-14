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

#include "modules/drivers/gnss/parser/newtonm2_parser.h"

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

Parser* Parser::CreateNewtonM2(const config::Config& config) {
  return new NewtonM2Parser(config);
}

NewtonM2Parser::NewtonM2Parser() {
  buffer_.reserve(newtonm2::BUFFER_SIZE);
  ins_.mutable_position_covariance()->Resize(9, newtonm2::FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, newtonm2::FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, newtonm2::FLOAT_NAN);

  if (1 != init_raw(&raw_)) {
    AFATAL << "memory allocation error for observation data structure.";
  }
}

NewtonM2Parser::NewtonM2Parser(const config::Config& config) {
  buffer_.reserve(newtonm2::BUFFER_SIZE);
  ins_.mutable_position_covariance()->Resize(9, newtonm2::FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, newtonm2::FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, newtonm2::FLOAT_NAN);

  if (config.has_imu_type()) {
    imu_type_ = config.imu_type();
  }

  if (1 != init_raw(&raw_)) {
    AFATAL << "memory allocation error for observation data structure.";
  }
}

Parser::MessageType NewtonM2Parser::GetMessage(MessagePtr* message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  while (data_ < data_end_) {
    if (buffer_.empty()) {  // Looking for SYNC0
      if (*data_ == novatel::SYNC_0) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for SYNC1
      if (*data_ == novatel::SYNC_1) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 2) {  // Looking for SYNC2
      switch (*data_) {
        case novatel::SYNC_2_LONG_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::LongHeader);
          break;
        case novatel::SYNC_2_SHORT_HEADER:
          buffer_.push_back(*data_++);
          header_length_ = sizeof(novatel::ShortHeader);
          break;
        default:
          buffer_.clear();
      }
    } else if (header_length_ > 0) {  // Working on header.
      if (buffer_.size() < header_length_) {
        buffer_.push_back(*data_++);
      } else {
        if (header_length_ == sizeof(novatel::LongHeader)) {
          total_length_ = header_length_ + novatel::CRC_LENGTH +
                          reinterpret_cast<novatel::LongHeader*>(buffer_.data())
                              ->message_length;
        } else if (header_length_ == sizeof(novatel::ShortHeader)) {
          total_length_ =
              header_length_ + novatel::CRC_LENGTH +
              reinterpret_cast<novatel::ShortHeader*>(buffer_.data())
                  ->message_length;
        } else {
          AERROR << "Incorrect header_length_. Should never reach here.";
          buffer_.clear();
        }
        header_length_ = 0;
      }
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) {  // Working on body.
        buffer_.push_back(*data_++);
        continue;
      }
      MessageType type = PrepareMessage(message_ptr);
      buffer_.clear();
      total_length_ = 0;
      if (type != MessageType::NONE) {
        return type;
      }
    }
  }
  return MessageType::NONE;
}

bool NewtonM2Parser::check_crc() {
  size_t l = buffer_.size() - novatel::CRC_LENGTH;
  return newtonm2::crc32_block(buffer_.data(), l) ==
         *reinterpret_cast<uint32_t*>(buffer_.data() + l);
}

Parser::MessageType NewtonM2Parser::PrepareMessage(MessagePtr* message_ptr) {
  if (!check_crc()) {
    AERROR << "CRC check failed.";
    return MessageType::NONE;
  }

  uint8_t* message = nullptr;
  novatel::MessageId message_id;
  uint16_t message_length;
  uint16_t gps_week;
  uint32_t gps_millisecs;
  if (buffer_[2] == novatel::SYNC_2_LONG_HEADER) {
    auto header = reinterpret_cast<const novatel::LongHeader*>(buffer_.data());
    message = buffer_.data() + sizeof(novatel::LongHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
  } else {
    auto header = reinterpret_cast<const novatel::ShortHeader*>(buffer_.data());
    message = buffer_.data() + sizeof(novatel::ShortHeader);
    gps_week = header->gps_week;
    gps_millisecs = header->gps_millisecs;
    message_id = header->message_id;
    message_length = header->message_length;
  }
  switch (message_id) {
    case novatel::BESTGNSSPOS:
      if (message_length != sizeof(novatel::BestPos)) {
        AERROR << "Incorrect message_length";
        break;
      }
      if (HandleGnssBestpos(reinterpret_cast<novatel::BestPos*>(message),
                            gps_week, gps_millisecs)) {
        *message_ptr = &bestpos_;
        return MessageType::BEST_GNSS_POS;
      }
      break;

    case novatel::BESTPOS:
    case novatel::PSRPOS:
      if (message_length != sizeof(novatel::BestPos)) {
        AERROR << "Incorrect message_length";
        break;
      }
      if (HandleBestPos(reinterpret_cast<novatel::BestPos*>(message), gps_week,
                        gps_millisecs)) {
        *message_ptr = &gnss_;
        return MessageType::GNSS;
      }
      break;

    case novatel::BESTGNSSVEL:
    case novatel::BESTVEL:
    case novatel::PSRVEL:
      if (message_length != sizeof(novatel::BestVel)) {
        AERROR << "Incorrect message_length";
        break;
      }
      if (HandleBestVel(reinterpret_cast<novatel::BestVel*>(message), gps_week,
                        gps_millisecs)) {
        *message_ptr = &gnss_;
        return MessageType::GNSS;
      }
      break;

    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
      if (message_length != sizeof(novatel::CorrImuData)) {
        AERROR << "Incorrect message_length";
        break;
      }

      if (HandleCorrImuData(reinterpret_cast<novatel::CorrImuData*>(message))) {
        *message_ptr = &ins_;
        return MessageType::INS;
      }
      break;

    case novatel::INSCOV:
    case novatel::INSCOVS:
      if (message_length != sizeof(novatel::InsCov)) {
        AERROR << "Incorrect message_length";
        break;
      }

      if (HandleInsCov(reinterpret_cast<novatel::InsCov*>(message))) {
        *message_ptr = &ins_;
        return MessageType::INS;
      }
      break;

    case novatel::INSPVA:
    case novatel::INSPVAS:
      if (message_length != sizeof(novatel::InsPva)) {
        AERROR << "Incorrect message_length";
        break;
      }

      if (HandleInsPva(reinterpret_cast<novatel::InsPva*>(message))) {
        *message_ptr = &ins_;
        return MessageType::INS;
      }
      break;

    case novatel::RAWIMUX:
    case novatel::RAWIMUSX:
      if (message_length != sizeof(novatel::RawImuX)) {
        AERROR << "Incorrect message_length";
        break;
      }

      if (HandleRawImuX(reinterpret_cast<novatel::RawImuX*>(message))) {
        *message_ptr = &imu_;
        return MessageType::IMU;
      }
      break;

    case novatel::RAWIMU:
    case novatel::RAWIMUS:
      if (message_length != sizeof(novatel::RawImu)) {
        AERROR << "Incorrect message_length";
        break;
      }

      if (HandleRawImu(reinterpret_cast<novatel::RawImu*>(message))) {
        *message_ptr = &imu_;
        return MessageType::IMU;
      }
      break;

    case novatel::INSPVAX:
      if (message_length != sizeof(novatel::InsPvaX)) {
        AERROR << "Incorrect message_length";
        break;
      }

      if (HandleInsPvax(reinterpret_cast<novatel::InsPvaX*>(message), gps_week,
                        gps_millisecs)) {
        *message_ptr = &ins_stat_;
        return MessageType::INS_STAT;
      }
      break;

    case novatel::BDSEPHEMERIS:
      if (message_length != sizeof(novatel::BDS_Ephemeris)) {
        AERROR << "Incorrect BDSEPHEMERIS message_length";
        break;
      }
      if (HandleBdsEph(reinterpret_cast<novatel::BDS_Ephemeris*>(message))) {
        *message_ptr = &gnss_ephemeris_;
        return MessageType::BDSEPHEMERIDES;
      }
      break;

    case novatel::GPSEPHEMERIS:
      if (message_length != sizeof(novatel::GPS_Ephemeris)) {
        AERROR << "Incorrect GPSEPHEMERIS message_length";
        break;
      }
      if (HandleGpsEph(reinterpret_cast<novatel::GPS_Ephemeris*>(message))) {
        *message_ptr = &gnss_ephemeris_;
        return MessageType::GPSEPHEMERIDES;
      }
      break;

    case novatel::GLOEPHEMERIS:
      if (message_length != sizeof(novatel::GLO_Ephemeris)) {
        AERROR << "Incorrect GLOEPHEMERIS message length";
        break;
      }
      if (HandleGloEph(reinterpret_cast<novatel::GLO_Ephemeris*>(message))) {
        *message_ptr = &gnss_ephemeris_;
        return MessageType::GLOEPHEMERIDES;
      }
      break;

    case novatel::RANGE:
      if (DecodeGnssObservation(buffer_.data(),
                                buffer_.data() + buffer_.size())) {
        *message_ptr = &gnss_observation_;
        return MessageType::OBSERVATION;
      }
      break;

    case novatel::HEADING:
      if (message_length != sizeof(novatel::Heading)) {
        AERROR << "Incorrect message_length";
        break;
      }
      if (HandleHeading(reinterpret_cast<novatel::Heading*>(message), gps_week,
                        gps_millisecs)) {
        *message_ptr = &heading_;
        return MessageType::HEADING;
      }
      break;

    default:
      break;
  }
  return MessageType::NONE;
}

bool NewtonM2Parser::HandleGnssBestpos(const novatel::BestPos* pos,
                                       uint16_t gps_week,
                                       uint32_t gps_millisecs) {
  bestpos_.set_sol_status(
      static_cast<apollo::drivers::gnss::SolutionStatus>(pos->solution_status));
  bestpos_.set_sol_type(
      static_cast<apollo::drivers::gnss::SolutionType>(pos->position_type));
  bestpos_.set_latitude(pos->latitude);
  bestpos_.set_longitude(pos->longitude);
  bestpos_.set_height_msl(pos->height_msl);
  bestpos_.set_undulation(pos->undulation);
  bestpos_.set_datum_id(
      static_cast<apollo::drivers::gnss::DatumId>(pos->datum_id));
  bestpos_.set_latitude_std_dev(pos->latitude_std_dev);
  bestpos_.set_longitude_std_dev(pos->longitude_std_dev);
  bestpos_.set_height_std_dev(pos->height_std_dev);
  bestpos_.set_base_station_id(pos->base_station_id);
  bestpos_.set_differential_age(pos->differential_age);
  bestpos_.set_solution_age(pos->solution_age);
  bestpos_.set_num_sats_tracked(pos->num_sats_tracked);
  bestpos_.set_num_sats_in_solution(pos->num_sats_in_solution);
  bestpos_.set_num_sats_l1(pos->num_sats_l1);
  bestpos_.set_num_sats_multi(pos->num_sats_multi);
  bestpos_.set_extended_solution_status(pos->extended_solution_status);
  bestpos_.set_galileo_beidou_used_mask(pos->galileo_beidou_used_mask);
  bestpos_.set_gps_glonass_used_mask(pos->gps_glonass_used_mask);

  double seconds = gps_week * newtonm2::SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  bestpos_.set_measurement_time(seconds);
  return true;
}

bool NewtonM2Parser::HandleBestPos(const novatel::BestPos* pos,
                                   uint16_t gps_week, uint32_t gps_millisecs) {
  gnss_.mutable_position()->set_lon(pos->longitude);
  gnss_.mutable_position()->set_lat(pos->latitude);
  gnss_.mutable_position()->set_height(pos->height_msl + pos->undulation);
  gnss_.mutable_position_std_dev()->set_x(pos->longitude_std_dev);
  gnss_.mutable_position_std_dev()->set_y(pos->latitude_std_dev);
  gnss_.mutable_position_std_dev()->set_z(pos->height_std_dev);
  gnss_.set_num_sats(pos->num_sats_in_solution);
  if (solution_status_ != pos->solution_status) {
    solution_status_ = pos->solution_status;
    AINFO << "Solution status: " << static_cast<int>(solution_status_);
  }
  if (position_type_ != pos->position_type) {
    position_type_ = pos->position_type;
    AINFO << "Position type: " << static_cast<int>(position_type_);
  }
  gnss_.set_solution_status(static_cast<uint32_t>(pos->solution_status));
  if (pos->solution_status == novatel::SolutionStatus::SOL_COMPUTED) {
    gnss_.set_position_type(static_cast<uint32_t>(pos->position_type));
    switch (pos->position_type) {
      case novatel::SolutionType::SINGLE:
      case novatel::SolutionType::INS_PSRSP:
        gnss_.set_type(apollo::drivers::gnss::Gnss::SINGLE);
        break;
      case novatel::SolutionType::PSRDIFF:
      case novatel::SolutionType::WAAS:
      case novatel::SolutionType::INS_SBAS:
        gnss_.set_type(apollo::drivers::gnss::Gnss::PSRDIFF);
        break;
      case novatel::SolutionType::FLOATCONV:
      case novatel::SolutionType::L1_FLOAT:
      case novatel::SolutionType::IONOFREE_FLOAT:
      case novatel::SolutionType::NARROW_FLOAT:
      case novatel::SolutionType::RTK_DIRECT_INS:
      case novatel::SolutionType::INS_RTKFLOAT:
        gnss_.set_type(apollo::drivers::gnss::Gnss::RTK_FLOAT);
        break;
      case novatel::SolutionType::WIDELANE:
      case novatel::SolutionType::NARROWLANE:
      case novatel::SolutionType::L1_INT:
      case novatel::SolutionType::WIDE_INT:
      case novatel::SolutionType::NARROW_INT:
      case novatel::SolutionType::INS_RTKFIXED:
        gnss_.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
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
        gnss_.set_type(apollo::drivers::gnss::Gnss::PPP);
        break;
      case novatel::SolutionType::PROPOGATED:
        gnss_.set_type(apollo::drivers::gnss::Gnss::PROPAGATED);
        break;
      default:
        gnss_.set_type(apollo::drivers::gnss::Gnss::INVALID);
    }
  } else {
    gnss_.set_type(apollo::drivers::gnss::Gnss::INVALID);
    gnss_.set_position_type(0);
  }
  if (pos->datum_id != novatel::DatumId::WGS84) {
    AERROR_EVERY(5) << "Unexpected Datum Id: "
                    << static_cast<int>(pos->datum_id);
  }

  double seconds = gps_week * newtonm2::SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  if (gnss_.measurement_time() != seconds) {
    gnss_.set_measurement_time(seconds);
    return false;
  }
  return true;
}

bool NewtonM2Parser::HandleBestVel(const novatel::BestVel* vel,
                                   uint16_t gps_week, uint32_t gps_millisecs) {
  if (velocity_type_ != vel->velocity_type) {
    velocity_type_ = vel->velocity_type;
    AINFO << "Velocity type: " << static_cast<int>(velocity_type_);
  }
  if (!gnss_.has_velocity_latency() ||
      gnss_.velocity_latency() != vel->latency) {
    AINFO << "Velocity latency: " << static_cast<int>(vel->latency);
    gnss_.set_velocity_latency(vel->latency);
  }
  double yaw = newtonm2::azimuth_deg_to_yaw_rad(vel->track_over_ground);
  gnss_.mutable_linear_velocity()->set_x(vel->horizontal_speed * cos(yaw));
  gnss_.mutable_linear_velocity()->set_y(vel->horizontal_speed * sin(yaw));
  gnss_.mutable_linear_velocity()->set_z(vel->vertical_speed);

  double seconds = gps_week * newtonm2::SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  if (gnss_.measurement_time() != seconds) {
    gnss_.set_measurement_time(seconds);
    return false;
  }
  return true;
}

bool NewtonM2Parser::HandleCorrImuData(const novatel::CorrImuData* imu) {
  newtonm2::rfu_to_flu(imu->x_velocity_change * imu_measurement_hz_,
                       imu->y_velocity_change * imu_measurement_hz_,
                       imu->z_velocity_change * imu_measurement_hz_,
                       ins_.mutable_linear_acceleration());
  newtonm2::rfu_to_flu(imu->x_angle_change * imu_measurement_hz_,
                       imu->y_angle_change * imu_measurement_hz_,
                       imu->z_angle_change * imu_measurement_hz_,
                       ins_.mutable_angular_velocity());

  double seconds =
      imu->gps_week * newtonm2::SECONDS_PER_WEEK + imu->gps_seconds;
  if (ins_.measurement_time() != seconds) {
    ins_.set_measurement_time(seconds);
    return false;
  }

  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  return true;
}

bool NewtonM2Parser::HandleInsCov(const novatel::InsCov* cov) {
  for (int i = 0; i < 9; ++i) {
    ins_.set_position_covariance(
        i, static_cast<float>(cov->position_covariance[i]));
    ins_.set_euler_angles_covariance(
        newtonm2::INDEX[i],
        static_cast<float>((newtonm2::DEG_TO_RAD_M2 * newtonm2::DEG_TO_RAD_M2) *
                           cov->attitude_covariance[i]));
    ins_.set_linear_velocity_covariance(
        i, static_cast<float>(cov->velocity_covariance[i]));
  }
  return false;
}

bool NewtonM2Parser::HandleInsPva(const novatel::InsPva* pva) {
  if (ins_status_ != pva->status) {
    ins_status_ = pva->status;
    AINFO << "INS status: " << static_cast<int>(ins_status_);
  }
  ins_.mutable_position()->set_lon(pva->longitude);
  ins_.mutable_position()->set_lat(pva->latitude);
  ins_.mutable_position()->set_height(pva->height);
  ins_.mutable_euler_angles()->set_x(pva->roll * newtonm2::DEG_TO_RAD_M2);
  ins_.mutable_euler_angles()->set_y(-pva->pitch * newtonm2::DEG_TO_RAD_M2);
  ins_.mutable_euler_angles()->set_z(
      newtonm2::azimuth_deg_to_yaw_rad(pva->azimuth));
  ins_.mutable_linear_velocity()->set_x(pva->east_velocity);
  ins_.mutable_linear_velocity()->set_y(pva->north_velocity);
  ins_.mutable_linear_velocity()->set_z(pva->up_velocity);

  switch (pva->status) {
    case novatel::InsStatus::ALIGNMENT_COMPLETE:
    case novatel::InsStatus::SOLUTION_GOOD:
      ins_.set_type(apollo::drivers::gnss::Ins::GOOD);
      break;
    case novatel::InsStatus::ALIGNING:
    case novatel::InsStatus::HIGH_VARIANCE:
    case novatel::InsStatus::SOLUTION_FREE:
      ins_.set_type(apollo::drivers::gnss::Ins::CONVERGING);
      break;
    default:
      ins_.set_type(apollo::drivers::gnss::Ins::INVALID);
  }

  double seconds =
      pva->gps_week * newtonm2::SECONDS_PER_WEEK + pva->gps_seconds;
  if (ins_.measurement_time() != seconds) {
    ins_.set_measurement_time(seconds);
    return false;
  }

  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  return true;
}

bool NewtonM2Parser::HandleInsPvax(const novatel::InsPvaX* pvax,
                                   uint16_t gps_week, uint32_t gps_millisecs) {
  double seconds = gps_week * newtonm2::SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  double unix_sec = apollo::drivers::util::gps2unix(seconds);
  ins_stat_.mutable_header()->set_timestamp_sec(unix_sec);
  ins_stat_.set_ins_status(pvax->ins_status);
  ins_stat_.set_pos_type(pvax->pos_type);
  return true;
}

bool NewtonM2Parser::HandleRawImuX(const novatel::RawImuX* imu) {
  if (imu->imu_error != 0) {
    AWARN << "IMU error. Status: " << std::hex << std::showbase
          << imu->imuStatus;
  }
  if (newtonm2::is_zero(gyro_scale_)) {
    config::ImuType imu_type = imu_type_;
    novatel::ImuParameter param = novatel::GetImuParameter(imu_type);
    AINFO << "IMU type: " << config::ImuType_Name(imu_type) << "; "
          << "Gyro scale: " << param.gyro_scale << "; "
          << "Accel scale: " << param.accel_scale << "; "
          << "Sampling rate: " << param.sampling_rate_hz << ".";

    if (newtonm2::is_zero(param.sampling_rate_hz)) {
      AERROR_EVERY(5) << "Unsupported IMU type: "
                      << config::ImuType_Name(imu_type);
      return false;
    }
    gyro_scale_ = param.gyro_scale * param.sampling_rate_hz;
    accel_scale_ = param.accel_scale * param.sampling_rate_hz;
    imu_measurement_hz_ = static_cast<float>(param.sampling_rate_hz);
    imu_measurement_span_ = static_cast<float>(1.0 / param.sampling_rate_hz);
    imu_.set_measurement_span(imu_measurement_span_);
  }

  double time = imu->gps_week * newtonm2::SECONDS_PER_WEEK + imu->gps_seconds;
  if (imu_measurement_time_previous_ > 0.0 &&
      fabs(time - imu_measurement_time_previous_ - imu_measurement_span_) >
          1e-4) {
    AWARN_EVERY(5) << "Unexpected delay between two IMU measurements at: "
                   << time - imu_measurement_time_previous_;
  }
  imu_.set_measurement_time(time);
  switch (imu_frame_mapping_) {
    case 5:  // Default mapping.
      newtonm2::rfu_to_flu(imu->x_velocity_change * accel_scale_,
                           -imu->y_velocity_change_neg * accel_scale_,
                           imu->z_velocity_change * accel_scale_,
                           imu_.mutable_linear_acceleration());
      newtonm2::rfu_to_flu(imu->x_angle_change * gyro_scale_,
                           -imu->y_angle_change_neg * gyro_scale_,
                           imu->z_angle_change * gyro_scale_,
                           imu_.mutable_angular_velocity());
      break;
    case 6:
      newtonm2::rfu_to_flu(-imu->y_velocity_change_neg * accel_scale_,
                           imu->x_velocity_change * accel_scale_,
                           -imu->z_velocity_change * accel_scale_,
                           imu_.mutable_linear_acceleration());
      newtonm2::rfu_to_flu(-imu->y_angle_change_neg * gyro_scale_,
                           imu->x_angle_change * gyro_scale_,
                           -imu->z_angle_change * gyro_scale_,
                           imu_.mutable_angular_velocity());
      break;
    default:
      AERROR_EVERY(5) << "Unsupported IMU frame mapping: "
                      << imu_frame_mapping_;
  }
  imu_measurement_time_previous_ = time;
  return true;
}

bool NewtonM2Parser::HandleRawImu(const novatel::RawImu* imu) {
  double gyro_scale = 0.0;
  double accel_scale = 0.0;
  float imu_measurement_span = 1.0f / 200.0f;

  if (newtonm2::is_zero(gyro_scale_)) {
    novatel::ImuParameter param = novatel::GetImuParameter(imu_type_);

    if (newtonm2::is_zero(param.sampling_rate_hz)) {
      AERROR_EVERY(5) << "Unsupported IMU type.";
      return false;
    }
    gyro_scale = param.gyro_scale * param.sampling_rate_hz;
    accel_scale = param.accel_scale * param.sampling_rate_hz;
    imu_measurement_span = static_cast<float>(1.0 / param.sampling_rate_hz);
    imu_.set_measurement_span(imu_measurement_span);
  } else {
    gyro_scale = gyro_scale_;
    accel_scale = accel_scale_;
    imu_measurement_span = imu_measurement_span_;
    imu_.set_measurement_span(imu_measurement_span);
  }

  double time = imu->gps_week * newtonm2::SECONDS_PER_WEEK + imu->gps_seconds;
  if (imu_measurement_time_previous_ > 0.0 &&
      fabs(time - imu_measurement_time_previous_ - imu_measurement_span) >
          1e-4) {
    AWARN << "Unexpected delay between two IMU measurements at: "
          << time - imu_measurement_time_previous_;
  }

  imu_.set_measurement_time(time);
  switch (imu_frame_mapping_) {
    case 5:  // Default mapping.
      newtonm2::rfu_to_flu(imu->x_velocity_change * accel_scale,
                           -imu->y_velocity_change_neg * accel_scale,
                           imu->z_velocity_change * accel_scale,
                           imu_.mutable_linear_acceleration());
      newtonm2::rfu_to_flu(imu->x_angle_change * gyro_scale,
                           -imu->y_angle_change_neg * gyro_scale,
                           imu->z_angle_change * gyro_scale,
                           imu_.mutable_angular_velocity());
      break;
    case 6:
      newtonm2::rfu_to_flu(-imu->y_velocity_change_neg * accel_scale,
                           imu->x_velocity_change * accel_scale,
                           -imu->z_velocity_change * accel_scale,
                           imu_.mutable_linear_acceleration());
      newtonm2::rfu_to_flu(-imu->y_angle_change_neg * gyro_scale,
                           imu->x_angle_change * gyro_scale,
                           -imu->z_angle_change * gyro_scale,
                           imu_.mutable_angular_velocity());
      break;
    default:
      AERROR_EVERY(5) << "Unsupported IMU frame mapping: "
                      << imu_frame_mapping_;
  }
  imu_measurement_time_previous_ = time;
  return true;
}

bool NewtonM2Parser::HandleGpsEph(const novatel::GPS_Ephemeris* gps_emph) {
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      gnss_ephemeris_.mutable_keppler_orbit();

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
  keppler_orbit->set_accuracy(
      static_cast<google::protobuf::uint32>(sqrt(gps_emph->ura)));
  keppler_orbit->set_health(gps_emph->health);
  keppler_orbit->set_tgd(gps_emph->tgd);
  keppler_orbit->set_iodc(gps_emph->iodc);
  return true;
}

bool NewtonM2Parser::HandleBdsEph(const novatel::BDS_Ephemeris* bds_emph) {
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::BDS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      gnss_ephemeris_.mutable_keppler_orbit();

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
  keppler_orbit->set_accuracy(
      static_cast<google::protobuf::uint32>(bds_emph->ura));
  keppler_orbit->set_health(bds_emph->health1);
  keppler_orbit->set_tgd(bds_emph->tdg1);
  keppler_orbit->set_iodc(bds_emph->aodc);
  return true;
}

bool NewtonM2Parser::HandleGloEph(const novatel::GLO_Ephemeris* glo_emph) {
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);

  apollo::drivers::gnss::GlonassOrbit* glonass_orbit =
      gnss_ephemeris_.mutable_glonass_orbit();
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

bool NewtonM2Parser::HandleHeading(const novatel::Heading* heading,
                                   uint16_t gps_week, uint32_t gps_millisecs) {
  heading_.set_solution_status(static_cast<uint32_t>(heading->solution_status));
  heading_.set_position_type(static_cast<uint32_t>(heading->position_type));
  heading_.set_baseline_length(heading->length);
  heading_.set_heading(heading->heading);
  heading_.set_pitch(heading->pitch);
  heading_.set_reserved(heading->reserved);
  heading_.set_heading_std_dev(heading->heading_std_dev);
  heading_.set_pitch_std_dev(heading->pitch_std_dev);
  heading_.set_station_id(heading->station_id);
  heading_.set_satellite_tracked_number(heading->num_sats_tracked);
  heading_.set_satellite_soulution_number(heading->num_sats_in_solution);
  heading_.set_satellite_number_obs(heading->num_sats_ele);
  heading_.set_satellite_number_multi(heading->num_sats_l2);
  heading_.set_solution_source(heading->solution_source);
  heading_.set_extended_solution_status(heading->extended_solution_status);
  heading_.set_galileo_beidou_sig_mask(heading->galileo_beidou_sig_mask);
  heading_.set_gps_glonass_sig_mask(heading->gps_glonass_sig_mask);
  double seconds = gps_week * newtonm2::SECONDS_PER_WEEK + gps_millisecs * 1e-3;
  heading_.set_measurement_time(seconds);
  return true;
}

void NewtonM2Parser::SetObservationTime() {
  int week = 0;
  double second = time2gpst(raw_.time, &week);
  gnss_observation_.set_gnss_time_type(apollo::drivers::gnss::GPS_TIME);
  gnss_observation_.set_gnss_week(week);
  gnss_observation_.set_gnss_second_s(second);
}

bool NewtonM2Parser::DecodeGnssObservation(const uint8_t* obs_data,
                                           const uint8_t* obs_data_end) {
  while (obs_data < obs_data_end) {
    const int status = input_oem4(&raw_, *obs_data++);
    switch (status) {
      case 1:  // observation data
        if (raw_.obs.n == 0) {
          AWARN << "Obs is zero";
        }

        gnss_observation_.Clear();
        gnss_observation_.set_receiver_id(0);
        SetObservationTime();
        gnss_observation_.set_sat_obs_num(raw_.obs.n);
        for (int i = 0; i < raw_.obs.n; ++i) {
          int prn = 0;
          int sys = 0;

          sys = satsys(raw_.obs.data[i].sat, &prn);

          apollo::drivers::gnss::GnssType gnss_type;
          if (!gnss_sys_type(sys, &gnss_type)) {
            break;
          }

          auto sat_obs = gnss_observation_.add_sat_obs();  // create obj
          sat_obs->set_sat_prn(prn);
          sat_obs->set_sat_sys(gnss_type);

          int j = 0;
          for (j = 0; j < NFREQ + NEXOBS; ++j) {
            if (newtonm2::is_zero(raw_.obs.data[i].L[j])) {
              break;
            }

            apollo::drivers::gnss::GnssBandID baud_id;
            if (!gnss_baud_id(gnss_type, j, &baud_id)) {
              break;
            }

            auto band_obs = sat_obs->add_band_obs();
            if (raw_.obs.data[i].code[i] == CODE_L1C) {
              band_obs->set_pseudo_type(
                  apollo::drivers::gnss::PseudoType::CORSE_CODE);
            } else if (raw_.obs.data[i].code[i] == CODE_L1P) {
              band_obs->set_pseudo_type(
                  apollo::drivers::gnss::PseudoType::PRECISION_CODE);
            } else {
              AINFO << "Code " << raw_.obs.data[i].code[i] << ", in seq " << j
                    << ", gnss type " << static_cast<int>(gnss_type);
            }

            band_obs->set_band_id(baud_id);
            band_obs->set_pseudo_range(raw_.obs.data[i].P[j]);
            band_obs->set_carrier_phase(raw_.obs.data[i].L[j]);
            band_obs->set_loss_lock_index(raw_.obs.data[i].SNR[j]);
            band_obs->set_doppler(raw_.obs.data[i].D[j]);
            band_obs->set_snr(raw_.obs.data[i].SNR[j]);
            band_obs->set_snr(raw_.obs.data[i].SNR[j]);
          }
          sat_obs->set_band_obs_num(j);
        }
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
