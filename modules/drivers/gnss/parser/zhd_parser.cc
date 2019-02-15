/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <memory>
#include <iomanip>

#include <limits>
#include <iostream>
#include <cmath>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/common/log.h"
#include "modules/drivers/gnss/parser/zhd_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"

#include "modules/drivers/gnss/proto/zhd_gps.pb.h"


namespace apollo {
namespace drivers {
namespace gnss {
// Anonymous namespace that contains helper constants and functions.
namespace {
    constexpr size_t BUFFER_SIZE = 256;
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();
    constexpr double azimuth_deg_to_yaw_rad(double azimuth, double angle) {
      return (angle - azimuth) * DEG_TO_RAD;
    }

}  // namespace

class ZhdParser : public Parser {
 public:
    ZhdParser();
    explicit ZhdParser(const config::Config& config);
    virtual MessageType GetMessage(MessagePtr* message_ptr);

 private:
    Parser::MessageType PrepareMessage(MessagePtr* message_ptr);
    // -1 is an unused value.
    bool check_checksum(void);

    std::vector<uint8_t> buffer_;
    size_t total_length_ = 0;

    ::apollo::drivers::gnss::Ins  ins_;
    ::apollo::drivers::gnss::ZhdGps zhdgps_;
    // add ycy for zhd gps
    config::ZhdConfig zhd_config_;
};

Parser* Parser::createZhd(const config::Config& config) {
  return new ZhdParser(config);
}

ZhdParser::ZhdParser() {
  buffer_.reserve(BUFFER_SIZE);
  buffer_.reserve(BUFFER_SIZE);
  buffer_.clear();
  total_length_ = 0;

  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);
}

ZhdParser::ZhdParser(const config::Config& config) {
  buffer_.reserve(BUFFER_SIZE);
  buffer_.clear();
  total_length_ = 0;

  ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
  ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

  if (config.has_zhd_config()) {
    zhd_config_ = config.zhd_config();
  } else {
    zhd_config_.set_angle_heading(90);
    AERROR << "angle_heading=" << hex << zhd_config_.angle_heading();
  }
}

Parser::MessageType ZhdParser::GetMessage(MessagePtr* message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  while (data_ < data_end_) {
    if (buffer_.size() == 0) {
      // Looking for SYNC0
      if (*data_ == zhd::SYNC_0) {
        buffer_.push_back(*data_);
      }
      ++data_;
      total_length_ = 0;
    } else if (buffer_.size() == 1) {
      // Looking for SYNC1
      if (*data_ == zhd::SYNC_1) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
        total_length_ = 0;
      }
    } else if (buffer_.size() == 2) {
      // Looking for VERSION_2
      if (*data_ == zhd::VERSION_2) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
        total_length_ = 0;
      }
    } else if (buffer_.size() == 3) {
      // Looking for VERSION_3
      if (*data_ == zhd::VERSION_3) {
        buffer_.push_back(*data_++);
        total_length_ = 6;
      } else {
        buffer_.clear();
        total_length_ = 0;
      }
    } else if (total_length_ > 0) {
        if (buffer_.size() >256) {
          AERROR << "buffer_.size() >256 ";
          buffer_.clear();
          total_length_ = 0;
        }
        if (buffer_.size() < total_length_) {
            buffer_.push_back(*data_++);
            if (buffer_.size() == 6) {
              total_length_ =
                (*reinterpret_cast<uint16_t*>(buffer_.data() + 4)) +2;
              if (total_length_ > 138) {
                AERROR << "buffer_.size()1 >138 ";
                total_length_ = 0;
                buffer_.clear();
                continue;
              }
            }
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

Parser::MessageType ZhdParser::PrepareMessage(MessagePtr* message_ptr) {
    if (false == check_checksum()) {
        AERROR << "check_checksum check failed.";
        return MessageType::NONE;
    }

    auto stZhdData =
        reinterpret_cast<const zhd::gps_rtk_zhd_packet_t*>(buffer_.data());

    struct tm timeinfo = {};

    timeinfo.tm_mday = stZhdData->day_utc;
    timeinfo.tm_mon = stZhdData->month_utc - 1;
    timeinfo.tm_year = stZhdData->year_utc - 1900;
    timeinfo.tm_hour = stZhdData->hour_utc;
    timeinfo.tm_min = stZhdData->min_utc;
    timeinfo.tm_sec = stZhdData->sec_utc / 100;
    timeinfo.tm_isdst = 0;

    double milliseconds_a = (stZhdData->sec_utc  % 100) * 10;
    double seconds = milliseconds_a / 1000;
    time_t epoch = mktime(&timeinfo);
    double time_stamp = static_cast<double>epoch + seconds;

    zhdgps_.set_measurement_time(time_stamp);
    zhdgps_.set_longitude(stZhdData->longitude);
    zhdgps_.set_latitude(stZhdData->latitude);
    zhdgps_.set_altitude(stZhdData->altitude);

    zhdgps_.set_eph(stZhdData->eph);
    zhdgps_.set_epv(stZhdData->epv);
    zhdgps_.set_vel_ground_m_s(stZhdData->vel_ground_m_s);
    zhdgps_.set_angle_tracktrue(stZhdData->angle_tracktrue);

    zhdgps_.set_angle_heading(stZhdData->angle_heading);
    zhdgps_.set_angle_pitch(stZhdData->angle_pitch);
    zhdgps_.set_vel_e_m_s(stZhdData->vel_e_m_s);
    zhdgps_.set_vel_n_m_s(stZhdData->vel_n_m_s);

    zhdgps_.set_vel_u_m_s(azimuth_deg_to_yaw_rad(
    stZhdData->angle_heading, zhd_config_.angle_heading()));

    zhdgps_.set_satellites_used(stZhdData->satellites_used);
    zhdgps_.set_satellites_track(stZhdData->satellites_track);

    zhdgps_.set_vel_ned_valid(stZhdData->vel_ned_valid);
    switch (stZhdData->fix_type) {
        case apollo::drivers::gnss::ZhdGps::FIX_TYPE_INVALID:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::FIX_TYPE_INVALID);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::FIX_TYPE_SPOINT:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::FIX_TYPE_SPOINT);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::FIX_TYPE_VBS_PPP:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::FIX_TYPE_VBS_PPP);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::FIX_TYPE_RT2:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::GOOD);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::FIX_TYPE_RT2);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::FIX_TYPE_PPP:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::FIX_TYPE_PPP);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::FIX_TYPE_DEAD_MOD:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(
            apollo::drivers::gnss::ZhdGps::FIX_TYPE_DEAD_MOD);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::FIX_TYPE_INPUT_MOD:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(
            apollo::drivers::gnss::ZhdGps::FIX_TYPE_INPUT_MOD);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::SIMULATOR_MODE:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::SIMULATOR_MODE);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::WAAS:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::WAAS);
          break;
        }
        case apollo::drivers::gnss::ZhdGps::INS_FIXED:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::INS_FIXED);
          break;
        }
        default:
        {
          zhdgps_.set_instype(apollo::drivers::gnss::ZhdGps::INVALID);
          zhdgps_.set_fix_type(apollo::drivers::gnss::ZhdGps::FIX_TYPE_INVALID);
          AERROR << "zhd gps invalid" << stZhdData->fix_type;
          break;
        }
      }

    switch ((uint32_t)stZhdData->angle_postype) {
        case apollo::drivers::gnss::ZhdGps::POS_TYPE_NARROW_INT:
        {
          zhdgps_.set_angle_postype(
            apollo::drivers::gnss::ZhdGps::POS_TYPE_NARROW_INT);
          break;
        }
        default:
        {
          zhdgps_.set_angle_postype(
            apollo::drivers::gnss::ZhdGps::POS_TYPE_NONE);
          AERROR << "zhd set_angle_postype invalid="
            << stZhdData->angle_postype;
          break;
        }
      }
      zhdgps_.set_head_deviation(stZhdData->head_deviation);
      zhdgps_.set_ins_state(stZhdData->ins_state);
      zhdgps_.set_gnss_alt_delta(stZhdData->gnss_alt_delta);
      zhdgps_.set_ellipsoidal_h(stZhdData->ellipsoidal_h);
      zhdgps_.set_diff_age(stZhdData->diff_age);

      *message_ptr = &zhdgps_;
      return MessageType::ZHD_GPS;
}

bool ZhdParser::check_checksum(void) {
  uint16_t xor_check = 0;
  uint16_t Checksum = 0;

  Checksum =
    (uint16_t)*reinterpret_cast<uint16_t*>(buffer_.data() +
    buffer_.size()-2);

  for (vector<uint8_t>::iterator riter = buffer_.begin();
    riter != (buffer_.end()-2); riter++) {
    xor_check = xor_check^(reinterpret_cast<uint8_t>(*riter));
  }
  if (xor_check != Checksum) {
    AERROR << " 0 = xor_check="
      << xor_check << "stZhd_checksum=" << Checksum;
    return false;
  }
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
