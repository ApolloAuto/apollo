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
// An parser for decoding binary messages from a ENS receiver. The following
// messages must be
// logged in order for this parser to work properly.

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "modules/common_msgs/sensor_msgs/gnss.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/enbroad_parser/enbroad_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/parser_common.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Encoding and decoding protocol scaling factor setting
#define Coder_Accel_Scale 120.0
#define Coder_Rate_Scale 300.0
#define Coder_MAG_Scale 1.0
#define Coder_IFof_Rate_Scale 600.0
#define Coder_Angle_Scale 360.0
#define Coder_Temp_Scale 200.0
#define Coder_Sensor_Scale 32768.0
#define Coder_IFof_Sensor_Scale 2147483648.0
#define Coder_EXP_E 2.718282.0
#define Coder_Vel_Scale 100.0
#define Coder_Pos_Scale 10000000000.0

constexpr size_t BUFFER_SIZE = 256;
uint16_t g_ENS_MSG_ID = 0;

union {
  char bd[8];
  unsigned short iv;
  short sv;
  int lv;
  unsigned int uv;
  float fv;
  double dv;
} m_uMemory;

float get_F32(char* msgbuff, int i) {
  m_uMemory.bd[0] = msgbuff[i];
  m_uMemory.bd[1] = msgbuff[i + 1];
  m_uMemory.bd[2] = msgbuff[i + 2];
  m_uMemory.bd[3] = msgbuff[i + 3];
  return m_uMemory.fv;
}

class EnbroadParse : public Parser {
 public:
  EnbroadParse();
  explicit EnbroadParse(const config::Config& config);

  virtual void GetMessages(MessageInfoVec* messages);

 private:
  bool PrepareMessage();
  bool check_sum();
  bool HandleNavData(const enbroad::NAV_DATA_TypeDef* pNavData);
  bool HandleSINSData(const enbroad::NAV_SINS_TypeDef* pSinsData);
  bool HandleExtendData(const enbroad::NAV_Extend_TypeDef* pExtendData);
  bool HandleIMUData(const enbroad::NAV_IMU_TypeDef* pImuData);
  bool HandleGNSSData(const enbroad::NAV_GNSS_TypeDef* pGnssData);

  size_t header_length_ = 0;
  size_t total_length_ = 0;
  std::vector<uint8_t> buffer_;

  GnssBestPose bestpos_;
  Imu imu_;
  Heading heading_;
  Ins ins_;
  InsStat ins_stat_;
};

Parser* Parser::CreateEnbroad(const config::Config& config) {
  return new EnbroadParse(config);
}

EnbroadParse::EnbroadParse() { buffer_.reserve(BUFFER_SIZE); }

EnbroadParse::EnbroadParse(const config::Config& config) {
  buffer_.reserve(BUFFER_SIZE);
}

void EnbroadParse::GetMessages(MessageInfoVec* messages) {
  if (data_ == nullptr) {
    return;
  }
  while (data_ < data_end_) {
    if (buffer_.empty()) {  // Looking for SYNC_HEAD_0
      if (*data_ == enbroad::SYNC_HEAD_0) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for SYNC_HEAD_1
      if (*data_ == enbroad::SYNC_HEAD_1) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 2) {  // Looking for SYNC_HEAD_2
      if (*data_ == enbroad::SYNC_HEAD_2) {
        buffer_.push_back(*data_++);
        header_length_ = sizeof(enbroad::FrameHeader);
      } else {
        buffer_.clear();
      }
    } else if (header_length_ > 0) {  // Working on header.
      if (buffer_.size() < header_length_) {
        buffer_.push_back(*data_++);
      } else {
        total_length_ = header_length_ +
                        reinterpret_cast<enbroad::FrameHeader*>(buffer_.data())
                            ->message_length +
                        1 + 2;
        header_length_ = 0;
      }
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) {  // Working on body.
        buffer_.push_back(*data_++);
        continue;
      }
      if (!PrepareMessage()) {
        buffer_.clear();
        total_length_ = 0;
      } else {
        buffer_.clear();
        total_length_ = 0;
        if (enbroad::BIN_NAV_DATA == g_ENS_MSG_ID) {
          messages->push_back(
              MessageInfo{MessageType::BEST_GNSS_POS,
                          reinterpret_cast<MessagePtr>(&bestpos_)});
          messages->push_back(MessageInfo{MessageType::IMU,
                                          reinterpret_cast<MessagePtr>(&imu_)});
          messages->push_back(MessageInfo{
              MessageType::HEADING, reinterpret_cast<MessagePtr>(&heading_)});
          messages->push_back(MessageInfo{MessageType::INS,
                                          reinterpret_cast<MessagePtr>(&ins_)});
          messages->push_back(MessageInfo{
              MessageType::INS_STAT, reinterpret_cast<MessagePtr>(&ins_stat_)});
        } else if (enbroad::BIN_Extend_DATA ==
                   g_ENS_MSG_ID) {  // ins=sins+extend
          messages->push_back(MessageInfo{MessageType::INS,
                                          reinterpret_cast<MessagePtr>(&ins_)});
          messages->push_back(MessageInfo{
              MessageType::INS_STAT, reinterpret_cast<MessagePtr>(&ins_stat_)});
        } else if (enbroad::BIN_IMU_DATA == g_ENS_MSG_ID) {
          messages->push_back(MessageInfo{MessageType::IMU,
                                          reinterpret_cast<MessagePtr>(&imu_)});
        } else if (enbroad::BIN_GNSS_DATA == g_ENS_MSG_ID) {
          messages->push_back(
              MessageInfo{MessageType::BEST_GNSS_POS,
                          reinterpret_cast<MessagePtr>(&bestpos_)});
          messages->push_back(MessageInfo{
              MessageType::HEADING, reinterpret_cast<MessagePtr>(&heading_)});
        }
      }
    }
  }
}
double normalizeAngleTo180(double angle) {
  while (angle > 180.0) {
    angle -= 360.0;
  }
  while (angle <= -180.0) {
    angle += 360.0;
  }
  return angle;
}
bool EnbroadParse::check_sum() {
  char checksum = 0;
  char compare = 0;
  size_t len = buffer_.size() - 3;
  size_t i = 0;
  while (len--) {
    checksum += *reinterpret_cast<int8_t*>(buffer_.data() + i);
    i++;
  }
  checksum = ~checksum;
  checksum = checksum | 0x30;
  compare = *reinterpret_cast<char*>(buffer_.data() + buffer_.size() - 3);
  if (checksum == compare) {
    return true;
  } else {
    return false;
  }
}

bool EnbroadParse::PrepareMessage() {
  static uint64_t msg_sum = 0;
  static uint64_t msg_bad = 0;
  g_ENS_MSG_ID = 0;
  msg_sum++;
  if (!check_sum()) {
    msg_bad++;
    AERROR << "check sum failed. bad frame ratio"
           << static_cast<double>(msg_bad / msg_sum);
    return false;
  }
  uint8_t* message = nullptr;
  enbroad::MessageId message_id;
  auto header = reinterpret_cast<const enbroad::FrameHeader*>(buffer_.data());
  message = buffer_.data() + sizeof(enbroad::FrameHeader);
  message_id = header->message_id;
  switch (message_id) {
    case enbroad::BIN_NAV_DATA:
      if (!HandleNavData(
              reinterpret_cast<enbroad::NAV_DATA_TypeDef*>(message))) {
        AWARN << "HandleNavData fail";
        return false;
      }
      break;
    case enbroad::BIN_SINS_DATA:
      if (!HandleSINSData(
              reinterpret_cast<enbroad::NAV_SINS_TypeDef*>(message))) {
        AWARN << "HandleSINSData fail";
        return false;
      }
      break;
    case enbroad::BIN_IMU_DATA:
      if (!HandleIMUData(
              reinterpret_cast<enbroad::NAV_IMU_TypeDef*>(message))) {
        AWARN << "HandleIMUData fail";
        return false;
      }
      break;
    case enbroad::BIN_GNSS_DATA:
      if (!HandleGNSSData(
              reinterpret_cast<enbroad::NAV_GNSS_TypeDef*>(message))) {
        return false;
      }
      break;
    case enbroad::BIN_Extend_DATA:
      if (!HandleExtendData(
              reinterpret_cast<enbroad::NAV_Extend_TypeDef*>(message))) {
        return false;
      }
      break;
    default:
      return false;
  }

  g_ENS_MSG_ID = message_id;
  return true;
}

bool EnbroadParse::HandleSINSData(const enbroad::NAV_SINS_TypeDef* pSinsData) {
  double seconds =
      pSinsData->gps_week * SECONDS_PER_WEEK + pSinsData->gpssecond * 1e-3;
  ins_.set_measurement_time(seconds);
  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  ins_.mutable_euler_angles()->set_x(pSinsData->roll * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_y(pSinsData->pitch * DEG_TO_RAD);
  // enbroad set northwest as right direction, Here northeast as right direction
  ins_.mutable_euler_angles()->set_z(
      azimuth_deg_to_yaw_rad(normalizeAngleTo180(-pSinsData->heading)));
  ins_.mutable_position()->set_lon(pSinsData->longitude);
  ins_.mutable_position()->set_lat(pSinsData->latitude);
  ins_.mutable_position()->set_height(pSinsData->altitude);
  ins_.mutable_linear_velocity()->set_x(pSinsData->ve);
  ins_.mutable_linear_velocity()->set_y(pSinsData->vn);
  ins_.mutable_linear_velocity()->set_z(pSinsData->vu);
  if (2 == static_cast<int>(pSinsData->navStatus)) {
    ins_.set_type(Ins::GOOD);
  } else if (1 == static_cast<int>(pSinsData->navStatus)) {
    ins_.set_type(Ins::CONVERGING);
  } else {
    ins_.set_type(Ins::INVALID);
  }

  ins_stat_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  // ins pos type define as follows
  // fusion GPS as INS_RTKFIXED,
  // fusion wheel as INS_RTKFLOAT,fusion motion as SINGLE,others as NONE
  if (enbroad::E_FUNSION_GPS == static_cast<int>(pSinsData->fusion)) {
    ins_stat_.set_pos_type(SolutionType::INS_RTKFIXED);
  } else {
    ins_stat_.set_pos_type(SolutionType::NONE);
  }

  if (2 == static_cast<int>(pSinsData->navStatus)) {
    ins_stat_.set_ins_status(SolutionStatus::SOL_COMPUTED);
  } else if (1 == static_cast<int>(pSinsData->navStatus)) {
    ins_stat_.set_ins_status(SolutionStatus::COLD_START);
  } else {
    ins_stat_.set_ins_status(SolutionStatus::INSUFFICIENT_OBS);
  }

  return true;
}
bool EnbroadParse::HandleIMUData(const enbroad::NAV_IMU_TypeDef* pImuData) {
  float imu_measurement_span = 1.0f / 100.0f;
  double seconds =
      pImuData->gps_week * SECONDS_PER_WEEK + pImuData->gpssecond * 1e-3;

  imu_.set_measurement_time(seconds);
  imu_.set_measurement_span(imu_measurement_span);
  imu_.mutable_linear_acceleration()->set_x(pImuData->accX);
  imu_.mutable_linear_acceleration()->set_y(pImuData->accY);
  imu_.mutable_linear_acceleration()->set_z(pImuData->accZ);
  imu_.mutable_angular_velocity()->set_x(pImuData->gyroX * DEG_TO_RAD);
  imu_.mutable_angular_velocity()->set_y(pImuData->gyroY * DEG_TO_RAD);
  imu_.mutable_angular_velocity()->set_z(pImuData->gyroZ * DEG_TO_RAD);
  return true;
}
bool EnbroadParse::HandleGNSSData(const enbroad::NAV_GNSS_TypeDef* pGnssData) {
  double seconds =
      pGnssData->gps_week * SECONDS_PER_WEEK + pGnssData->gpssecond * 1e-3;

  bestpos_.set_measurement_time(seconds);
  heading_.set_measurement_time(seconds);
  // bestpos_.set_base_station_id("0");  // base station id
  bestpos_.set_num_sats_tracked(pGnssData->satsNum);
  bestpos_.set_num_sats_in_solution(pGnssData->satsNum);
  bestpos_.set_num_sats_in_solution(pGnssData->satsNum);
  bestpos_.set_num_sats_multi(pGnssData->satsNum);
  heading_.set_satellite_tracked_number(pGnssData->satsNum);
  heading_.set_satellite_soulution_number(pGnssData->satsNum);
  heading_.set_satellite_number_obs(pGnssData->satsNum);
  heading_.set_satellite_number_multi(pGnssData->satsNum);
  // bestpos_.set_galileo_beidou_used_mask(0);
  // bestpos_.set_gps_glonass_used_mask(0);
  bestpos_.set_solution_age(pGnssData->age);  // solution age (sec)
  if (enbroad::E_GPS_RTK_FIXED == static_cast<int>(pGnssData->rtkStatus)) {
    bestpos_.set_sol_type(SolutionType::INS_RTKFIXED);
  } else if (enbroad::E_GPS_RTK_FLOAT ==
             static_cast<int>(pGnssData->rtkStatus)) {
    bestpos_.set_sol_type(SolutionType::INS_RTKFLOAT);
  } else if (enbroad::E_GPS_RTK_SPP == static_cast<int>(pGnssData->rtkStatus) ||
             enbroad::E_GPS_RTK_DGPS ==
                 static_cast<int>(pGnssData->rtkStatus)) {
    bestpos_.set_sol_type(SolutionType::SINGLE);
  } else {
    bestpos_.set_sol_type(SolutionType::NONE);
  }

  bestpos_.set_latitude(pGnssData->latitude);
  bestpos_.set_longitude(pGnssData->longitude);
  bestpos_.set_height_msl(pGnssData->altitude);

  if (enbroad::E_GPS_RTK_FIXED == static_cast<int>(pGnssData->headingStatus)) {
    heading_.set_position_type(SolutionType::INS_RTKFIXED);
  } else if (enbroad::E_GPS_RTK_FLOAT ==
             static_cast<int>(pGnssData->headingStatus)) {
    heading_.set_position_type(SolutionType::INS_RTKFLOAT);
  } else if (enbroad::E_GPS_RTK_SPP ==
                 static_cast<int>(pGnssData->headingStatus) ||
             enbroad::E_GPS_RTK_DGPS ==
                 static_cast<int>(pGnssData->headingStatus)) {
    heading_.set_position_type(SolutionType::SINGLE);
  } else {
    heading_.set_position_type(SolutionType::NONE);
  }
  heading_.set_baseline_length(pGnssData->baseline);
  heading_.set_heading(pGnssData->heading);
  heading_.set_reserved(0);
  heading_.set_heading_std_dev(pGnssData->xigema_heading);
  // heading_.set_heading_std_dev(0.0);
  // heading_.set_pitch_std_dev(0.0);
  // heading_.set_station_id("0");

  // heading_.set_solution_source(0);
  // heading_.set_extended_solution_status(0);
  // heading_.set_galileo_beidou_sig_mask(0);
  // heading_.set_gps_glonass_sig_mask(0);

  // AERROR << "pGnssData->xigema_lat=" << pGnssData->xigema_lat;
  // AERROR << "pGnssData->xigema_lon=" << pGnssData->xigema_lon;
  // AERROR << "ppGnssData->xigema_alt=" << pGnssData->xigema_alt;
  // AERROR << "ppGnssData->xigema_heading=" << pGnssData->xigema_heading;

  return true;
}

// *Strongly recommend:
// Do not place NAV_Extend_TypeDef in imu_, heading_, bestpos_
// to avoid data anomalies
bool EnbroadParse::HandleExtendData(
    const enbroad::NAV_Extend_TypeDef* pExtendData) {
  ins_.mutable_linear_acceleration()->set_x(
      static_cast<double>(pExtendData->corrAccX));
  ins_.mutable_linear_acceleration()->set_y(
      static_cast<double>(pExtendData->corrAccY));
  ins_.mutable_linear_acceleration()->set_z(
      static_cast<double>(pExtendData->corrAccZ));
  ins_.mutable_angular_velocity()->set_x(
      static_cast<double>(pExtendData->corrGyroX * DEG_TO_RAD));
  ins_.mutable_angular_velocity()->set_y(
      static_cast<double>(pExtendData->corrGyroY * DEG_TO_RAD));
  ins_.mutable_angular_velocity()->set_z(
      static_cast<double>(pExtendData->corrGyroZ * DEG_TO_RAD));
#if 0
  AERROR << "pExtendData->corrAccX=" << pExtendData->corrAccX;
  AERROR << "pExtendData->corrAccY=" << pExtendData->corrAccY;
  AERROR << "pExtendData->corrAccZ=" << pExtendData->corrAccZ;

  AERROR << "pExtendData->corrGyroX=" << pExtendData->corrGyroX;
  AERROR << "pExtendData->corrGyroY=" << pExtendData->corrGyroY;
  AERROR << "pExtendData->corrGyroZ=" << pExtendData->corrGyroZ;

  AERROR << "pExtendData->gnssAttFromIMU=" << pExtendData->gnssAttFromIMU;
  AERROR << "pExtendData->odsAttFromIMU=" << pExtendData->odsAttFromIMU;

  AERROR << "pExtendData->Undulation=" << pExtendData->Undulation;
#endif
  return true;
}

bool EnbroadParse::HandleNavData(const enbroad::NAV_DATA_TypeDef* pNavData) {
  static uint16_t rtkStatus;
  static uint16_t Nav_Standard_flag;
  static uint16_t Sate_Num;
  static float baseline;
  // char buff[8]={0};
  float imu_measurement_span = 1.0f / 100.0f;
  double seconds =
      pNavData->gps_week * SECONDS_PER_WEEK + pNavData->gps_millisecs * 1e-3;
  ins_.set_measurement_time(seconds);
  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  ins_.mutable_euler_angles()->set_x(static_cast<double>(
      pNavData->roll * Coder_Angle_Scale / Coder_Sensor_Scale * DEG_TO_RAD));
  ins_.mutable_euler_angles()->set_y(static_cast<double>(
      pNavData->pitch * Coder_Angle_Scale / Coder_Sensor_Scale * DEG_TO_RAD));
  // enbroad set northwest as right direction, Here northeast as right direction
  ins_.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(normalizeAngleTo180(
      -pNavData->head * Coder_Angle_Scale / Coder_Sensor_Scale)));
  ins_.mutable_position()->set_lon(
      static_cast<double>(pNavData->lon / Coder_Pos_Scale));
  ins_.mutable_position()->set_lat(
      static_cast<double>(pNavData->lat / Coder_Pos_Scale));
  ins_.mutable_position()->set_height(
      static_cast<double>(pNavData->alt / 1000.0));
  ins_.mutable_linear_acceleration()->set_x(static_cast<double>(
      pNavData->accX * Coder_Accel_Scale / Coder_Sensor_Scale));
  ins_.mutable_linear_acceleration()->set_y(static_cast<double>(
      pNavData->accY * Coder_Accel_Scale / Coder_Sensor_Scale));
  ins_.mutable_linear_acceleration()->set_z(static_cast<double>(
      pNavData->accZ * Coder_Accel_Scale / Coder_Sensor_Scale));
  ins_.mutable_angular_velocity()->set_x(
      static_cast<double>(pNavData->gyroX * Coder_Rate_Scale /
                          Coder_Sensor_Scale) *
      DEG_TO_RAD);
  ins_.mutable_angular_velocity()->set_y(
      static_cast<double>(pNavData->gyroY * Coder_Rate_Scale /
                          Coder_Sensor_Scale) *
      DEG_TO_RAD);
  ins_.mutable_angular_velocity()->set_z(
      static_cast<double>(pNavData->gyroZ * Coder_Rate_Scale /
                          Coder_Sensor_Scale) *
      DEG_TO_RAD);
  ins_.mutable_linear_velocity()->set_x(
      static_cast<double>(pNavData->ve * Coder_Vel_Scale / Coder_Sensor_Scale));
  ins_.mutable_linear_velocity()->set_y(
      static_cast<double>(pNavData->vn * Coder_Vel_Scale / Coder_Sensor_Scale));
  ins_.mutable_linear_velocity()->set_z(
      static_cast<double>(pNavData->vu * Coder_Vel_Scale / Coder_Sensor_Scale));
  ins_.set_type(Ins::GOOD);
  switch (pNavData->poll_type) {
    case enbroad::E_POLL_DEV_TEMP:
      break;
    case enbroad::E_POLL_GNSS_STATE:
      rtkStatus = pNavData->poll_frame1;
      break;
    case enbroad::E_POLL_CAN_STATE:
      break;
    case enbroad::E_POLL_INS_STATE:
      Nav_Standard_flag = pNavData->poll_frame1;
    case enbroad::E_POLL_INS_STANDARD_GNSS2:
      // poll_frame1+poll_frame2×éfloat
      // char *pAddr= static_cast<char *>(&pNavData->poll_frame1);
      // buff[0]=pNavData->poll_frame1&0XFF00;
      // buff[1]=pNavData->poll_frame1&0X00FF;
      // buff[2]=pNavData->poll_frame2&0XFF00;
      // buff[3]=pNavData->poll_frame2&0X00FF;
      // AERROR << "E_POLL_INS_STANDARD_GNSS2=" << get_F32(pAddr,0);
      break;
    case enbroad::E_POLL_GNSS2_STATE:
      Sate_Num = pNavData->poll_frame1;
      baseline = pNavData->poll_frame2 / 1000.0;
      break;
    default:
      break;
  }
  imu_.set_measurement_time(seconds);
  imu_.set_measurement_span(imu_measurement_span);
  imu_.mutable_linear_acceleration()->set_x(static_cast<double>(
      pNavData->accX * Coder_Accel_Scale / Coder_Sensor_Scale));
  imu_.mutable_linear_acceleration()->set_y(static_cast<double>(
      pNavData->accY * Coder_Accel_Scale / Coder_Sensor_Scale));
  imu_.mutable_linear_acceleration()->set_z(static_cast<double>(
      pNavData->accZ * Coder_Accel_Scale / Coder_Sensor_Scale));
  imu_.mutable_angular_velocity()->set_x(
      static_cast<double>(pNavData->gyroX * Coder_Rate_Scale /
                          Coder_Sensor_Scale) *
      DEG_TO_RAD);
  imu_.mutable_angular_velocity()->set_y(
      static_cast<double>(pNavData->gyroY * Coder_Rate_Scale /
                          Coder_Sensor_Scale) *
      DEG_TO_RAD);
  imu_.mutable_angular_velocity()->set_z(
      static_cast<double>(pNavData->gyroZ * Coder_Rate_Scale /
                          Coder_Sensor_Scale) *
      DEG_TO_RAD);
  bestpos_.set_measurement_time(seconds);
  bestpos_.set_longitude(static_cast<double>(pNavData->lon / Coder_Pos_Scale));
  bestpos_.set_latitude(static_cast<double>(pNavData->lat / Coder_Pos_Scale));
  bestpos_.set_height_msl(static_cast<double>(pNavData->alt / 1000.0));
  bestpos_.set_undulation(0.0);  // undulation = height_wgs84 - height_msl
  bestpos_.set_datum_id(
      static_cast<apollo::drivers::gnss::DatumId>(enbroad::DatumId::WGS84));
  bestpos_.set_latitude_std_dev(0.0);
  bestpos_.set_longitude_std_dev(0.0);
  bestpos_.set_height_std_dev(0.0);
  bestpos_.set_base_station_id("0");
  bestpos_.set_solution_age(0.0);  // solution age (sec)
  bestpos_.set_num_sats_tracked(Sate_Num);
  bestpos_.set_num_sats_in_solution(Sate_Num);
  bestpos_.set_num_sats_in_solution(Sate_Num);
  bestpos_.set_num_sats_multi(Sate_Num);
  bestpos_.set_extended_solution_status(SolutionType::INS_RTKFIXED);
  bestpos_.set_galileo_beidou_used_mask(0);
  bestpos_.set_gps_glonass_used_mask(0);
  heading_.set_measurement_time(seconds);
  heading_.set_pitch(static_cast<double>(pNavData->pitch * Coder_Angle_Scale /
                                         Coder_Sensor_Scale));
  // enbroad set northwest as right direction,
  // Here northeast as right direction
  heading_.set_heading(normalizeAngleTo180(-pNavData->head * Coder_Angle_Scale /
                                           Coder_Sensor_Scale));
  heading_.set_baseline_length(baseline);
  heading_.set_reserved(0);
  heading_.set_heading_std_dev(0.0);
  heading_.set_pitch_std_dev(0.0);
  heading_.set_station_id("0");
  heading_.set_satellite_tracked_number(Sate_Num);
  heading_.set_satellite_soulution_number(Sate_Num);
  heading_.set_satellite_number_obs(Sate_Num);
  heading_.set_satellite_number_multi(Sate_Num);
  heading_.set_solution_source(0);
  heading_.set_extended_solution_status(0);
  heading_.set_galileo_beidou_sig_mask(0);
  heading_.set_gps_glonass_sig_mask(0);
  ins_stat_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  // According to the RTK state:
  // fixed as "INS-RTKFIXED",float as "INS-RTKFLOAT",
  // single or RTD as "SINGLE",no solution as "NONE"
  if (enbroad::E_GPS_RTK_FIXED == rtkStatus) {
    ins_stat_.set_pos_type(SolutionType::INS_RTKFIXED);
    bestpos_.set_sol_type(SolutionType::INS_RTKFIXED);
    heading_.set_position_type(SolutionType::INS_RTKFIXED);
  } else if (enbroad::E_GPS_RTK_FLOAT == rtkStatus) {
    ins_stat_.set_pos_type(SolutionType::INS_RTKFLOAT);
    bestpos_.set_sol_type(SolutionType::INS_RTKFLOAT);
    heading_.set_position_type(SolutionType::INS_RTKFLOAT);
  } else if (enbroad::E_GPS_RTK_SPP == rtkStatus ||
             enbroad::E_GPS_RTK_DGPS == rtkStatus) {
    ins_stat_.set_pos_type(SolutionType::SINGLE);
    bestpos_.set_sol_type(SolutionType::SINGLE);
    heading_.set_position_type(SolutionType::SINGLE);
  } else {
    ins_stat_.set_pos_type(SolutionType::NONE);
    bestpos_.set_sol_type(SolutionType::NONE);
    heading_.set_position_type(SolutionType::NONE);
  }
  // According to sins calibration status:
  // no calibration as "SOL_COMPUTED",
  // during calibration as "SOL_COMPUTED",
  // completing calibration as "SOL_COMPUTED"
  if (enbroad::E_NAV_STANDARD_PROCCSSED == Nav_Standard_flag) {
    ins_stat_.set_ins_status(SolutionStatus::SOL_COMPUTED);
    bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
    heading_.set_solution_status(SolutionStatus::SOL_COMPUTED);
  } else if (enbroad::E_NAV_STANDARD_PROCCSSING == Nav_Standard_flag) {
    ins_stat_.set_ins_status(SolutionStatus::COLD_START);
    bestpos_.set_sol_status(SolutionStatus::COLD_START);
    heading_.set_solution_status(SolutionStatus::COLD_START);
  } else {
    ins_stat_.set_ins_status(SolutionStatus::INSUFFICIENT_OBS);
    bestpos_.set_sol_status(SolutionStatus::INSUFFICIENT_OBS);
    heading_.set_solution_status(SolutionStatus::INSUFFICIENT_OBS);
  }
  return true;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
