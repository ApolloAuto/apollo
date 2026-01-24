/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include <time.h>

#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/gnss/parser/asensing_parser/asensing_base_parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

#define ACC_UNIT 0.0001220703 - 4
#define G_UNIT 9.7883105
#define GYRO_UNIT 0.0076293 - 250
#define ANGLE_UNIT 0.010986 - 360
#define ALT_UINT 0.001 - 10000
#define LOC_UNIT 1E-07 - 180
#define SPEED_UNIT 0.0030517 - 100
#define LOC_STD_UNIT 0.001
#define HEADING_STD_UNIT 0.01

constexpr uint32_t FRAME_ins_acc = 0x500;
constexpr uint32_t FRAME_ins_gyro = 0x501;
constexpr uint32_t FRAME_ins_angle = 0x502;
constexpr uint32_t FRAME_ins_height_and_time = 0x503;
constexpr uint32_t FRAME_ins_latitude_longitude = 0x504;
constexpr uint32_t FRAME_ins_speed = 0x505;
constexpr uint32_t FRAME_ins_datainfo = 0x506;
constexpr uint32_t FRAME_ins_std = 0x507;
constexpr uint32_t FRAME_gnss_utc = 0x508;
constexpr uint32_t FRAME_gps_latitude_longitude = 0x509;
constexpr uint32_t FRAME_gps_height_and_week = 0x50A;
constexpr uint32_t FRAME_gps_heading_and_pitch = 0x50C;
constexpr uint32_t FRAME_gps_std = 0x512;

struct AsensingData {
  uint32_t gps_week = 0;
  uint32_t week_ms = 0;
  double ins_gps_sec = 0;
  double gps_sec = 0;

  // gps
  double gps_latitude = -1.0;
  double gps_longitude;
  double gps_altitude;
  float gps_heading = -1.0;
  float gps_pitch;
  float gps_lat_std = 0.0;
  float gps_lon_std = 0.0;
  float gps_alti_std = 0.0;
  float gps_yaw_std = 0.0;

  // INS
  float heading;
  float pitch;
  float roll;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
  double latitude;
  double longitude;
  double altitude;
  float ve;
  float vn;
  float vd;
  float lat_std = 0.0;
  float lon_std = 0.0;
  float alti_std = 0.0;
  float vn_std = 0.0;
  float ve_std = 0.0;
  float vd_std = 0.0;
  float roll_std = 0.0;
  float pitch_std = 0.0;
  float yaw_std = 0.0;
  uint8_t satellites_num = 0;
  double age;
  float differential_age = 0.0;
  SolutionStatus solution_status;
  SolutionType solution_type;
};

template <typename T>
T ToValue(const uint8_t *data, const int &index) {
  T t = 0;
  for (uint8_t i = 0; i < sizeof(T); ++i) {
    t |= static_cast<T>(data[index + i]) << ((sizeof(T) - i - 1) * 8);
  }
  return t;
}

template <typename T0, typename T1>
T1 ToValue(const uint8_t *data, const int &index) {
  return static_cast<T1>(ToValue<T0>(data, index));
}

class AsensingCanParser : public AsensingBaseParser {
 public:
  explicit AsensingCanParser(const config::Config &config)
      : AsensingBaseParser(config) {}
  void GetMessages(MessageInfoVec *messages) override;
  bool PrepareMessage() override;
  void FillGnssBestpos() override;
  void FillIns() override;
  void FillInsStat() override;
  void FillImu() override;
  void FillHeading() override;

 private:
  void Decode(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsAcc(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsGyro(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsAngle(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsHeightAndTime(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsLatitudeLongitude(
      const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsSpeed(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsDataInfo(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeInsStd(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeGnssUtc(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeGpsLatitudeLongitude(
      const apollo::drivers::canbus::CanFrame &frame);
  void DecodeGpsHeightAndWeek(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeGpsHeadingAndPitch(const apollo::drivers::canbus::CanFrame &frame);
  void DecodeGpsStd(const apollo::drivers::canbus::CanFrame &frame);

  AsensingData adata_;
  uint32_t cur_can_id_;
};

Parser *Parser::CreateAsensingCan(const config::Config &config) {
  return new AsensingCanParser(config);
}

void AsensingCanParser::GetMessages(MessageInfoVec *messages) {
  if (data_ == nullptr) {
    return;
  }
  PrepareMessage();

  if (cur_can_id_ == FRAME_gps_latitude_longitude) {
    FillGnssBestpos();
    messages->push_back(MessageInfo{MessageType::BEST_GNSS_POS,
                                    reinterpret_cast<MessagePtr>(&bestpos_)});
    return;
  } else if (cur_can_id_ == FRAME_gps_heading_and_pitch) {
    FillHeading();
    messages->push_back(MessageInfo{MessageType::HEADING,
                                    reinterpret_cast<MessagePtr>(&heading_)});
    return;
  } else if (cur_can_id_ != FRAME_ins_height_and_time) {
    return;
  }

  FillImu();
  FillIns();
  FillInsStat();
  if (auto_fill_gps_msg_ && adata_.gps_latitude < 0 &&
      bestpos_ratecontrol_.check()) {
    FillGnssBestpos();
    messages->push_back(MessageInfo{MessageType::BEST_GNSS_POS,
                                    reinterpret_cast<MessagePtr>(&bestpos_)});
  }
  if (auto_fill_gps_msg_ && adata_.gps_heading < -0.1) {
    FillHeading();
    messages->push_back(MessageInfo{MessageType::HEADING,
                                    reinterpret_cast<MessagePtr>(&heading_)});
  }
  messages->push_back(
      MessageInfo{MessageType::IMU, reinterpret_cast<MessagePtr>(&imu_)});

  messages->push_back(
      MessageInfo{MessageType::INS, reinterpret_cast<MessagePtr>(&ins_)});
  messages->push_back(MessageInfo{MessageType::INS_STAT,
                                  reinterpret_cast<MessagePtr>(&ins_stat_)});
}

bool AsensingCanParser::PrepareMessage() {
  auto can_frame =
      reinterpret_cast<const apollo::drivers::canbus::CanFrame *>(data_);
  ADEBUG << "Received can frame: " << can_frame->CanFrameString();
  Decode(*can_frame);
  cur_can_id_ = can_frame->id;
  return true;
}

void AsensingCanParser::Decode(const apollo::drivers::canbus::CanFrame &frame) {
  switch (frame.id) {
    case FRAME_ins_acc:
      DecodeInsAcc(frame);
      break;
    case FRAME_ins_gyro:
      DecodeInsGyro(frame);
      break;
    case FRAME_ins_angle:
      DecodeInsAngle(frame);
      break;
    case FRAME_ins_height_and_time:
      DecodeInsHeightAndTime(frame);
      break;
    case FRAME_ins_latitude_longitude:
      DecodeInsLatitudeLongitude(frame);
      break;
    case FRAME_ins_speed:
      DecodeInsSpeed(frame);
      break;
    case FRAME_ins_datainfo:
      DecodeInsDataInfo(frame);
      break;
    case FRAME_ins_std:
      DecodeInsStd(frame);
      break;
    case FRAME_gnss_utc:
      DecodeGnssUtc(frame);
      break;
    case FRAME_gps_latitude_longitude:
      DecodeGpsLatitudeLongitude(frame);
      break;
    case FRAME_gps_height_and_week:
      DecodeGpsHeightAndWeek(frame);
      break;
    case FRAME_gps_heading_and_pitch:
      DecodeGpsHeadingAndPitch(frame);
      break;
    case FRAME_gps_std:
      DecodeGpsStd(frame);
      break;
    default:
      AWARN << "Unknown CAN frame id: " << std::hex << frame.id;
      break;
  }
}

void AsensingCanParser::DecodeInsAcc(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.acc_x = (ToValue<uint16_t, float>(frame.data, 0) * ACC_UNIT) * G_UNIT;
  adata_.acc_y = (ToValue<uint16_t, float>(frame.data, 2) * ACC_UNIT) * G_UNIT;
  adata_.acc_z = -(ToValue<uint16_t, float>(frame.data, 4) * ACC_UNIT) * G_UNIT;
}

void AsensingCanParser::DecodeInsGyro(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.gyro_x =
      (ToValue<uint16_t, float>(frame.data, 0) * GYRO_UNIT) * DEG_TO_RAD;
  adata_.gyro_y =
      (ToValue<uint16_t, float>(frame.data, 2) * GYRO_UNIT) * DEG_TO_RAD;
  adata_.gyro_z =
      (ToValue<uint16_t, float>(frame.data, 4) * GYRO_UNIT) * DEG_TO_RAD;
}
void AsensingCanParser::DecodeInsAngle(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.pitch = ToValue<uint16_t, float>(frame.data, 0) * ANGLE_UNIT;
  adata_.roll = ToValue<uint16_t, float>(frame.data, 2) * ANGLE_UNIT;
  adata_.heading = ToValue<uint16_t, float>(frame.data, 4) * ANGLE_UNIT;
  if (adata_.heading < 0) {
    adata_.heading += 360;
  }
}
void AsensingCanParser::DecodeInsHeightAndTime(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.altitude = ToValue<uint32_t, double>(frame.data, 0) * ALT_UINT;
  adata_.week_ms = ToValue<uint32_t>(frame.data, 4);
  adata_.ins_gps_sec = adata_.gps_week * SECONDS_PER_WEEK +
                       static_cast<double>(adata_.week_ms) / 1000;
}
void AsensingCanParser::DecodeInsLatitudeLongitude(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.latitude = ToValue<uint32_t, double>(frame.data, 0) * LOC_UNIT;
  adata_.longitude = ToValue<uint32_t, double>(frame.data, 4) * LOC_UNIT;
}
void AsensingCanParser::DecodeInsSpeed(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.vn = ToValue<uint16_t, float>(frame.data, 0) * SPEED_UNIT;
  adata_.ve = ToValue<uint16_t, float>(frame.data, 2) * SPEED_UNIT;
  adata_.vd = ToValue<uint16_t, float>(frame.data, 4) * SPEED_UNIT;
}
void AsensingCanParser::DecodeInsDataInfo(
    const apollo::drivers::canbus::CanFrame &frame) {
  uint8_t flag_pos = frame.data[0];
  if (SolutionType_IsValid(flag_pos)) {
    solution_type_ = SolutionType(flag_pos);
  } else {
    solution_type_ = SolutionType::NONE;
  }
  adata_.differential_age = frame.data[3];
  switch (frame.data[7]) {
    case 2:
      solution_status_ = SolutionStatus::SOL_COMPUTED;
      break;
    default:
      solution_status_ = SolutionStatus::INSUFFICIENT_OBS;
  }
}
void AsensingCanParser::DecodeInsStd(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.lat_std = ToValue<uint16_t, float>(frame.data, 0) * LOC_STD_UNIT;
  adata_.lon_std = ToValue<uint16_t, float>(frame.data, 2) * LOC_STD_UNIT;
  adata_.alti_std = ToValue<uint16_t, float>(frame.data, 4) * LOC_STD_UNIT;
  adata_.yaw_std = ToValue<uint16_t, float>(frame.data, 6) * HEADING_STD_UNIT;
}
void AsensingCanParser::DecodeGnssUtc(
    const apollo::drivers::canbus::CanFrame &frame) {
  struct tm utc_tm = {.tm_sec = frame.data[5],
                      .tm_min = frame.data[4],
                      .tm_hour = frame.data[3],
                      .tm_mday = frame.data[2],
                      .tm_mon = frame.data[1] - 1,
                      .tm_year = frame.data[0] + 2000 - 1900};
  time_t timestamp = mktime(&utc_tm);
  double ms = ToValue<uint16_t, double>(frame.data, 6) * 0.001;
  if (timestamp != -1) {
    adata_.gps_sec =
        apollo::drivers::util::unix2gps(static_cast<double>(timestamp) + ms);
  } else {
    AERROR << "Invalid utc time: " << utc_tm.tm_year << "-" << utc_tm.tm_mon
           << "-" << utc_tm.tm_mday << " " << utc_tm.tm_hour << ":"
           << utc_tm.tm_min << ":" << utc_tm.tm_sec << "." << ms;
  }
}

void AsensingCanParser::DecodeGpsLatitudeLongitude(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.gps_latitude = ToValue<uint32_t, double>(frame.data, 0) * LOC_UNIT;
  adata_.gps_longitude = ToValue<uint32_t, double>(frame.data, 4) * LOC_UNIT;
}
void AsensingCanParser::DecodeGpsHeightAndWeek(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.gps_altitude = ToValue<uint32_t, double>(frame.data, 0) * ALT_UINT;
  adata_.gps_week = ToValue<uint32_t>(frame.data, 4);
}
void AsensingCanParser::DecodeGpsHeadingAndPitch(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.gps_heading = ToValue<uint16_t, float>(frame.data, 0) * ANGLE_UNIT;
  if (adata_.gps_heading < 0) {
    adata_.gps_heading += 360;
  }
  adata_.gps_pitch = ToValue<uint16_t, double>(frame.data, 2) * ANGLE_UNIT;
}
void AsensingCanParser::DecodeGpsStd(
    const apollo::drivers::canbus::CanFrame &frame) {
  adata_.gps_lat_std = ToValue<uint16_t, float>(frame.data, 0) * LOC_STD_UNIT;
  adata_.gps_lon_std = ToValue<uint16_t, float>(frame.data, 2) * LOC_STD_UNIT;
  adata_.gps_alti_std = ToValue<uint16_t, float>(frame.data, 4) * LOC_STD_UNIT;
  adata_.gps_yaw_std =
      ToValue<uint16_t, float>(frame.data, 6) * HEADING_STD_UNIT;
}

void AsensingCanParser::FillGnssBestpos() {
  bestpos_.set_measurement_time(adata_.gps_sec);
  bestpos_.set_sol_status(solution_status_);
  bestpos_.set_sol_type(solution_type_);
  if (adata_.gps_latitude > 0) {
    bestpos_.set_latitude(adata_.gps_latitude);
    bestpos_.set_longitude(adata_.gps_longitude);
    bestpos_.set_height_msl(adata_.gps_altitude);
    bestpos_.set_latitude_std_dev(adata_.gps_lat_std);
    bestpos_.set_longitude_std_dev(adata_.gps_lon_std);
    bestpos_.set_height_std_dev(adata_.gps_alti_std);
  } else {
    bestpos_.set_latitude(adata_.latitude);
    bestpos_.set_longitude(adata_.longitude);
    bestpos_.set_height_msl(adata_.altitude);
    bestpos_.set_latitude_std_dev(adata_.lat_std);
    bestpos_.set_longitude_std_dev(adata_.lon_std);
    bestpos_.set_height_std_dev(adata_.alti_std);
  }
  bestpos_.set_differential_age(adata_.differential_age);
  // bestpos_.set_num_sats_tracked(0);
  // bestpos_.set_num_sats_in_solution(0);
  // bestpos_.set_num_sats_l1(0);
  // bestpos_.set_num_sats_multi(0);
}

void AsensingCanParser::FillIns() {
  ins_.mutable_euler_angles()->set_x(adata_.roll * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_y(-adata_.pitch * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(adata_.heading));

  ins_.mutable_position()->set_lon(adata_.longitude);
  ins_.mutable_position()->set_lat(adata_.latitude);
  ins_.mutable_position()->set_height(adata_.altitude);
  ins_.mutable_linear_velocity()->set_x(adata_.ve);
  ins_.mutable_linear_velocity()->set_y(adata_.vn);
  ins_.mutable_linear_velocity()->set_z(-adata_.vd);
  rfu_to_flu(adata_.acc_x, adata_.acc_y, adata_.acc_z,
             ins_.mutable_linear_acceleration());
  rfu_to_flu(adata_.gyro_x, adata_.gyro_y, adata_.gyro_z,
             ins_.mutable_angular_velocity());
  ins_.set_measurement_time(adata_.ins_gps_sec);
  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());

  switch (solution_type_) {
    case SolutionType::INS_RTKFIXED:
    case SolutionType::NARROW_INT:
    case SolutionType::INS_RTKFLOAT:
    case SolutionType::NARROW_FLOAT:
      ins_.set_type(Ins::GOOD);
      break;
    case SolutionType::SINGLE:
      ins_.set_type(Ins::CONVERGING);
      break;
    default:
      ins_.set_type(Ins::INVALID);
      break;
  }
}

void AsensingCanParser::FillInsStat() {
  ins_stat_.set_ins_status(solution_status_);
  ins_stat_.set_pos_type(solution_type_);
}

void AsensingCanParser::FillImu() {
  rfu_to_flu(adata_.acc_x, adata_.acc_y, adata_.acc_z,
             imu_.mutable_linear_acceleration());
  rfu_to_flu(adata_.gyro_x, adata_.gyro_x, adata_.gyro_z,
             imu_.mutable_angular_velocity());
  imu_.set_measurement_time(adata_.ins_gps_sec);
}

void AsensingCanParser::FillHeading() {
  heading_.set_solution_status(solution_status_);
  heading_.set_position_type(solution_type_);
  heading_.set_measurement_time(adata_.gps_sec);
  if (adata_.gps_heading > -0.1) {
    heading_.set_heading(adata_.gps_heading);
    heading_.set_pitch(adata_.gps_pitch);
    heading_.set_heading_std_dev(adata_.gps_yaw_std);
  } else {
    heading_.set_heading(adata_.heading);
    heading_.set_pitch(adata_.pitch);
    heading_.set_heading_std_dev(adata_.yaw_std);
  }
  heading_.set_pitch_std_dev(adata_.pitch_std);

  // heading_.set_station_id("0");
  // heading_.set_satellite_number_multi(0);
  // heading_.set_satellite_soulution_number(0);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
