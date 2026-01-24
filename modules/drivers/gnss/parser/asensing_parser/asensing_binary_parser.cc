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
#include "modules/drivers/gnss/parser/asensing_parser/asensing_base_parser.h"
#include "modules/drivers/gnss/parser/asensing_parser/protocol_asensing.h"

namespace apollo {
namespace drivers {
namespace gnss {

class AsensingBinaryParser : public AsensingBaseParser {
 public:
  explicit AsensingBinaryParser(const config::Config &config)
      : AsensingBaseParser(config) {}
  bool PrepareMessage() override;

  void FillGnssBestpos() override;
  void FillIns() override;
  void FillInsStat() override;
  void FillImu() override;
  void FillHeading() override;

 private:
  ProtocolAsensing asensing;
  Decode_0A decode_a;
  Decode_0B decode_b;
};

Parser *Parser::CreateAsensingBinary(const config::Config &config) {
  return new AsensingBinaryParser(config);
}

bool AsensingBinaryParser::PrepareMessage() {
  asensing.addData(
      std::string(reinterpret_cast<const char *>(data_), data_end_ - data_));
  AINFO << "get protocol: " << asensing.getProtocol();
  if (asensing.getProtocol() != decode_b.m_typeImu) {
    return false;
  }
  if ((decode_b.insdata.InsStatus & 0x0F) == 0x0F) {
    solution_status_ = SolutionStatus::SOL_COMPUTED;
  } else {
    solution_status_ = SolutionStatus::INSUFFICIENT_OBS;
  }
  if (SolutionType_IsValid(decode_b.insdata.flag_pos)) {
    solution_type_ = SolutionType(decode_b.insdata.flag_pos);
  } else {
    solution_type_ = SolutionType::NONE;
  }
  gps_sec_ =
      apollo::drivers::util::unix2gps(decode_b.insdata.SysTime_ms / 1000.0);
  return true;
}

void AsensingBinaryParser::FillGnssBestpos() {
  bestpos_.set_measurement_time(gps_sec_);
  bestpos_.set_sol_status(solution_status_);
  bestpos_.set_sol_type(solution_type_);
  bestpos_.set_latitude(decode_b.insdata.Lat_gnss_deg);
  bestpos_.set_longitude(decode_b.insdata.Lon_gnss_deg);
  bestpos_.set_height_msl(decode_b.insdata.Alt_gnss_m);

  if (decode_b.insdata.LoopType == 0) {
    bestpos_.set_latitude_std_dev(exp(decode_b.insdata.data1 / 100));
    bestpos_.set_longitude_std_dev(exp(decode_b.insdata.data2 / 100));
    bestpos_.set_height_std_dev(exp(decode_b.insdata.data3 / 100));
  }
  bestpos_.set_differential_age(decode_b.insdata.differential_age);
  // bestpos_.set_num_sats_tracked(0);
  // bestpos_.set_num_sats_in_solution(0);
  // bestpos_.set_num_sats_l1(0);
  // bestpos_.set_num_sats_multi(0);
}

void AsensingBinaryParser::FillIns() {
  ins_.mutable_euler_angles()->set_x(decode_b.insdata.Roll_deg * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_y(-decode_b.insdata.Pitch_deg * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_z(
      azimuth_deg_to_yaw_rad(decode_b.insdata.Yaw_deg));

  ins_.mutable_position()->set_lon(decode_b.insdata.Lon_deg);
  ins_.mutable_position()->set_lat(decode_b.insdata.Lat_deg);
  ins_.mutable_position()->set_height(decode_b.insdata.Alt_m);
  ins_.mutable_linear_velocity()->set_x(decode_b.insdata.VelE_mps);
  ins_.mutable_linear_velocity()->set_y(decode_b.insdata.VelN_mps);
  ins_.mutable_linear_velocity()->set_z(-decode_b.insdata.VelD_mps);
  rfu_to_flu(decode_b.insdata.AccX_g, decode_b.insdata.AccY_g,
             decode_b.insdata.AccZ_g, ins_.mutable_linear_acceleration());
  rfu_to_flu(decode_b.insdata.GyroX, decode_b.insdata.GyroY,
             decode_b.insdata.GyroZ, ins_.mutable_angular_velocity());
  ins_.set_measurement_time(gps_sec_);
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

void AsensingBinaryParser::FillInsStat() {
  ins_stat_.set_ins_status(solution_status_);
  ins_stat_.set_pos_type(solution_type_);
}

void AsensingBinaryParser::FillImu() {
  rfu_to_flu(decode_b.insdata.AccX_g, decode_b.insdata.AccY_g,
             decode_b.insdata.AccZ_g, imu_.mutable_linear_acceleration());
  rfu_to_flu(decode_b.insdata.GyroX, decode_b.insdata.GyroY,
             decode_b.insdata.GyroZ, imu_.mutable_angular_velocity());
  imu_.set_measurement_time(gps_sec_);
}

void AsensingBinaryParser::FillHeading() {
  heading_.set_solution_status(solution_status_);
  heading_.set_position_type(solution_type_);
  heading_.set_measurement_time(gps_sec_);
  heading_.set_heading(decode_b.insdata.Yaw_deg);
  heading_.set_pitch(decode_b.insdata.Pitch_deg);

  if (decode_b.insdata.LoopType == 2) {
    heading_.set_heading_std_dev(exp(decode_b.insdata.data3 / 100));
    heading_.set_pitch_std_dev(exp(decode_b.insdata.data2 / 100));
  }

  // heading_.set_station_id("0");
  // heading_.set_satellite_number_multi(0);
  // heading_.set_satellite_soulution_number(0);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
