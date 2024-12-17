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

// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
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
#include "modules/drivers/gnss/parser/asensing_parser/protocol_asensing.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/parser_common.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

class AsensingParser : public Parser {
 public:
  AsensingParser() {}
  explicit AsensingParser(const config::Config &config);

  virtual void GetMessages(MessageInfoVec *messages);

 private:
  void PrepareMessage();

  void FillGnssBestpos();
  void FillIns();
  void FillInsStat();
  void FillImu();
  void FillHeading();

  SolutionStatus solution_status_ = SolutionStatus::INSUFFICIENT_OBS;
  SolutionType solution_type_ = SolutionType::NONE;
  double gps_sec_;

  GnssBestPose bestpos_;
  // bestpos 1hz rate control
  RateControl bestpos_ratecontrol_{PERIOD_NS_1HZ};
  Imu imu_;
  Heading heading_;
  Ins ins_;
  InsStat ins_stat_;

  ProtocolAsensing asensing;
  Decode_0A decode_a;
  Decode_0B decode_b;
};

Parser *Parser::CreateAsensing(const config::Config &config) {
  return new AsensingParser(config);
}

AsensingParser::AsensingParser(const config::Config &config) {}

void AsensingParser::GetMessages(MessageInfoVec *messages) {
  if (data_ == nullptr) {
    return;
  }
  asensing.addData(
      std::string(reinterpret_cast<const char *>(data_), data_end_ - data_));
  if (asensing.getProtocol() != decode_b.m_typeImu) {
    return;
  }
  PrepareMessage();

  FillGnssBestpos();
  FillImu();
  FillHeading();
  FillIns();
  FillInsStat();

  if (bestpos_ratecontrol_.check()) {
    messages->push_back(MessageInfo{MessageType::BEST_GNSS_POS,
                                    reinterpret_cast<MessagePtr>(&bestpos_)});
  }
  messages->push_back(
      MessageInfo{MessageType::IMU, reinterpret_cast<MessagePtr>(&imu_)});
  messages->push_back(MessageInfo{MessageType::HEADING,
                                  reinterpret_cast<MessagePtr>(&heading_)});
  messages->push_back(
      MessageInfo{MessageType::INS, reinterpret_cast<MessagePtr>(&ins_)});
  messages->push_back(MessageInfo{MessageType::INS_STAT,
                                  reinterpret_cast<MessagePtr>(&ins_stat_)});
}

void AsensingParser::PrepareMessage() {
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
}

void AsensingParser::FillGnssBestpos() {
  bestpos_.set_measurement_time(gps_sec_);
  bestpos_.set_sol_status(solution_status_);
  bestpos_.set_sol_type(solution_type_);
  bestpos_.set_latitude(decode_b.insdata.Lat_deg);
  bestpos_.set_longitude(decode_b.insdata.Lon_deg);
  bestpos_.set_height_msl(decode_b.insdata.Alt_m);

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

void AsensingParser::FillIns() {
  ins_.mutable_euler_angles()->set_x(decode_b.insdata.Roll_rad);
  ins_.mutable_euler_angles()->set_y(-decode_b.insdata.Pitch_rad);
  ins_.mutable_euler_angles()->set_z(decode_b.insdata.Yaw_rad);

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

void AsensingParser::FillInsStat() {
  ins_stat_.set_ins_status(solution_status_);
  ins_stat_.set_pos_type(solution_type_);
}

void AsensingParser::FillImu() {
  rfu_to_flu(decode_b.insdata.AccX_g, decode_b.insdata.AccY_g,
             decode_b.insdata.AccZ_g, imu_.mutable_linear_acceleration());
  rfu_to_flu(decode_b.insdata.GyroX, decode_b.insdata.GyroY,
             decode_b.insdata.GyroZ, imu_.mutable_angular_velocity());
  imu_.set_measurement_time(gps_sec_);
}

void AsensingParser::FillHeading() {
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
