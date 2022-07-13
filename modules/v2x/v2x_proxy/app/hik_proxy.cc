/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
/**
 * @file v2x_proxy.cc
 * @brief define v2x proxy class
 */
#include "modules/v2x/v2x_proxy/app/hik_proxy.h"

#include <netinet/in.h>
#include <sys/socket.h>

#include <cmath>
#include <cstdio>
#include <ostream>
#include <vector>

#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/v2x/proto/hik_message.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/v2x/common/v2x_proxy_gflags.h"

#define FRAME_SIZE 1024

namespace apollo {
namespace v2x {

using ::apollo::canbus::Chassis;
using ::apollo::hdmap::adapter::CoordinateConvertTool;
using ::apollo::localization::LocalizationEstimate;
using ::apollo::perception::TrafficLight;
using ::apollo::perception::TrafficLightDetection;
using ::apollo::v2x::CarStatus;
using ::apollo::v2x::obu::ObuRsi;
using ::apollo::v2x::obu::ObuTrafficLight;

std::string constract_header(uint8_t type, std::string &json_str) {
  uint32_t json_length = json_str.length();
  unsigned char array[20] = {0x5A, 0xA5, 0x00, 0x01, 0x00, 0x01, 0x00,
                             0x00, 0x00, 0x03, 0x00, type, 0x00, 0x00,
                             0x00, 0x05, 0x00, 0x00, 0x00, 0x00};
  array[19] = static_cast<char>(json_length);
  array[18] = static_cast<char>(json_length >> 8);
  array[17] = static_cast<char>(json_length >> 16);
  array[16] = static_cast<char>(json_length >> 24);
  std::string head((char *)array, 20);
  head.append(json_str);
  char res = 0;
  for (auto ch : head) {
    res = res ^ ch;
  }
  head.append(1, res);
  return head;
}

HikProxy::~HikProxy() {
  close(sock_fd_);
  AINFO << "recv thread exit";
}

bool HikProxy::MsgHandle(int fd) {
  struct sockaddr_in client_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));
  size_t total_recv = 2 * FRAME_SIZE;
  unsigned char total_buf[2 * FRAME_SIZE] = {0};
  size_t bytes =
      static_cast<int>(recvfrom(fd, total_buf, total_recv, 0,
                                (struct sockaddr *)&client_addr, &sock_len));
  if (bytes <= 0 || bytes > total_recv) {
    return false;
  }
  if (total_buf[0] != 0x5A || total_buf[1] != 0xA5) {
    AERROR << "start flag is wrong, get wrong message" << std::hex
           << total_buf[0] << "," << std::hex << total_buf[1] << " byte"
           << bytes;
    return false;
  }
  if (bytes < 21) {
    AERROR << "recevie message is too short";
    return false;
  }
  size_t len = (total_buf[16] << 8) + total_buf[17];
  len = (len << 8) + total_buf[18];
  len = (len << 8) + total_buf[19];
  if (bytes - 20 < len) {
    AERROR << "json data is not enough";
    return false;
  }
  std::string str((char *)(total_buf + 20), len);
  Spat spat;
  if (!JsonParse::toProto(str, &spat, "spat")) return true;
  ADEBUG << "RECEIVE: " << str << std::endl;
  TrafficLightDetection tf_pb;
  for (auto &signal : spat.signalphase()) {
    auto tf = tf_pb.add_traffic_light();
    switch (signal.ledstatus()) {
      case 3:
        tf->set_color(TrafficLight::RED);
        break;
      case 7:
        tf->set_color(TrafficLight::YELLOW);
        break;
      case 5:
        tf->set_color(TrafficLight::GREEN);
        break;
      default:
        break;
    }
    tf->set_id("TL" + std::to_string(spat.nodeid()) + "_" +
               std::to_string(signal.lightid()));
    tf->set_remaining_time(signal.waittime());
  }
  common::util::FillHeader("v2x", &tf_pb);
  udp_traffic_light_writer_->Write(tf_pb);
  return true;
}

HikProxy::HikProxy()
    : node_(::apollo::cyber::CreateNode("v2x_proxy")), exit_(false) {
  if (node_ == nullptr) {
    AFATAL << "Create v2x proxy node failed.";
    exit(1);
  }
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  udp_traffic_light_writer_ =
      node_->CreateWriter<::apollo::perception::TrafficLightDetection>(
          "/apollo/perception/traffic_light");
  canbus_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<const Chassis> &msg) {
        {
          std::lock_guard<std::mutex> lg(mutex_canbus_);
          chassis_msg_.Clear();
          chassis_msg_.CopyFrom(*msg);
        }
      });
  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic,
      [this](const std::shared_ptr<const ADCTrajectory> &msg) {
        {
          std::lock_guard<std::mutex> lg(mutex_canbus_);
          planning_msg_.Clear();
          planning_msg_.CopyFrom(*msg);
        }
      });
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [this](const std::shared_ptr<const LocalizationEstimate> &msg) {
        {
          std::lock_guard<std::mutex> lg(mutex_localization_);
          this->localization_msg_.Clear();
          this->localization_msg_.CopyFrom(*msg);
        }
      });
  AINFO << "client(locol host) port is: " << FLAGS_local_host_port;
  uint16_t client_port = atoi(FLAGS_local_host_port.c_str());
  listener_->Initialize(this, &HikProxy::MsgHandle, client_port);
  hdmap_ = std::make_shared<::apollo::hdmap::HDMap>();
  const auto hdmap_file = apollo::hdmap::BaseMapFile();
  if (0 != hdmap_->LoadMapFromFile(hdmap_file)) {
    AERROR << "Failed to load hdmap file: " << hdmap_file;
    return;
  }
  AINFO << "Load hdmap file is: " << hdmap_file;
  ::apollo::cyber::TimerOption v2x_car_status_timer_option;
  v2x_car_status_timer_option.period =
      static_cast<uint32_t>((1000 + FLAGS_v2x_car_status_timer_frequency - 1) /
                            FLAGS_v2x_car_status_timer_frequency);
  v2x_car_status_timer_option.callback = [this]() {
    this->OnV2xCarStatusTimer();
  };
  v2x_car_status_timer_option.oneshot = false;
  v2x_car_status_timer_.reset(
      new ::apollo::cyber::Timer(v2x_car_status_timer_option));

  v2x_car_status_timer_->Start();
  AINFO << "obu_server_host is: " << FLAGS_obu_host_ip
        << ", server_port is: " << FLAGS_obu_host_port;
  server_addr_.sin_addr.s_addr =
      inet_addr(static_cast<std::string>(FLAGS_obu_host_ip).c_str());
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port =
      htons(static_cast<uint16_t>(atoi(FLAGS_obu_host_port.c_str())));
  sock_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  // int res =
  //   connect(sock_fd_, (struct sockaddr *)&server_addr,
  //   sizeof(server_addr));
  // if (res < 0) {
  //  close(sock_fd_);
  // AERROR<<"can not etablish connect";
  CoordinateConvertTool::GetInstance()->SetConvertParam(from_coordinate_,
                                                        to_coordinate_);
  init_flag_ = true;
  this->MsgDispatcher();
}

void HikProxy::MsgDispatcher() {
  AINFO << "START LISTEN";
  listener_->Listen();
  AINFO << "FINISH LISTEN";
}

void HikProxy::OnV2xCarStatusTimer() {
  // get loc
  if (init_flag_ == false) return;
  if (!localization_msg_.has_pose() || !chassis_msg_.has_speed_mps()) return;
  OBULocalization obu_localization;
  auto pose = localization_msg_.mutable_pose()->mutable_position();
  double lon(0), lat(0), alt(0);
  auto status = CoordinateConvertTool::GetInstance()->CoordiateConvert(
      pose->x(), pose->y(), pose->z(), &lon, &lat, &alt);
  obu_localization.set_longitude(lon);
  obu_localization.set_latitude(lat);
  obu_localization.set_speed(chassis_msg_.speed_mps() * 3.6);
  double raw_heading =
      -localization_msg_.mutable_pose()->heading() * RAD_TO_DEG + 90;
  if (raw_heading < 0) raw_heading += 360;
  obu_localization.set_heading(raw_heading);  // hik north 0 deg  clockwise
  uint64_t cur_time = ::apollo::cyber::Time::Now().ToMicrosecond();
  obu_localization.set_timestamp(cur_time);
  std::string json_str;
  JsonParse::toJsonString(obu_localization, &json_str);
  std::string res = constract_header(3, json_str);
  ssize_t nbytes = sendto(sock_fd_, res.c_str(), res.length(), 0,
                          (sockaddr *)&server_addr_, sizeof(server_addr_));
  if (nbytes == -1) AERROR << "Send OBU Localization error";
  OBUCarID car_id;
  car_id.set_carnum("D-KIT");
  car_id.set_cartype(4);
  json_str.clear();
  JsonParse::toJsonString(car_id, &json_str);
  res = constract_header(0, json_str);
  // nbytes = send(sock_fd_,res.c_str(),res.length(), 0);
  nbytes = sendto(sock_fd_, res.c_str(), res.length(), 0,
                  (sockaddr *)&server_addr_, sizeof(server_addr_));
  if (nbytes == -1) AERROR << "Send OBU car id error";
  VehicleData vehicle_data;
  vehicle_data.set_vehiclespeed(chassis_msg_.speed_mps() * 3.6);
  if (chassis_msg_.has_steering_percentage())
    vehicle_data.set_steerwheelangle(chassis_msg_.steering_percentage() *
                                     vehicle_param_.max_steer_angle());
  else
    vehicle_data.set_steerwheelangle(0);
  vehicle_data.set_brakepedalposition(chassis_msg_.brake_percentage());
  switch (chassis_msg_.gear_location()) {
    case Chassis::GEAR_NEUTRAL:
      vehicle_data.set_gear(2);
      break;
    case Chassis::GEAR_DRIVE:
      vehicle_data.set_gear(0);
      break;
    case Chassis::GEAR_PARKING:
      vehicle_data.set_gear(3);
      break;
    case Chassis::GEAR_REVERSE:
      vehicle_data.set_gear(1);
      break;
    default:
      vehicle_data.set_gear(2);
  }
  vehicle_data.set_parkingbrake(chassis_msg_.parking_brake());
  vehicle_data.set_turnsignal(0);
  switch (planning_msg_.mutable_decision()
              ->mutable_vehicle_signal()
              ->turn_signal()) {
    case apollo::common::VehicleSignal::TURN_LEFT:
      vehicle_data.set_turnsignal(1);
      break;
    case apollo::common::VehicleSignal::TURN_RIGHT:
      vehicle_data.set_turnsignal(2);
      break;
    case apollo::common::VehicleSignal::TURN_NONE:
      vehicle_data.set_turnsignal(0);
      break;
    default:
      break;
  }
  vehicle_data.set_hazardlight(0);
  if (chassis_msg_.driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    vehicle_data.set_drivermodel(0);
  } else {
    vehicle_data.set_drivermodel(2);
  }
  vehicle_data.set_emergencyparking(0);
  json_str.clear();
  JsonParse::toJsonString(vehicle_data, &json_str);
  res = constract_header(1, json_str);
  // nbytes = send(sock_fd_,res.c_str(),res.length(), 0);
  nbytes = sendto(sock_fd_, res.c_str(), res.length(), 0,
                  (sockaddr *)&server_addr_, sizeof(server_addr_));
  if (nbytes == -1) AERROR << "Send OBU car id error";
}

}  // namespace v2x
}  // namespace apollo
