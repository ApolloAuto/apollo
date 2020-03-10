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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <thread>

#include "cyber/common/log.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "modules/bridge/common/bridge_proto_serialized_buf.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/time/time.h"

using apollo::common::time::Clock;

bool send(const std::string &remote_ip, uint16_t remote_port, uint32_t count) {
  if (count == 0) {
    count = 10000;
  }
  float total = static_cast<float>(count);
  float hundred = 100.00;
  for (uint32_t i = 0; i < count; i++) {
    double timestamp_ = Clock::NowInSeconds() - 2.0;
    float coefficient = static_cast<float>(i);
    auto pb_msg = std::make_shared<apollo::canbus::Chassis>();
    pb_msg->mutable_header()->set_sequence_num(i);
    pb_msg->mutable_header()->set_timestamp_sec(timestamp_);
    pb_msg->set_engine_started(true);
    pb_msg->set_engine_rpm(static_cast<float>(coefficient * 2.0));
    pb_msg->set_odometer_m(coefficient);
    pb_msg->set_fuel_range_m(100);
    pb_msg->set_throttle_percentage(coefficient * hundred / total);
    pb_msg->set_brake_percentage(coefficient * hundred / total);
    pb_msg->set_steering_percentage(coefficient * hundred / total);
    pb_msg->set_steering_torque_nm(coefficient);
    pb_msg->set_parking_brake(i % 2);
    pb_msg->set_high_beam_signal(false);
    pb_msg->set_low_beam_signal(true);
    pb_msg->set_left_turn_signal(false);
    pb_msg->set_right_turn_signal(false);
    pb_msg->set_horn(false);
    pb_msg->set_wiper(false);
    pb_msg->set_disengage_status(false);
    pb_msg->set_driving_mode(apollo::canbus::Chassis::COMPLETE_MANUAL);
    pb_msg->set_error_code(apollo::canbus::Chassis::NO_ERROR);
    pb_msg->set_gear_location(apollo::canbus::Chassis::GEAR_NEUTRAL);

    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(remote_port);

    ADEBUG << "connecting to server... ";

    int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

    int res =
        connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (res < 0) {
      ADEBUG << "connected server failed ";
      continue;
    }

    ADEBUG << "connected to server success. port [" << remote_port << "]";

    apollo::bridge::BridgeProtoSerializedBuf<apollo::canbus::Chassis> proto_buf;
    proto_buf.Serialize(pb_msg, "Chassis");
    for (size_t j = 0; j < proto_buf.GetSerializedBufCount(); j++) {
      ssize_t nbytes = send(sock_fd, proto_buf.GetSerializedBuf(j),
                            proto_buf.GetSerializedBufSize(j), 0);
      if (nbytes != static_cast<ssize_t>(proto_buf.GetSerializedBufSize(j))) {
        ADEBUG << "sent msg failed ";
        break;
      }
      ADEBUG << "sent " << nbytes << " bytes to server with sequence num " << i;
    }
    close(sock_fd);

    // 1000Hz
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

int main(int argc, char *argv[]) {
  uint32_t count = 0;
  if (argc < 2) {
    count = 10000;
  } else {
    count = atoi(argv[1]);
  }
  send("127.0.0.1", 8900, count);
  return 0;
}
