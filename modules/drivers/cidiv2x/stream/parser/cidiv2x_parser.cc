/******************************************************************************
 * Copyright 2019 The CiDi Authors. All Rights Reserved.
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

// An parser for decoding binary messages from a CiDi OBU device. The following
// messages must be
// logged in order for this parser to work properly.
//

#include "modules/drivers/cidiv2x/stream/parser/cidiv2x_parser.h"

#include <arpa/inet.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/cidiv2x/proto/cidiv2x.pb.h"
#include "modules/drivers/cidiv2x/stream/util/macros.h"

namespace apollo {
namespace drivers {
namespace cidiv2x {

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr size_t BUFFER_SIZE = 1024;

constexpr size_t MSG_SIZE = 320;

}  // namespace

using ::apollo::drivers::CiDiV2X;

bool CidiV2xParser::Init() {
  cidiv2x_writer_ = node_->CreateWriter<CiDiV2X>(FLAGS_cidiv2x_topic);
  is_inited_ = true;
  return true;
}

CidiV2xParser::CidiV2xParser() { buffer_.reserve(BUFFER_SIZE); }

CidiV2xParser::CidiV2xParser(const config::Config& config,
                             const std::shared_ptr<apollo::cyber::Node>& node)
    : node_(node) {
  buffer_.reserve(BUFFER_SIZE);
}

bool CidiV2xParser::ParseRawData(const std::string& msg) {
  ADEBUG << "Update the data.";
  Update(msg);
  ADEBUG << "Call GetMessage.";
  return GetMessage();
}

bool CidiV2xParser::GetMessage() {
  bool result = false;
  if (data_ == nullptr) {
    ADEBUG << "data_ == nullptr";
    return result;
  }
  if (data_end_ - data_ < (uint16_t)MSG_SIZE) {
    AWARN << "data size less than MSG_SIZE: " << MSG_SIZE;
    return result;
  }

  while (data_ < data_end_) {
    if (buffer_.size() == 0) {  // Looking for SYNC0
      if (*data_ == v2xmsg::SYNC_0) {
        buffer_.push_back(*data_);
      }
      ++data_;
    } else if (buffer_.size() == 1) {  // Looking for SYNC1
      if (*data_ == v2xmsg::SYNC_1) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 2) {  // Looking for SYNC2
      if (*data_ == v2xmsg::SYNC_2) {
        buffer_.push_back(*data_++);
      } else {
        buffer_.clear();
      }
    } else if (buffer_.size() == 3) {  // Looking for SYNC3
      if (*data_ == v2xmsg::SYNC_3) {
        buffer_.push_back(*data_++);
        total_length_ = MSG_SIZE;
      } else {
        buffer_.clear();
      }
    } else if (total_length_ > 0) {
      if (buffer_.size() < total_length_) {
        buffer_.push_back(*data_++);
        continue;
      }
      ADEBUG << "Call PrepareMessage, buffer_ size: " << buffer_.size();
      result = PrepareMessage();
      buffer_.clear();
      total_length_ = 0;
    }
  }

  return result;
}

bool CidiV2xParser::PrepareMessage() {
  std::lock_guard<std::mutex> lock(cidiv2x_mutex_);
  cidiv2x_.Clear();
  uint8_t* message = nullptr;
  message = buffer_.data() + 4;
  if (!HandleCurrentCarInfo(
          reinterpret_cast<v2xmsg::CurrentCarInfo*>(message))) {
    return false;
  }
  message = buffer_.data() + 128;
  if (!HandleLaneInfo(reinterpret_cast<v2xmsg::LaneInfo*>(message))) {
    return false;
  }
  message = buffer_.data() + 160;
  if (!HandleTrafficLightInfo(
          reinterpret_cast<v2xmsg::TrafficLightInfo*>(message))) {
    return false;
  }
  message = buffer_.data() + 192;
  if (!HandleSignInfo(reinterpret_cast<v2xmsg::SignInfo*>(message))) {
    return false;
  }

  common::util::FillHeader("cidiv2x", &cidiv2x_);
  ADEBUG << cidiv2x_.ShortDebugString();
  ADEBUG << "Publish Cidiv2x.";
  cidiv2x_writer_->Write(std::make_shared<CiDiV2X>(cidiv2x_));
  return true;
}

bool CidiV2xParser::HandleCurrentCarInfo(const v2xmsg::CurrentCarInfo* car) {
  auto* car_info = cidiv2x_.mutable_current_car_info();
  car_info->set_message_cycle_count(ntohl(car->message_cycle_count));
  car_info->set_car_id(ntohl(car->car_id));
  car_info->set_car_length(ntohs(car->car_length));
  car_info->set_car_width(ntohs(car->car_width));
  car_info->set_localization_type(
      static_cast<LocalizationType>(car->localization_type));
  car_info->set_num_statellites(car->num_statellites);
  car_info->set_latitude(ntohl(car->latitude));
  car_info->set_longitude(ntohl(car->longitude));
  car_info->set_height_msl(ntohl(car->height_msl));
  car_info->set_heading(ntohs(car->heading));
  car_info->set_gps_velocity(ntohs(car->gps_velocity));
  car_info->set_gps_acceleration(ntohs(car->gps_acceleration));

  return true;
}
bool CidiV2xParser::HandleLaneInfo(const v2xmsg::LaneInfo* lane) {
  auto* lane_info = cidiv2x_.mutable_lane_info();
  lane_info->set_lane_cycle_count(ntohl(lane->lane_cycle_count));
  lane_info->set_in_map(lane->in_map);
  lane_info->set_in_lane(lane->in_lane);
  lane_info->set_total_lane(lane->total_lane);
  lane_info->set_current_lane_id(lane->current_lane_id);
  for (uint8_t i = 0; i < 8; i++) {
    auto* flag = lane_info->add_lane_flags();
    flag->set_lane_id(i + 1);
    bool turn = (lane->lane_flags[i] & 0x01) == 0x01;
    flag->set_allow_left_turn(turn);
    turn = (lane->lane_flags[i] & 0x02) == 0x02;
    flag->set_allow_go_straight(turn);
    turn = (lane->lane_flags[i] & 0x04) == 0x04;
    flag->set_allow_right_turn(turn);
  }

  return true;
}
bool CidiV2xParser::HandleTrafficLightInfo(
    const v2xmsg::TrafficLightInfo* tflight) {
  auto* tflight_info = cidiv2x_.mutable_traffic_light_info();
  tflight_info->set_traffic_light_cycle_count(
      ntohl(tflight->traffic_light_cycle_count));
  auto* current_lane_info =
      cidiv2x_.mutable_traffic_light_info()->mutable_current_lane_light();
  current_lane_info->set_receive_flags(tflight->receive_flags);
  switch (tflight->color_status) {
    case 'R':
      current_lane_info->set_color_status(SingleLightInfo::RED);
      break;
    case 'G':
      current_lane_info->set_color_status(SingleLightInfo::GREEN);
      break;
    case 'Y':
      current_lane_info->set_color_status(SingleLightInfo::YELLOW);
      break;
    default:
      current_lane_info->set_color_status(SingleLightInfo::UNKNOWN);
      break;
  }
  current_lane_info->set_light_remain_times(tflight->light_remain_times);

  auto* left_turn_info =
      cidiv2x_.mutable_traffic_light_info()->mutable_left_turn();
  left_turn_info->set_receive_flags(tflight->left_turn_flags);
  switch (tflight->left_color_status) {
    case 'R':
      left_turn_info->set_color_status(SingleLightInfo::RED);
      break;
    case 'G':
      left_turn_info->set_color_status(SingleLightInfo::GREEN);
      break;
    case 'Y':
      left_turn_info->set_color_status(SingleLightInfo::YELLOW);
      break;
    default:
      left_turn_info->set_color_status(SingleLightInfo::UNKNOWN);
      break;
  }
  left_turn_info->set_light_remain_times(tflight->left_remain_times);

  auto* straight_info =
      cidiv2x_.mutable_traffic_light_info()->mutable_straight();
  straight_info->set_receive_flags(tflight->straight_flags);
  switch (tflight->left_color_status) {
    case 'R':
      straight_info->set_color_status(SingleLightInfo::RED);
      break;
    case 'G':
      straight_info->set_color_status(SingleLightInfo::GREEN);
      break;
    case 'Y':
      straight_info->set_color_status(SingleLightInfo::YELLOW);
      break;
    default:
      straight_info->set_color_status(SingleLightInfo::UNKNOWN);
      break;
  }
  straight_info->set_light_remain_times(tflight->straight_remain_times);

  auto* right_turn_info =
      cidiv2x_.mutable_traffic_light_info()->mutable_right_turn();
  right_turn_info->set_receive_flags(tflight->right_turn_flags);
  switch (tflight->right_color_status) {
    case 'R':
      right_turn_info->set_color_status(SingleLightInfo::RED);
      break;
    case 'G':
      right_turn_info->set_color_status(SingleLightInfo::GREEN);
      break;
    case 'Y':
      right_turn_info->set_color_status(SingleLightInfo::YELLOW);
      break;
    default:
      right_turn_info->set_color_status(SingleLightInfo::UNKNOWN);
      break;
  }
  right_turn_info->set_light_remain_times(tflight->right_remain_times);

  auto* lane_stop_point =
      cidiv2x_.mutable_traffic_light_info()->mutable_current_lane_stop_point();
  lane_stop_point->set_latitude(ntohl(tflight->latitude));
  lane_stop_point->set_longitude(ntohl(tflight->longitude));
  return true;
}

bool CidiV2xParser::HandleSignInfo(const v2xmsg::SignInfo* sign) {
  // Todo(inumo): add SignInfo proto
  return true;
}

}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo
