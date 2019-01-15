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

#pragma once

#include <stdint.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cyber/cyber.h"

#include "modules/drivers/cidiv2x/proto/cidiv2x.pb.h"
#include "modules/drivers/cidiv2x/stream/proto/config.pb.h"
#include "modules/drivers/cidiv2x/stream/util/macros.h"

namespace apollo {
namespace drivers {
namespace cidiv2x {
namespace v2xmsg {

#pragma pack(push, 1)  // Turn off struct padding.

enum SyncByte : uint8_t {
  SYNC_0 = 0x55,
  SYNC_1 = 0x56,
  SYNC_2 = 0x57,
  SYNC_3 = 0x58,
};

struct MessageType {
  enum MessageFormat {
    BINARY = 0b00,
    ASCII = 0b01,
    ABREVIATED_ASCII = 0b10,
    NMEA = 0b11,
  };

  enum ResponseBit {
    ORIGINAL_MESSAGE = 0b0,
    RESPONSE_MESSAGE = 0b1,
  };

  uint8_t reserved : 5;
  MessageFormat format : 2;
  ResponseBit response : 1;
};

enum LocalizationType : uint8_t {
  INVALID = 0,  // Invalid solution due to insufficient observations,
                // integrity warning, etc.
  PROPAGATED,   // Propagated by a Kalman filter without new observations.

  // It is recommended using the following types of solution.
  SINGLE,       // Standard GNSS solution without any corrections.
  PSRDIFF,      // Pseudorange differential solution, including WAAS/SBAS
                // solution.
  PPP,          // Precise Point Positioning (PPP) solution.
  RTK_FLOAT,    // Real Time Kinematic (RTK) float solution.
  RTK_INTEGER,  // RTK integer solution.
};

struct CurrentCarInfo {
  uint32_t message_cycle_count;
  uint32_t car_id;
  uint16_t car_length;
  uint16_t car_width;
  LocalizationType localization_type;  // real uint32_t
  uint8_t num_statellites;
  uint32_t latitude;
  uint32_t longitude;
  uint32_t height_msl;
  uint16_t heading;
  uint16_t gps_velocity;
  uint16_t gps_velocity1;
  uint16_t gps_velocity2;
  uint16_t gps_acceleration;
  uint16_t gps_acceleration1;
  uint16_t gps_acceleration2;
};
static_assert(sizeof(CurrentCarInfo) == 40, "Incorrect size of CurrentCarInfo");

struct LaneInfo {
  uint32_t lane_cycle_count;
  uint8_t in_map;
  uint8_t in_lane;
  uint8_t total_lane;
  uint8_t current_lane_id;
  uint8_t lane_flags[8];
};
static_assert(sizeof(LaneInfo) == 16, "Incorrect size of LaneInfo");

enum Color : uint8_t {
  UNKNOWN = 0,
  RED = 'R',
  YELLOW = 'Y',
  GREEN = 'G',
  BLACK = 'B',
};

struct TrafficLightInfo {
  uint32_t traffic_light_cycle_count;
  uint8_t receive_flags;
  Color color_status;
  uint8_t light_remain_times;
  uint8_t left_turn_flags;
  Color left_color_status;
  uint8_t left_remain_times;
  uint8_t straight_flags;
  Color straight_color_status;
  uint8_t straight_remain_times;
  uint8_t right_turn_flags;
  Color right_color_status;
  uint8_t right_remain_times;
  uint32_t latitude;
  uint32_t longitude;
};
static_assert(sizeof(TrafficLightInfo) == 24,
              "Incorrect size of TrafficLightInfo");

enum SignType : uint8_t {
  NO_PARKING = 1,
  PERDESTRIAN_LANE,
  RESET_AREA,
  NEAR_SCHOOL,
  PARKING,
  SPEED_LIMIT,
  NO_ENTRANCE,
  NO_BEAMS,
  WEIHT_LIMIT,
  BUS_STATION,
};

struct SignDetails {
  SignType sign_type;
  uint8_t sign_value;
  uint32_t sign_latitude;
  uint32_t sign_longitude;
  uint16_t sing_reversed;
};
static_assert(sizeof(SignDetails) == 12, "Incorrect size of SignDetails");

struct SignInfo {
  uint32_t sign_cycle_time;
  uint8_t sign_num;
  SignDetails sign_details[10];
};
static_assert(sizeof(SignInfo) == 125, "Incorrect size of SignInfo");

}  // namespace v2xmsg

class CidiV2xParser {
 public:
  CidiV2xParser();
  explicit CidiV2xParser(const config::Config& config,
                         const std::shared_ptr<apollo::cyber::Node>& node);

  bool Init();

  bool GetMessage();

  void ClearData() { cidiv2x_.Clear(); }

  CiDiV2X& GetData(void) { return cidiv2x_; }

  bool ParseRawData(const std::string& msg);

  void Update(const uint8_t* data, size_t length) {
    data_ = data;
    data_end_ = data + length;
  }
  void Update(const std::string& data) {
    Update(reinterpret_cast<const uint8_t*>(data.data()), data.size());
  }

 private:
  bool PrepareMessage();

  // The handle_xxx functions return whether a message is ready.
  bool HandleCurrentCarInfo(const v2xmsg::CurrentCarInfo* car);
  bool HandleLaneInfo(const v2xmsg::LaneInfo* lane);
  bool HandleTrafficLightInfo(const v2xmsg::TrafficLightInfo* tflight);
  bool HandleSignInfo(const v2xmsg::SignInfo* sign);

  std::vector<uint8_t> buffer_;

  bool is_inited_ = false;

  size_t total_length_ = 0;

  const uint8_t* data_ = nullptr;
  const uint8_t* data_end_ = nullptr;

  std::mutex cidiv2x_mutex_;
  CiDiV2X cidiv2x_;

  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<CiDiV2X>> cidiv2x_writer_ = nullptr;
};

}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo
