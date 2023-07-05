/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <memory>

#include "modules/v2x/proto/v2x_car_status.pb.h"
#include "modules/v2x/proto/v2x_junction.pb.h"
#include "modules/v2x/proto/v2x_obu_rsi.pb.h"
#include "modules/v2x/proto/v2x_obu_traffic_light.pb.h"
#include "modules/v2x/proto/v2x_rsi.pb.h"
#include "modules/common_msgs/v2x_msgs/v2x_traffic_light.pb.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"

namespace apollo {
namespace v2x {
enum RsiAlterType {
  SPEED_LIMIT = 85,
  SPEED_LIMIT_BRIDGE = 8,
  SPEED_LIMIT_TUNNEL = 21,
  CONSTRUCTION_AHEAD = 38,
  BUS_LANE = 123,
  TIDAL_LANE = 41,
  TRAFFIC_JAM = 47,
  TRAFFIC_ACCIDENT = 244,
  NO_HONKING = 80,
  SLOW_DOWN_SECTION = 35,
  ACCIDENT_PRONE = 34,
  OVERSPEED_VEHICLE = 801,
  EMERGENCY_BRAKING = 802,
  ANTIDROMIC_VEHICLE = 803,
  ZOMBIES_VEHICLE = 804,
  CONTROLLOSS_VEHICLE = 1000,
  SPECIAL_VEHICLE = 2000,
};

using OSLightColor = ::apollo::v2x::SingleTrafficLight_Color;
using OSLightype = ::apollo::v2x::SingleTrafficLight_Type;
using ObuLightype = ::apollo::v2x::obu::SingleTrafficLight_Type;
using OSLight = ::apollo::v2x::IntersectionTrafficLightData;
using ObuLight = ::apollo::v2x::obu::ObuTrafficLight;
using OSLocation = ::apollo::localization::LocalizationEstimate;
using OSRsi = ::apollo::v2x::RsiMsg;
using ObuRsi = ::apollo::v2x::obu::ObuRsi;
using HDJunction = ::apollo::hdmap::JunctionInfoConstPtr;
using ObuJunction = ::apollo::v2x::Junction;
class ProtoAdapter final {
 public:
  static OSLightype LightTypeObu2Sys(int32_t type);
  static bool LightObu2Sys(const ObuLight &obu_light,
                           std::shared_ptr<OSLight> *os_light);
  static bool RsiObu2Sys(const ObuRsi *obu_rsi, std::shared_ptr<OSRsi> *os_rsi);
  static bool JunctionHd2obu(const HDJunction &hd_junction,
                             std::shared_ptr<ObuJunction> *obu_junction);
};

}  // namespace v2x
}  // namespace apollo
