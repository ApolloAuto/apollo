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

#pragma once

namespace apollo {
namespace drivers {
namespace nano_radar {

const int CONTIID_START = 0x600;
const int CONTIID_END = 0x702;
const int WAIT_TIME = 4000;

// Try this many times when receiving using bcan, by default.
const int BCAN_RECV_TRIES = 4;

const int RADAR_CONFIG = 0x200;
const int RADAR_STATE = 0x201;

const int CAN_BUFFER_NUM = 20;
const int CANBYTE = 13;
const int ETHER_WAIT = 5000000;
const int DISCONNECT_WAIT = 5;

// Object general information
const double OBJECT_DIST_RES = 0.2;
const double OBJECT_DIST_LONG_MIN = -500;
const double OBJECT_DIST_LAT_MIN = -204.6;
const double OBJECT_VREL_RES = 0.25;
const double OBJECT_VREL_LONG_MIN = -128.0;
const double OBJECT_VREL_LAT_MIN = -64.0;
const double OBJECT_RCS_RES = 0.5;
const double OBJECT_RCS_MIN = -64.0;

// Object extended information
const double OBJECT_AREL_RES = 0.01;
const double OBJECT_AREL_LONG_MIN = -10.0;
const double OBJECT_AREL_LAT_MIN = -2.5;
const double OBJECT_ORIENTATION_ANGEL_MIN = -180.0;
const double OBJECT_ORIENTATION_ANGEL_RES = 0.4;
const double OBJECT_WIDTH_RES = 0.2;
const double OBJECT_LENGTH_RES = 0.2;

// Region state information
const double REGION_POINT_RES = 0.2;
const double REGION_POINT_LONG_MIN = -500;
const double REGION_POINT_LAT_MIN = -204.6;

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
