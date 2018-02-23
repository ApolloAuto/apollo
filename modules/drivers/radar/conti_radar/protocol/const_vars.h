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

#ifndef MODULES_DRIVERS_RADAR_CONTI_RADAR_CONST_VARS_H_
#define MODULES_DRIVERS_RADAR_CONTI_RADAR_CONST_VARS_H_

namespace apollo {
namespace drivers {
namespace conti_radar {

const int CONTIID_START = 0x600;
const int CONTIID_END = 0x702;
const int WAIT_TIME = 4000;

// Try this many times when reciving using bcan, by default.
const int BCAN_RECV_TRIES = 4;

const int RADAR_CONFIG = 0x200;
const int RADAR_STATE = 0x201;

const int CAN_BUFFER_NUM = 20;
const int CANBYTE = 13;
const int ETHER_WAIT = 5000000;
const int DISCONNECT_WAIT = 5;

// cluster quality

const double LINEAR_RMS[32] = {0.005, 0.006, 0.008,  0.011, 0.014, 0.018, 0.023,
                               0.029, 0.038, 0.049,  0.063, 0.081, 0.105, 0.135,
                               0.174, 0.224, 0.288,  0.371, 0.478, 0.616, 0.794,
                               1.023, 1.317, 1.697,  2.187, 2.817, 3.630, 4.676,
                               6.025, 7.762, 10.000, 100.00};

const double ANGLE_RMS[32] = {
    0.005,  0.007,  0.010,  0.014,  0.020,  0.029,   0.041,   0.058,
    0.082,  0.116,  0.165,  0.234,  0.332,  0.471,   0.669,   0.949,
    1.346,  1.909,  2.709,  3.843,  5.451,  7.734,   10.971,  15.565,
    22.061, 31.325, 44.439, 63.044, 69.437, 126.881, 180.000, 360.00};

const double PROBOFEXIST[8] = {0.00, 0.25, 0.5, 0.75, 0.90, 0.99, 0.999, 1.0};

const double CLUSTER_DIST_RES = 0.2;
const double CLUSTER_DIST_LONG_MIN = -500;
const double CLUSTER_DIST_LAT_MIN = -102.3;
const double CLUSTER_VREL_RES = 0.25;
const double CLUSTER_VREL_LONG_MIN = -128.0;
const double CLUSTER_VREL_LAT_MIN = -64.0;
const double CLUSTER_RCS_RES = 0.5;
const double CLUSTER_RCS = -64.0;

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

}  // namespace conti_radar
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_RADAR_CONTI_RADAR_CONST_VARS_H_
