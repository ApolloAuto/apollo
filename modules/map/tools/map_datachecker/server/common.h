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
#pragma once

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <ctime>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "cyber/cyber.h"

namespace apollo {
namespace hdmap {

constexpr double kRADIANS_TO_DEGREES = 180.0 / M_PI;
constexpr double kDEGRESS_TO_RADIANS = M_PI / 180.0;

typedef unsigned char uchar;

struct FramePose {
  double time_stamp;  // unix time
  double tx, ty, tz;
  double qx, qy, qz, qw;
  double latitude, longitude, altitude;
  double velx, vely, velz;
  double roll, pitch, azimuth;
  unsigned int ins_status;
  unsigned int solution_status;
  unsigned int position_type;
  float diff_age;
  double local_std;
};

enum class State { IDLE, RUNNING };

struct JSonConf {
  std::vector<std::pair<std::string, double>> topic_list;
  bool use_system_time;
  double topic_rate_tolerance;

  unsigned int solution_status;
  std::set<unsigned int> position_type_range;
  std::pair<float, float> diff_age_range;
  double local_std_upper_limit;

  /*Period of channel check trigger. The unit is seconds.*/
  int channel_check_trigger_gap;
  /*Period of alignment, The unit is seconds.
  Eight route maneuver is also included in the alignment process.*/
  int alignment_featch_pose_sleep;
  /*the time that static alignment must last. The unit is seconds.*/
  double static_align_duration;
  /*the maximum time that bad pose included in static alignment.
  The unit is seconds.*/
  double static_align_tolerance;
  /*Maximum Motion Distance Tolerated by Static Alignment*/
  double static_align_dist_thresh;

  /*Angle threshold between adjacent frames in eight route. The unit is degree*/
  double eight_angle;
  /*the time that eight route must last，The unit is seconds.*/
  double eight_duration;
  /*Minimum speed should be achieved in eight route*/
  double eight_vel;
  /*The tolerance of bad pose in eight route. The unit is frame*/
  int eight_bad_pose_tolerance;

  /*Minimum frame number threshold for acquisition of multiple loops*/
  int laps_frames_thresh;
  /*The angle error of adjacent poses on the same trajectory should not be
  greater than alpha_err_thresh. The unit is degree*/
  double laps_alpha_err_thresh;
  /*The time stamp interval of adjacent poses on the same track should not be
  greater than time_err_thresh. The unit is minutes.*/
  double laps_time_err_thresh;
  /*The diameter of the searched square area*/
  int laps_search_diameter;
  /*loops to check*/
  size_t laps_number;
  /*additional loops to check*/
  int laps_number_additional;
  /*Proportional thresholds for all required points
  in acquisition cycle checking*/
  double laps_rate_thresh;
};

std::shared_ptr<JSonConf> ParseJson(std::string conf_path);

inline double GetYaw(double from_x, double from_y, double to_x, double to_y) {
  double vecx = to_x - from_x;
  double vecy = to_y - from_y;
  double alpha = acos(vecy / sqrt(vecx * vecx + vecy * vecy));
  if (vecx < 0) {
    alpha = 2 * M_PI - alpha;
  }
  return kRADIANS_TO_DEGREES * alpha;
}

inline double UnixtimeNow() { return apollo::cyber::Time::Now().ToSecond(); }

}  // namespace hdmap
}  // namespace apollo
