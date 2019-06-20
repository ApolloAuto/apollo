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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_SERVER_COMMON_HPP
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_SERVER_COMMON_HPP
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

constexpr double RADIANS_TO_DEGREES = 180.0 / M_PI;
constexpr double DEGRESS_TO_RADIANS = M_PI / 180.0;
constexpr double EARTH_RADIUS = 6378137;
constexpr double sm_a = 6378137.0;
constexpr double sm_b = 6356752.31425;
constexpr double sm_EccSquared = 6.69437999013e-03;
constexpr double UTMScaleFactor = 0.9996;

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
  /*addtional loops to check*/
  int laps_number_additional;
  /*Proportional thresholds for all required points
  in acquisition cycle checking*/
  double laps_rate_thresh;
};

inline std::shared_ptr<JSonConf> parse_json(std::string conf_path) {
  AINFO << "parsing json config";
  boost::filesystem::path path(conf_path);
  if (!boost::filesystem::exists(path)) {
    AERROR << "json config file " << conf_path << " is not exist";
    return nullptr;
  }

  std::shared_ptr<JSonConf> conf(new JSonConf);
  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_json(conf_path, pt);
    conf->use_system_time = pt.get<bool>("use_system_time");
    conf->topic_rate_tolerance = pt.get<double>("topic_rate_tolerance");
    boost::property_tree::ptree children2 = pt.get_child("topic_list");
    for (auto it = children2.begin(); it != children2.end(); ++it) {
      conf->topic_list.push_back(
          std::make_pair(it->first, it->second.get_value<double>()));
    }

    conf->solution_status = pt.get<unsigned int>("solution_status");

    boost::property_tree::ptree position_type = pt.get_child("position_type");
    for (auto it = position_type.begin(); it != position_type.end(); ++it) {
      conf->position_type_range.insert(it->second.get_value<unsigned int>());
    }

    conf->local_std_upper_limit = pt.get<double>("local_std_upper_limit");

    boost::property_tree::ptree diff_age = pt.get_child("diff_age");
    {
      auto it = diff_age.begin();
      conf->diff_age_range.first = it->second.get_value<float>();
      ++it;
      conf->diff_age_range.second = it->second.get_value<float>();
    }

    conf->channel_check_trigger_gap = pt.get<int>("channel_check_trigger_gap");
    conf->alignment_featch_pose_sleep =
        pt.get<int>("alignment_featch_pose_sleep");
    conf->static_align_duration = pt.get<double>("static_align_duration");
    conf->static_align_tolerance = pt.get<double>("static_align_tolerance");
    conf->static_align_dist_thresh = pt.get<double>("static_align_dist_thresh");

    conf->eight_angle = pt.get<double>("eight_angle");
    conf->eight_duration = pt.get<double>("eight_duration");
    conf->eight_vel = pt.get<double>("eight_vel");
    conf->eight_bad_pose_tolerance = pt.get<int>("eight_bad_pose_tolerance");

    conf->laps_frames_thresh = pt.get<int>("laps_frames_thresh");
    conf->laps_alpha_err_thresh = pt.get<double>("laps_alpha_err_thresh");
    conf->laps_time_err_thresh = pt.get<double>("laps_time_err_thresh");
    conf->laps_search_diameter = pt.get<int>("laps_search_diameter");
    conf->laps_number = pt.get<int>("laps_number");
    conf->laps_number_additional = pt.get<int>("laps_number_additional");
    conf->laps_rate_thresh = pt.get<double>("laps_rate_thresh");
  } catch (const boost::property_tree::json_parser_error& e) {
    AERROR << e.what();
    return nullptr;
  } catch (const std::exception& e) {
    AERROR << e.what();
    return nullptr;
  }
  return conf;
}

inline double get_yaw(double from_x, double from_y, double to_x, double to_y) {
  double vecx = to_x - from_x;
  double vecy = to_y - from_y;
  double alpha = acos(vecy / sqrt(vecx * vecx + vecy * vecy));
  if (vecx < 0) {
    alpha = 2 * M_PI - alpha;
  }
  return RADIANS_TO_DEGREES * alpha;
}

inline double unixtime_now() { return apollo::cyber::Time::Now().ToSecond(); }

}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_SERVER_COMMON_HPP
