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
#ifndef _MODULES_MAP_TOOLS_MAP_DATACHECKER_COMMON_HPP
#define _MODULES_MAP_TOOLS_MAP_DATACHECKER_COMMON_HPP
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <vector>
#include <set>
#include <string>
#include <utility>
#include <cmath>
#include <ctime>
#include <memory>
#include <iostream>
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

enum class State {
  IDLE,
  RUNNING
};


struct JSonConf {
  std::vector<std::pair<std::string, double>> topic_list;
  bool use_system_time;
  double topic_rate_tolerance;

  unsigned int solution_status;
  std::set<unsigned int> position_type_range;
  std::pair<float, float> diff_age_range;
  double local_std_upper_limit;
  // double local_err;

  /*channel检查触发周期，单位是秒*/
  int channel_check_trigger_gap;
  /*对准检查周期，单位是秒。绕八字也归入对准过程*/
  int alignment_featch_pose_sleep;
  /*static alignment 对准必须持续的时间，单位是秒*/
  double static_align_duration;
  /*static alignment 容忍的连续bad pose持续时间，单位是秒*/
  double static_align_tolerance;
  /**/
  double static_align_dist_thresh;

  /*转弯最小角度阈值，单位是度。超过该阈值认为车辆在转弯*/
  double dynamic_align_turn_angle;
  /*动态对齐的角度偏差阈值，单位是度*/
  double dynamic_align_straight_angle;
  /*动态对准必须达到的最低速度，单位是m/s*/
  double dynamic_align_vel;
  /*动态对准必须持续的时间，单位是秒*/
  double dynamic_align_duration;
  /*动态对准错误pose容忍度，单位是帧*/
  int dynamic_align_bad_pose_tolerance;
  /*动态对准允许分段数*/
  int dynamic_align_segment;
  /*动态对准每段最小长度，单位是米*/
  double dynamic_align_dist_per_segment;


  /*绕八字解算相邻帧之间的角度阈值，单位是度*/
  double eight_angle;
  /*绕八字的最短时间要求，单位是秒*/
  double eight_duration;
  /*绕八字时应达到的最低速度*/
  double eight_vel;
  /*绕八字错误pose容忍度，单位是帧*/
  int eight_bad_pose_tolerance;


  /*采集多圈的最低帧数阈值*/
  int laps_frames_thresh;
  /*同一条轨迹上相邻帧pose的角度误差不应该大于alpha_err_thresh，单位是度*/
  double laps_alpha_err_thresh;
  /*同一条轨迹上相邻帧pose的时间戳间隔不应该大于time_err_thresh，单位是分钟*/
  double laps_time_err_thresh;
  /*圈数检查时搜索的正方形区域的直径*/
  int laps_search_diameter;
  /*要求采集的圈数*/
  size_t laps_number;
  /*校验圈数*/
  int laps_number_additional;
  /*圈数检查中所有符合要求的点的比例阈值*/
  double laps_rate_thresh;
  /*采样频率*/
  int laps_inspva_downsample_freq;
};

inline std::shared_ptr<JSonConf> parse_json(std::string conf_path) {
  std::cerr << "parsing json config" << std::endl;
  boost::filesystem::path path(conf_path);
  if (!boost::filesystem::exists(path)) {
    std::cerr << "json config file "
          << conf_path << " is not exist" << std::endl;
    return nullptr;
  }

  std::shared_ptr<JSonConf> conf(new JSonConf);
  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_json(conf_path, pt);
    conf->use_system_time = pt.get<bool>("use_system_time");
    conf->topic_rate_tolerance = pt.get<double>("topic_rate_tolerance");
    boost::property_tree::ptree children2 = pt.get_child("topic_list");
    for (auto it = children2.begin(); it != children2.end(); it++) {
      conf->topic_list.push_back(
        std::make_pair(it->first, it->second.get_value<double>()));
    }

    conf->solution_status = pt.get<unsigned int>("solution_status");

    boost::property_tree::ptree
      position_type = pt.get_child("position_type");
    for (auto it = position_type.begin(); it != position_type.end(); it++) {
      conf->position_type_range.insert(
        it->second.get_value<unsigned int>());
    }

    conf->local_std_upper_limit = pt.get<double>("local_std_upper_limit");

    boost::property_tree::ptree diff_age = pt.get_child("diff_age");
    {
      auto it = diff_age.begin();
      conf->diff_age_range.first = it->second.get_value<float>();
      it++;
      conf->diff_age_range.second = it->second.get_value<float>();
    }

    conf->channel_check_trigger_gap
      = pt.get<int>("channel_check_trigger_gap");
    conf->alignment_featch_pose_sleep
      = pt.get<int>("alignment_featch_pose_sleep");
    conf->static_align_duration
      = pt.get<double>("static_align_duration");
    conf->static_align_tolerance
      = pt.get<double>("static_align_tolerance");
    conf->static_align_dist_thresh
      = pt.get<double>("static_align_dist_thresh");

    conf->dynamic_align_turn_angle
      = pt.get<double>("dynamic_align_turn_angle");
    conf->dynamic_align_straight_angle
      = pt.get<double>("dynamic_align_straight_angle");
    conf->dynamic_align_vel
      = pt.get<double>("dynamic_align_vel");
    conf->dynamic_align_duration
      = pt.get<double>("dynamic_align_duration");
    conf->dynamic_align_bad_pose_tolerance
      = pt.get<int>("dynamic_align_bad_pose_tolerance");
    conf->dynamic_align_segment
      = pt.get<int>("dynamic_align_segment");
    conf->dynamic_align_dist_per_segment
      = pt.get<double>("dynamic_align_dist_per_segment");

    conf->eight_angle
      = pt.get<double>("eight_angle");
    conf->eight_duration
      = pt.get<double>("eight_duration");
    conf->eight_vel
      = pt.get<double>("eight_vel");
    conf->eight_bad_pose_tolerance
      = pt.get<int>("eight_bad_pose_tolerance");

    conf->laps_frames_thresh
      = pt.get<int>("laps_frames_thresh");
    conf->laps_alpha_err_thresh
      = pt.get<double>("laps_alpha_err_thresh");
    conf->laps_time_err_thresh
      = pt.get<double>("laps_time_err_thresh");
    conf->laps_search_diameter
      = pt.get<int>("laps_search_diameter");
    conf->laps_number
      = pt.get<int>("laps_number");
    conf->laps_number_additional
      = pt.get<int>("laps_number_additional");
    conf->laps_rate_thresh
      = pt.get<double>("laps_rate_thresh");
    conf->laps_inspva_downsample_freq
      = pt.get<int>("laps_inspva_downsample_freq");
  }
  catch(const boost::property_tree::json_parser_error& e) {
    std::cerr << e.what() << '\n';
  }
  catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  return conf;
}


inline double get_yaw(double from_x, double from_y, double to_x, double to_y) {
  double vecx = to_x - from_x;
  double vecy = to_y - from_y;

  double alpha = acos(vecy / sqrt(vecx * vecx + vecy * vecy));
  if ( vecx < 0 ) {
    alpha = 2 * M_PI - alpha;
  }
  return RADIANS_TO_DEGREES * alpha;
}

inline double unixtime_now() {
  return apollo::cyber::Time::Now().ToSecond();
}

}  // namespace hdmap
}  // namespace apollo

#endif  // _MODULES_MAP_TOOLS_MAP_DATACHECKER_COMMON_HPP
