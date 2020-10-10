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
#include "modules/map/tools/map_datachecker/server/common.h"

#include "cyber/cyber.h"

namespace apollo {
namespace hdmap {

std::shared_ptr<JsonConf> ParseJson(std::string conf_path) {
  AINFO << "parsing json config";
  boost::filesystem::path path(conf_path);
  if (!boost::filesystem::exists(path)) {
    AERROR << "json config file " << conf_path << " does not exist";
    return nullptr;
  }

  std::shared_ptr<JsonConf> conf(new JsonConf);
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

}  // namespace hdmap
}  // namespace apollo
