/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light_tracking/tracker/semantic_decision.h"

#include <map>

#include "boost/bind.hpp"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace trafficlight {

std::map<base::TLColor, std::string> s_color_strs = {
    {base::TLColor::TL_UNKNOWN_COLOR, "unknown"},
    {base::TLColor::TL_RED, "red"},
    {base::TLColor::TL_GREEN, "green"},
    {base::TLColor::TL_YELLOW, "yellow"},
    {base::TLColor::TL_BLACK, "black"}};

bool compare(const SemanticTable &s1, const SemanticTable &s2) {
  return s1.semantic == s2.semantic;
}

SemanticReviser::SemanticReviser()
    : revise_time_s_(1.5f),
      blink_threshold_s_(0.4f),
      non_blink_threshold_s_(0.8f),
      hysteretic_threshold_(1) {}

bool SemanticReviser::Init(const TrafficLightTrackerInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  if (!cyber::common::GetProtoFromFile(config_file, &semantic_param_)) {
    AERROR << "load proto param failed, root dir: " << config_file;
    return false;
  }

  int non_blink_coef = 2;
  revise_time_s_ = semantic_param_.revise_time_second();
  blink_threshold_s_ = semantic_param_.blink_threshold_second();
  hysteretic_threshold_ = semantic_param_.hysteretic_threshold_count();
  non_blink_threshold_s_ =
      blink_threshold_s_ * static_cast<float>(non_blink_coef);

  ADEBUG << "revise_time_s_: " << revise_time_s_;
  ADEBUG << "blink_threshold_s_: " << blink_threshold_s_;
  ADEBUG << "hysteretic_threshold_: " << hysteretic_threshold_;

  return true;
}

void SemanticReviser::UpdateHistoryAndLights(
    const SemanticTable &cur, std::vector<base::TrafficLightPtr> *lights,
    std::vector<SemanticTable>::iterator *history) {
  (*history)->time_stamp = cur.time_stamp;
  if ((*history)->color == base::TLColor::TL_BLACK) {
    if ((*history)->hystertic_window.hysteretic_color == cur.color) {
      (*history)->hystertic_window.hysteretic_count++;
    } else {
      (*history)->hystertic_window.hysteretic_color = cur.color;
      (*history)->hystertic_window.hysteretic_count = 1;
    }
    ADEBUG << "Black lights hysteretic change to " << s_color_strs[cur.color]
           << " count " << (*history)->hystertic_window.hysteretic_count
           << " threshold " << hysteretic_threshold_;

    if ((*history)->hystertic_window.hysteretic_count > hysteretic_threshold_) {
      (*history)->color = cur.color;
      (*history)->hystertic_window.hysteretic_count = 0;
      ADEBUG << "Black lights hysteretic change to " << s_color_strs[cur.color];
    } else {
      ReviseLights(lights, cur.light_ids, (*history)->color);
    }
  } else {
    (*history)->color = cur.color;
  }
}

base::TLColor SemanticReviser::ReviseBySemantic(
    SemanticTable semantic_table, std::vector<base::TrafficLightPtr> *lights) {
  std::vector<int> vote(static_cast<int>(base::TLColor::TL_TOTAL_COLOR_NUM), 0);
  std::vector<base::TrafficLightPtr> &lights_ref = *lights;
  base::TLColor max_color = base::TLColor::TL_UNKNOWN_COLOR;

  for (size_t i = 0; i < semantic_table.light_ids.size(); ++i) {
    int index = semantic_table.light_ids.at(i);
    base::TrafficLightPtr light = lights_ref[index];
    auto color = light->status.color;
    vote.at(static_cast<int>(color))++;
  }

  if ((vote.at(static_cast<size_t>(base::TLColor::TL_RED)) == 0) &&
      (vote.at(static_cast<size_t>(base::TLColor::TL_GREEN)) == 0) &&
      (vote.at(static_cast<size_t>(base::TLColor::TL_YELLOW)) == 0)) {
    if (vote.at(static_cast<size_t>(base::TLColor::TL_BLACK)) > 0) {
      return base::TLColor::TL_BLACK;
    } else {
      return base::TLColor::TL_UNKNOWN_COLOR;
    }
  }

  vote.at(static_cast<size_t>(base::TLColor::TL_BLACK)) = 0;
  vote.at(static_cast<size_t>(base::TLColor::TL_UNKNOWN_COLOR)) = 0;

  auto biggest = std::max_element(std::begin(vote), std::end(vote));

  int max_color_num = *biggest;
  max_color = base::TLColor(std::distance(std::begin(vote), biggest));

  vote.erase(biggest);

  auto second_biggest = std::max_element(std::begin(vote), std::end(vote));

  ADEBUG << "color " << s_color_strs[max_color] << " is max " << max_color_num;

  if (max_color_num == *second_biggest) {
    return base::TLColor::TL_UNKNOWN_COLOR;
  } else {
    return max_color;
  }
}

void SemanticReviser::ReviseLights(std::vector<base::TrafficLightPtr> *lights,
                                   const std::vector<int> &light_ids,
                                   base::TLColor dst_color) {
  for (auto index : light_ids) {
    lights->at(index)->status.color = dst_color;
  }

  ADEBUG << "revise " << light_ids.size() << " lights to "
         << s_color_strs[dst_color];
}

void SemanticReviser::ReviseByTimeSeries(
    double time_stamp, SemanticTable semantic_table,
    std::vector<base::TrafficLightPtr> *lights) {
  ADEBUG << "revise " << semantic_table.semantic
         << ", lights number:" << semantic_table.light_ids.size();

  std::vector<base::TrafficLightPtr> &lights_ref = *lights;
  base::TLColor cur_color = ReviseBySemantic(semantic_table, lights);
  base::TLColor pre_color = base::TLColor::TL_UNKNOWN_COLOR;
  semantic_table.color = cur_color;
  semantic_table.time_stamp = time_stamp;
  ADEBUG << "revise same semantic lights";
  ReviseLights(lights, semantic_table.light_ids, cur_color);

  std::vector<SemanticTable>::iterator iter =
      std::find_if(std::begin(history_semantic_), std::end(history_semantic_),
                   boost::bind(compare, _1, semantic_table));

  if (iter != history_semantic_.end()) {
    pre_color = iter->color;
    if (time_stamp - iter->time_stamp < revise_time_s_) {
      ADEBUG << "revise by time series";
      switch (cur_color) {
        case base::TLColor::TL_YELLOW:
          if (iter->color == base::TLColor::TL_RED) {
            ReviseLights(lights, semantic_table.light_ids, iter->color);
            iter->time_stamp = time_stamp;
            iter->hystertic_window.hysteretic_count = 0;
          } else {
            UpdateHistoryAndLights(semantic_table, lights, &iter);
            ADEBUG << "High confidence color " << s_color_strs[cur_color];
          }
          break;
        case base::TLColor::TL_RED:
        case base::TLColor::TL_GREEN:
          UpdateHistoryAndLights(semantic_table, lights, &iter);
          if (time_stamp - iter->last_bright_time_stamp > blink_threshold_s_ &&
              iter->last_dark_time_stamp > iter->last_bright_time_stamp) {
            iter->blink = true;
          }
          iter->last_bright_time_stamp = time_stamp;
          ADEBUG << "High confidence color " << s_color_strs[cur_color];
          break;
        case base::TLColor::TL_BLACK:
          iter->last_dark_time_stamp = time_stamp;
          iter->hystertic_window.hysteretic_count = 0;
          if (iter->color == base::TLColor::TL_UNKNOWN_COLOR ||
              iter->color == base::TLColor::TL_BLACK) {
            iter->time_stamp = time_stamp;
            UpdateHistoryAndLights(semantic_table, lights, &iter);
          } else {
            ReviseLights(lights, semantic_table.light_ids, iter->color);
          }
          break;
        case base::TLColor::TL_UNKNOWN_COLOR:
        default:
          ReviseLights(lights, semantic_table.light_ids, iter->color);
          break;
      }
    } else {
      iter->time_stamp = time_stamp;
      iter->color = cur_color;
    }

    // set blink status
    if (pre_color != iter->color ||
        fabs(iter->last_dark_time_stamp - iter->last_bright_time_stamp) >
            non_blink_threshold_s_) {
      iter->blink = false;
    }

    for (auto index : semantic_table.light_ids) {
      lights_ref[index]->status.blink =
          (iter->blink && iter->color == base::TLColor::TL_GREEN);
    }
    ADEBUG << "semantic " << semantic_table.semantic << " color "
           << s_color_strs[iter->color] << " blink " << iter->blink << " cur "
           << s_color_strs[cur_color];
    ADEBUG << "cur ts " << time_stamp;
    ADEBUG << "bri ts " << iter->last_bright_time_stamp;
    ADEBUG << "dar ts " << iter->last_dark_time_stamp;
  } else {
    semantic_table.last_dark_time_stamp = semantic_table.time_stamp;
    semantic_table.last_bright_time_stamp = semantic_table.time_stamp;
    history_semantic_.push_back(semantic_table);
  }
}

bool SemanticReviser::Track(camera::TrafficLightFrame *frame) {
  double time_stamp = frame->timestamp;
  std::vector<base::TrafficLightPtr> &lights_ref = frame->traffic_lights;
  std::vector<SemanticTable> semantic_table;
  ADEBUG << "start revise ";

  if (lights_ref.empty()) {
    history_semantic_.clear();
    ADEBUG << "no lights to revise, return";
    return true;
  }

  ADEBUG << "lights_ref size is  : " << lights_ref.size();
  for (size_t i = 0; i < lights_ref.size(); i++) {
    base::TrafficLightPtr light = lights_ref.at(i);
    int cur_semantic = light->semantic;
    ADEBUG << "light " << light->id << " semantic " << cur_semantic;

    SemanticTable tmp;
    std::stringstream ss;

    if (cur_semantic > 0) {
      ss << "Semantic_" << cur_semantic;
    } else {
      ss << "No_semantic_light_" << light->id;
    }

    tmp.semantic = ss.str();
    tmp.light_ids.push_back(static_cast<int>(i));
    tmp.color = light->status.color;
    tmp.time_stamp = time_stamp;
    tmp.blink = false;
    auto iter =
        std::find_if(std::begin(semantic_table), std::end(semantic_table),
                     boost::bind(compare, _1, tmp));

    if (iter != semantic_table.end()) {
      iter->light_ids.push_back(static_cast<int>(i));
    } else {
      semantic_table.push_back(tmp);
    }
  }

  ADEBUG << "semantic_table size is : " << semantic_table.size();
  for (size_t i = 0; i < semantic_table.size(); ++i) {
    SemanticTable cur_semantic_table = semantic_table.at(i);
    ReviseByTimeSeries(time_stamp, cur_semantic_table, &lights_ref);
  }

  return true;
}

REGISTER_TRAFFIC_LIGHT_TRACKER(SemanticReviser);

}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
