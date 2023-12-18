/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include "ad_rss/core/RssResponseResolving.hpp"
#include "ad_rss/core/RssResponseTransformation.hpp"
#include "ad_rss/core/RssSituationChecking.hpp"
#include "ad_rss/core/RssSituationExtraction.hpp"
#include "ad_rss/situation/SituationVector.hpp"
#include "ad_rss/state/ResponseState.hpp"
#include "ad_rss/state/ResponseStateVector.hpp"
#include "situation/RSSFormulas.hpp"
#include "modules/planning/tasks/rss_decider/proto/rss_decider.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/task.h"

namespace apollo {
namespace planning {

struct rss_world_model_struct {
  std::string err_code;
  double front_obs_dist;
  double obs_s_start;
  double obs_s_end;
  double obs_l_start;
  double obs_l_end;
  double obs_speed;
  double ego_v_s_start;
  double ego_v_s_end;
  double ego_v_l_start;
  double ego_v_l_end;
  double lane_leftmost;
  double lane_rightmost;
  double lane_length;
  double lane_width;
  double OR_front_lon_min;
  double OR_front_lon_max;
  double OR_front_lat_min;
  double OR_front_lat_max;
  double OR_rear_lon_min;
  double OR_rear_lon_max;
  double OR_rear_lat_min;
  double OR_rear_lat_max;
  double adc_vel;
  double laneSeg_len_min;
  double laneSeg_len_max;
  double laneSeg_width_min;
  double laneSeg_width_max;
};

class RssDecider : public Task {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<DependencyInjector> &injector) override;

  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(Frame *frame,
                                 ReferenceLineInfo *reference_line_info);
  struct rss_world_model_struct rss_world_info;
  void rss_config_default_dynamics(::ad_rss::world::Dynamics *dynamics);
  void rss_create_ego_object(::ad_rss::world::Object *ego, double vel_lon,
                             double vel_lat);
  void rss_create_other_object(::ad_rss::world::Object *other, double vel_lon,
                               double vel_lat);
  void rss_dump_world_info(const struct rss_world_model_struct &rss_info);

  RssDeciderConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RssDecider, Task)

}  // namespace planning
}  // namespace apollo
