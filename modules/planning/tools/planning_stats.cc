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

#include <chrono>
#include <thread>

#include "gflags/gflags.h"
#include "ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_stats.pb.h"

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::planning::ADCTrajectory;
using apollo::planning::PlanningStats;
using apollo::planning::StatsGroup;

PlanningStats g_stats;

void CollectStatsGroup(bool has_val, double val, StatsGroup* stats_group) {
  if (has_val) {
    stats_group->set_num(stats_group->num() + 1);
    stats_group->set_sum(stats_group->sum() + val);
    stats_group->set_max(std::max(stats_group->max(), val));
    stats_group->set_min(std::min(stats_group->min(), val));
    stats_group->set_avg(stats_group->sum() / stats_group->num());
  }
}

void OnPlanning(const ADCTrajectory& trajectory) {
  CollectStatsGroup(trajectory.has_total_path_length(),
                    trajectory.total_path_length(),
                    g_stats.mutable_total_path_length());
  CollectStatsGroup(trajectory.has_total_path_time(),
                    trajectory.total_path_time(),
                    g_stats.mutable_total_path_time());
  for (const auto& tp : trajectory.trajectory_point()) {
    CollectStatsGroup(tp.has_v(), tp.v(), g_stats.mutable_v());
    CollectStatsGroup(tp.has_a(), tp.a(), g_stats.mutable_a());
    const auto& p = tp.path_point();
    CollectStatsGroup(p.has_kappa(), p.kappa(), g_stats.mutable_kappa());
    CollectStatsGroup(p.has_dkappa(), p.dkappa(), g_stats.mutable_dkappa());
  }
  AINFO << g_stats.DebugString();
}

int main(int argc, char** argv) {
  using std::this_thread::sleep_for;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "planning_stats");

  FLAGS_alsologtostderr = true;
  AdapterManagerConfig config;
  config.set_is_ros(true);
  auto* sub_config = config.add_config();
  sub_config->set_mode(AdapterConfig::RECEIVE_ONLY);
  sub_config->set_type(AdapterConfig::PLANNING_TRAJECTORY);

  AdapterManager::Init(config);
  AdapterManager::AddPlanningCallback(OnPlanning);

  AINFO << "AdapterManager is initialized.";

  ros::spin();

  return 0;
}
