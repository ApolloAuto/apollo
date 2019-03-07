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

#include "modules/monitor/software/channel_monitor.h"

#include <memory>
#include <string>

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/string_util.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

DEFINE_string(channel_monitor_name, "ChannelMonitor",
              "Name of the channel monitor.");

DEFINE_double(channel_monitor_interval, 5,
              "Channel monitor checking interval in seconds.");

namespace apollo {
namespace monitor {
namespace {
using apollo::common::util::StrCat;

// We have to specify exact type of each channel. This function is a wrapper for
// those only need a ReaderBase.
std::shared_ptr<cyber::ReaderBase> GetReader(const std::string& channel) {
  auto manager = MonitorManager::Instance();
  if (channel == FLAGS_control_command_topic) {
    return manager->CreateReader<control::ControlCommand>(channel);
  } else if (channel == FLAGS_localization_topic) {
    return manager->CreateReader<localization::LocalizationEstimate>(channel);
  } else if (channel == FLAGS_perception_obstacle_topic) {
    return manager->CreateReader<perception::PerceptionObstacles>(channel);
  } else if (channel == FLAGS_prediction_topic) {
    return manager->CreateReader<prediction::PredictionObstacles>(channel);
  } else if (channel == FLAGS_planning_trajectory_topic) {
    return manager->CreateReader<planning::ADCTrajectory>(channel);
  } else if (channel == FLAGS_conti_radar_topic) {
    return manager->CreateReader<drivers::ContiRadar>(channel);
  } else if (channel == FLAGS_relative_map_topic) {
    return manager->CreateReader<relative_map::MapMsg>(channel);
  } else if (channel == FLAGS_pointcloud_topic ||
             channel == FLAGS_pointcloud_128_topic ||
             channel == FLAGS_pointcloud_16_front_up_topic) {
    return manager->CreateReader<drivers::PointCloud>(channel);
  }
  // Add more channels here if you want to monitor.
  AERROR << "Channel is not handled by ChannelMonitor: " << channel;
  return nullptr;
}

}  // namespace

ChannelMonitor::ChannelMonitor()
    : RecurrentRunner(FLAGS_channel_monitor_name,
                      FLAGS_channel_monitor_interval) {}

void ChannelMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  const auto& mode = manager->GetHMIMode();
  auto* components = manager->GetStatus()->mutable_components();
  for (const auto& iter : mode.monitored_components()) {
    const std::string& name = iter.first;
    const auto& config = iter.second;
    if (config.has_channel()) {
      UpdateStatus(config.channel(),
                   components->at(name).mutable_channel_status());
    }
  }
}

void ChannelMonitor::UpdateStatus(
    const apollo::dreamview::ChannelMonitorConfig& config,
    ComponentStatus* status) {
  status->clear_status();
  auto reader = GetReader(config.name());
  if (reader == nullptr) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::UNKNOWN,
        StrCat(config.name(), " is not registered in ChannelMonitor."), status);
    return;
  }

  const double delay = reader->GetDelaySec();
  if (delay < 0 || delay > config.delay_fatal()) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::FATAL,
        StrCat(config.name(), " delayed for ", delay, " seconds."), status);
  } else {
    SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", status);
  }
}

}  // namespace monitor
}  // namespace apollo
