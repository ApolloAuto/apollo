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

#include "modules/monitor/software/topic_monitor.h"

#include "cybertron/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/string_util.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

DEFINE_string(topic_monitor_name, "TopicMonitor", "Name of the topic monitor.");

DEFINE_double(topic_monitor_interval, 5, "Topic status checking interval (s).");

namespace apollo {
namespace monitor {

std::shared_ptr<cybertron::ReaderBase> TopicMonitor::CreateReaderFromChannel(
    const std::string &channel) {
  if (channel == FLAGS_control_command_topic) {
    return MonitorManager::CreateReader<apollo::control::ControlCommand>(
        FLAGS_control_command_topic);
  } else if (channel == FLAGS_localization_topic) {
    return MonitorManager::CreateReader<
        apollo::localization::LocalizationEstimate>(FLAGS_localization_topic);
  } else if (channel == FLAGS_perception_obstacle_topic) {
    return MonitorManager::CreateReader<
        apollo::perception::PerceptionObstacles>(
        FLAGS_perception_obstacle_topic);
  } else if (channel == FLAGS_prediction_topic) {
    return MonitorManager::CreateReader<
        apollo::prediction::PredictionObstacles>(FLAGS_prediction_topic);
  } else if (channel == FLAGS_planning_trajectory_topic) {
    return MonitorManager::CreateReader<apollo::planning::ADCTrajectory>(
        FLAGS_planning_trajectory_topic);
  } else if (channel == FLAGS_conti_radar_topic) {
    return MonitorManager::CreateReader<apollo::drivers::ContiRadar>(
        FLAGS_conti_radar_topic);
  } else if (channel == FLAGS_relative_map_topic) {
    return MonitorManager::CreateReader<apollo::relative_map::MapMsg>(
        FLAGS_relative_map_topic);
  }
  AFATAL << "Channel is not registered: " << channel;
  return nullptr;
}

TopicMonitor::TopicMonitor(const TopicConf &config, TopicStatus *status)
    : RecurrentRunner(FLAGS_topic_monitor_name, FLAGS_topic_monitor_interval),
      config_(config),
      status_(status) {
  reader_ = CreateReaderFromChannel(config.channel());
}

void TopicMonitor::RunOnce(const double current_time) {
  if (!reader_->HasReceived()) {
    status_->set_message_delay(-1);
    return;
  }
  const double delay = reader_->GetDelaySec();
  if (delay > config_.acceptable_delay()) {
    status_->set_message_delay(delay);
  } else {
    status_->clear_message_delay();
  }
}

}  // namespace monitor
}  // namespace apollo
