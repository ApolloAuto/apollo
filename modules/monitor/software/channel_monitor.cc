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
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "google/protobuf/compiler/parser.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/dynamic_message.h"

#include "modules/common/latency_recorder/proto/latency_record.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(channel_monitor_name, "ChannelMonitor",
              "Name of the channel monitor.");

DEFINE_double(channel_monitor_interval, 5,
              "Channel monitor checking interval in seconds.");

namespace apollo {
namespace monitor {
namespace {

using ReaderAndMessagePair =
    std::pair<std::shared_ptr<cyber::ReaderBase>,
              std::shared_ptr<google::protobuf::Message>>;

template <typename T>
ReaderAndMessagePair CreateReaderAndLatestsMessage(const std::string& channel) {
  const auto reader = MonitorManager::Instance()->CreateReader<T>(channel);
  reader->Observe();
  const auto message = reader->GetLatestObserved();
  return {reader, message};
}

// We have to specify exact type of each channel. This function is a wrapper for
// those only need a ReaderBase.
ReaderAndMessagePair GetReaderAndLatestMessage(const std::string& channel) {
  static const auto channel_function_map =
      std::unordered_map<std::string, std::function<ReaderAndMessagePair(
                                          const std::string& channel)>>{
          {FLAGS_control_command_topic,
           &CreateReaderAndLatestsMessage<control::ControlCommand>},
          {FLAGS_localization_topic,
           &CreateReaderAndLatestsMessage<localization::LocalizationEstimate>},
          {FLAGS_perception_obstacle_topic,
           &CreateReaderAndLatestsMessage<perception::PerceptionObstacles>},
          {FLAGS_prediction_topic,
           &CreateReaderAndLatestsMessage<prediction::PredictionObstacles>},
          {FLAGS_planning_trajectory_topic,
           &CreateReaderAndLatestsMessage<planning::ADCTrajectory>},
          {FLAGS_conti_radar_topic,
           &CreateReaderAndLatestsMessage<drivers::ContiRadar>},
          {FLAGS_relative_map_topic,
           &CreateReaderAndLatestsMessage<relative_map::MapMsg>},
          {FLAGS_pointcloud_topic,
           &CreateReaderAndLatestsMessage<drivers::PointCloud>},
          {FLAGS_pointcloud_16_topic,
           &CreateReaderAndLatestsMessage<drivers::PointCloud>},
          {FLAGS_pointcloud_128_topic,
           &CreateReaderAndLatestsMessage<drivers::PointCloud>},
          {FLAGS_pointcloud_16_front_up_topic,
           &CreateReaderAndLatestsMessage<drivers::PointCloud>}
          // Add more channels here if you want to monitor.
      };

  auto entry = channel_function_map.find(channel);
  if (entry != channel_function_map.end()) {
    return (entry->second)(channel);
  }

  AERROR << "Channel is not handled by ChannelMonitor: " << channel;
  return {nullptr, nullptr};
}

bool ValidateFields(const google::protobuf::Message& message,
                    const std::vector<std::string>& fields,
                    const size_t field_step) {
  if (field_step >= fields.size()) {
    return true;
  }
  const auto* desc = message.GetDescriptor();
  const auto* refl = message.GetReflection();
  const auto field_count = desc->field_count();
  for (int field_idx = 0; field_idx < field_count; ++field_idx) {
    const auto* field_desc = desc->field(field_idx);
    if (field_desc->name() == fields[field_step]) {
      if (field_desc->is_repeated()) {
        // For repeated field, we do not expect it has deeper level validation
        const auto size = refl->FieldSize(message, field_desc);
        return size > 0 && field_step == fields.size() - 1;
      }
      if (field_desc->type() !=
          google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        return refl->HasField(message, field_desc) &&
               field_step == fields.size() - 1;
      }
      return ValidateFields(refl->GetMessage(message, field_desc), fields,
                            field_step + 1);
    }
  }
  return false;
}

}  // namespace

ChannelMonitor::ChannelMonitor(
    const std::shared_ptr<LatencyMonitor>& latency_monitor)
    : RecurrentRunner(FLAGS_channel_monitor_name,
                      FLAGS_channel_monitor_interval),
      latency_monitor_(latency_monitor) {}

void ChannelMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  const auto& mode = manager->GetHMIMode();
  auto* components = manager->GetStatus()->mutable_components();
  for (const auto& iter : mode.monitored_components()) {
    const std::string& name = iter.first;
    const auto& config = iter.second;
    if (config.has_channel()) {
      double freq;
      const auto update_freq =
          latency_monitor_->GetFrequency(config.channel().name(), &freq);
      UpdateStatus(config.channel(),
                   components->at(name).mutable_channel_status(), update_freq,
                   freq);
    }
  }
}

void ChannelMonitor::UpdateStatus(
    const apollo::dreamview::ChannelMonitorConfig& config,
    ComponentStatus* status, const bool update_freq, const double freq) {
  status->clear_status();

  const auto reader_message_pair = GetReaderAndLatestMessage(config.name());
  const auto reader = reader_message_pair.first;
  const auto message = reader_message_pair.second;

  if (reader == nullptr) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::UNKNOWN,
        absl::StrCat(config.name(), " is not registered in ChannelMonitor."),
        status);
    return;
  }

  // Check channel delay
  const double delay = reader->GetDelaySec();
  if (delay < 0 || delay > config.delay_fatal()) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::FATAL,
        absl::StrCat(config.name(), " delayed for ", delay, " seconds."),
        status);
  }

  // Check channel fields
  const std::string field_sepr = ".";
  if (message != nullptr) {
    for (const auto& field : config.mandatory_fields()) {
      if (!ValidateFields(*message, absl::StrSplit(field, field_sepr), 0)) {
        SummaryMonitor::EscalateStatus(
            ComponentStatus::ERROR,
            absl::StrCat(config.name(), " missing field ", field), status);
      }
    }
  }

  // Check channel frequency
  if (update_freq) {
    if (freq > config.max_frequency_allowed()) {
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN,
          absl::StrCat(config.name(), " has frequency ", freq,
                       " > max allowed ", config.max_frequency_allowed()),
          status);
    }
    if (freq < config.min_frequency_allowed()) {
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN,
          absl::StrCat(config.name(), " has frequency ", freq,
                       " < min allowed ", config.max_frequency_allowed()),
          status);
    }
  }

  SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", status);
}

}  // namespace monitor
}  // namespace apollo
