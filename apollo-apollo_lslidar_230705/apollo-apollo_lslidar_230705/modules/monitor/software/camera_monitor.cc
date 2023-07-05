/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/monitor/software/camera_monitor.h"

#include <memory>
#include <set>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(camera_monitor_name, "CameraMonitor",
              "Name of the camera monitor.");
DEFINE_double(camera_monitor_interval, 5,
              "Camera monitor checking interval in seconds.");
DEFINE_string(camera_component_name, "Camera", "Camera component name.");

namespace apollo {
namespace monitor {
namespace {

using ReaderAndMessagePair = std::pair<std::shared_ptr<cyber::ReaderBase>,
                                       std::shared_ptr<drivers::Image>>;

ReaderAndMessagePair CreateReaderAndLatestsMessage(const std::string& camera) {
  const auto reader =
      MonitorManager::Instance()->CreateReader<drivers::Image>(camera);
  reader->Observe();
  const auto message = reader->GetLatestObserved();
  reader->ClearData();
  return {reader, message};
}

static const auto camera_topic_set = std::set<std::string>{
    FLAGS_image_long_topic,         FLAGS_camera_image_long_topic,
    FLAGS_camera_image_short_topic, FLAGS_camera_front_6mm_topic,
    FLAGS_camera_front_6mm_2_topic, FLAGS_camera_front_12mm_topic,
    // Add more cameras here if you want to monitor.
};

}  // namespace

CameraMonitor::CameraMonitor()
    : RecurrentRunner(FLAGS_camera_monitor_name,
                      FLAGS_camera_monitor_interval) {}

void CameraMonitor::RunOnce(const double current_time) {
  auto* manager = MonitorManager::Instance();
  auto* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(), FLAGS_camera_component_name);
  if (component == nullptr) {
    // camera is not monitored in current mode, skip.
    return;
  }
  auto* status = component->mutable_other_status();
  UpdateStatus(status);
}

void CameraMonitor::UpdateStatus(ComponentStatus* status) {
  status->clear_status();
  std::string frame_id = "";
  for (const auto& topic : camera_topic_set) {
    const auto& reader_message_pair = CreateReaderAndLatestsMessage(topic);
    const auto& reader = reader_message_pair.first;
    const auto& message = reader_message_pair.second;
    if (reader != nullptr && message != nullptr) {
      if (frame_id.empty()) {
        const auto& header = message->header();
        if (header.has_frame_id()) {
          frame_id = header.frame_id();
        }
      } else {
        SummaryMonitor::EscalateStatus(
            ComponentStatus::ERROR,
            absl::StrCat("Only one camera is permitted"), status);
      }
    }
  }
  if (frame_id.empty()) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::ERROR, absl::StrCat("No camera is detected"), status);
  } else {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::OK, absl::StrCat("Detected one camera: ", frame_id),
        status);
  }
}

}  // namespace monitor
}  // namespace apollo
