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
#include <fstream>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "ros/include/ros/ros.h"
#include "ros/include/rosbag/bag.h"
#include "ros/include/rosbag/view.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"

DEFINE_string(adapter_config_filename,
              "/apollo/modules/data/conf/event_collector_adapter.conf",
              "Path for adapter configuration.");

DEFINE_string(events_filename, "events.txt", "File to save event list.");

DEFINE_string(events_related_bags_filename, "events_related_bags.txt",
              "File to save events related bags list.");

DEFINE_double(event_time_backtrack_seconds, 20,
              "Backtrack from the event time to consider bags as related.");

namespace apollo {
namespace data {
namespace {
using apollo::common::adapter::AdapterManager;
using apollo::canbus::Chassis;

void OnSigInt(int32_t signal_num) {
  // only response for ctrl + c
  if (signal_num != SIGINT) {
    return;
  }
  AINFO << "EventCollector got signal: " << signal_num;

  // Only stop once.
  static bool is_stopping = false;
  if (!is_stopping) {
    is_stopping = true;
    ros::shutdown();
  }
}

class EventCollector {
 public:
  void Init(int32_t argc, char **argv) {
    signal(SIGINT, OnSigInt);

    ros::init(argc, argv, "EventCollector");
    AdapterManager::Init(FLAGS_adapter_config_filename);
  }

  void Start() {
    AdapterManager::AddDriveEventCallback(&EventCollector::OnDriveEvent, this);
    AdapterManager::AddChassisCallback(&EventCollector::OnChassis, this);
    AdapterManager::AddMonitorCallback(&EventCollector::OnMonitorMessage, this);
    AINFO << "Start spining...";
    ros::spin();
  }

  void Stop() {
    // Save events_related_bags.
    std::ofstream fout(FLAGS_events_related_bags_filename);
    if (!fout) {
      AERROR << "Failed to write " << FLAGS_events_related_bags_filename;
      return;
    }
    AINFO << "Saving info to " << FLAGS_events_related_bags_filename << "...";
    fout << std::fixed << std::setprecision(1);

    sort(events_.begin(), events_.end());
    for (const std::string& bag_filename :
         apollo::common::util::ListSubPaths("./", DT_REG)) {
      if (!apollo::common::util::EndWith(bag_filename, ".bag")) {
        continue;
      }

      rosbag::Bag bag(bag_filename);
      rosbag::View view(bag);
      const double begin_time = view.getBeginTime().toSec();
      const double end_time =
          view.getEndTime().toSec() + FLAGS_event_time_backtrack_seconds;

      bool started_line = false;
      const std::tuple<double, std::string> key(begin_time, "");
      for (auto iter = std::lower_bound(events_.begin(), events_.end(), key);
           iter != events_.end() && std::get<0>(*iter) < end_time; ++iter) {
        // event_time = std::get<0>(*iter)
        // event_message = std::get<1>(*iter)
        if (!started_line) {
          started_line = true;
          fout << bag_filename << "\n";
        }
        fout << "  Offset = " << std::get<0>(*iter) - begin_time << ", "
             << std::get<1>(*iter) << "\n";
      }
    }
  }

 private:
  // Event time and message.
  std::vector<std::tuple<double, std::string>> events_;
  Chassis::DrivingMode last_driving_mode_;

  void OnDriveEvent(const apollo::common::DriveEvent& event) {
    // The header time is the real event time.
    SaveEvent(event.header().timestamp_sec(), "DriveEvent", event.event());
  }

  void OnChassis(const Chassis& chassis) {
    // Save event when driving_mode changes from COMPLETE_AUTO_DRIVE to
    // EMERGENCY_MODE which is taken as a disengagement.
    if (last_driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE &&
        chassis.driving_mode() == Chassis::EMERGENCY_MODE) {
      SaveEvent(chassis.header().timestamp_sec(), "Disengagement");
    }
    last_driving_mode_ = chassis.driving_mode();
  }

  void OnMonitorMessage(
      const apollo::common::monitor::MonitorMessage& monitor_msg) {
    using apollo::common::monitor::MonitorMessageItem;
    // Save all ERROR and FATAL monitor logs.
    const double time_sec = monitor_msg.header().timestamp_sec();
    for (const auto& item : monitor_msg.item()) {
      if (item.log_level() == MonitorMessageItem::ERROR ||
          item.log_level() == MonitorMessageItem::FATAL) {
        SaveEvent(time_sec, "MonitorErrorLog", item.msg());
      }
    }
  }

  void SaveEvent(const double timestamp_sec, const std::string& type,
                 const std::string& description = "") {
    const auto msg = apollo::common::util::StrCat(
        timestamp_sec, " [", type, "] ", description);

    AINFO << "SaveEvent: " << msg;
    events_.emplace_back(timestamp_sec, msg);

    // Append new event and flush.
    std::ofstream fout(FLAGS_events_filename, std::ofstream::app);
    if (fout) {
      fout << msg << std::endl;
    } else {
      AERROR << "Failed to write to " << FLAGS_events_filename;
    }
  }
};

}  // namespace
}  // namespace data
}  // namespace apollo

int main(int32_t argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::data::EventCollector event_collector;
  event_collector.Init(argc, argv);
  event_collector.Start();
  event_collector.Stop();

  return 0;
}
