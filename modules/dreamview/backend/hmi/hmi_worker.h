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

#pragma once

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/drive_event.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/dreamview/proto/hmi_config.pb.h"
#include "modules/dreamview/proto/hmi_mode.pb.h"
#include "modules/dreamview/proto/hmi_status.pb.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

// Singleton worker which does the actual work of HMI actions.
class HMIWorker {
 public:
  HMIWorker() : HMIWorker(cyber::CreateNode("HMI")) {}
  explicit HMIWorker(const std::shared_ptr<apollo::cyber::Node>& node);
  void Start();
  void Stop();

  // HMI action trigger.
  bool Trigger(const HMIAction action);
  bool Trigger(const HMIAction action, const std::string& value);

  // Register handler which will be called on HMIStatus update.
  // It will be called ASAP if there are changes, or else periodically
  // controlled by FLAGS_hmi_status_update_interval.
  using StatusUpdateHandler =
      std::function<void(const bool status_changed, HMIStatus* status)>;
  inline void RegisterStatusUpdateHandler(StatusUpdateHandler handler) {
    status_update_handlers_.push_back(handler);
  }

  // Submit a DriveEvent.
  void SubmitDriveEvent(const uint64_t event_time_ms,
                        const std::string& event_msg,
                        const std::vector<std::string>& event_types,
                        const bool is_reportable);

  // Get current HMI status.
  HMIStatus GetStatus() const;

  // Load HMIConfig and HMIMode.
  static HMIConfig LoadConfig();
  static HMIMode LoadMode(const std::string& mode_config_path);

 private:
  void InitReadersAndWriters();
  void InitStatus();
  void StatusUpdateThreadLoop();

  // Start / reset current mode.
  void SetupMode() const;
  void ResetMode() const;

  // Change current mode, launch, map, vehicle and driving mode.
  void ChangeMode(const std::string& mode_name);
  void ChangeMap(const std::string& map_name);
  void ChangeVehicle(const std::string& vehicle_name);
  bool ChangeDrivingMode(const apollo::canbus::Chassis::DrivingMode mode);

  // Start / stop a module.
  void StartModule(const std::string& module) const;
  void StopModule(const std::string& module) const;

  const HMIConfig config_;

  // HMI status maintenance.
  HMIStatus status_;
  HMIMode current_mode_;
  bool status_changed_ = false;
  bool stop_ = false;
  mutable boost::shared_mutex status_mutex_;
  std::future<void> thread_future_;
  std::vector<StatusUpdateHandler> status_update_handlers_;

  // Cyber members.
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Writer<HMIStatus>> status_writer_;
  std::shared_ptr<cyber::Writer<apollo::control::PadMessage>> pad_writer_;
  std::shared_ptr<cyber::Writer<apollo::common::DriveEvent>>
      drive_event_writer_;
};

}  // namespace dreamview
}  // namespace apollo
