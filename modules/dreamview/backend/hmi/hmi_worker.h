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

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "nlohmann/json.hpp"
#include "google/protobuf/util/json_util.h"

#include "modules/common_msgs/audio_msgs/audio_event.pb.h"
#include "modules/common_msgs/basic_msgs/drive_event.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_status.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/external_command_msgs/action_command.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/simulation_msgs/scenario.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_config.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_mode.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/time.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

// Singleton worker which does the actual work of HMI actions.
class HMIWorker {
 public:
  using DvCallback = std::function<nlohmann::json(
      const std::string& function_name, const nlohmann::json& param_json)>;
  HMIWorker() : HMIWorker(cyber::CreateNode("HMI")) {}
  explicit HMIWorker(const std::shared_ptr<apollo::cyber::Node>& node);
  void Start(DvCallback callback_api);
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

  // Submit an AudioEvent
  void SubmitAudioEvent(const uint64_t event_time_ms, const int obstacle_id,
                        const int audio_type, const int moving_result,
                        const int audio_direction, const bool is_siren_on);

  // Submit a DriveEvent.
  void SubmitDriveEvent(const uint64_t event_time_ms,
                        const std::string& event_msg,
                        const std::vector<std::string>& event_types,
                        const bool is_reportable);

  // Run sensor calibration preprocess
  void SensorCalibrationPreprocess(const std::string& task_type);

  // Run vehicle calibration preprocess
  void VehicleCalibrationPreprocess();

  // Get current HMI status.
  HMIStatus GetStatus() const;

  bool UpdateScenarioSetToStatus(const std::string& scenario_set_id,
                                 const std::string& scenario_set_name);
  bool UpdateScenarioSet(const std::string& scenario_set_id,
                         const std::string& scenario_set_name,
                         ScenarioSet* new_scenario_set);
  bool UpdateDynamicModelToStatus(const std::string& dynamic_model_name);
  void UpdateComponentStatus();
  // bool UpdateRecordToStatus(const std::string& record_id,
  //                     const std::string& record_status);
  bool LoadRecords();
  bool ReloadVehicles();
  void GetScenarioSetPath(const std::string& scenario_set_id,
                          std::string* scenario_set_path);
  void UpdateCameraSensorChannelToStatus(const std::string& channel_name);
  void UpdatePointCloudChannelToStatus(const std::string& channel_name);

 private:
  void InitReadersAndWriters();
  void InitStatus();
  void StatusUpdateThreadLoop();
  // get command result
  std::string GetCommandRes(const std::string& cmd);

  // Start / reset current mode.
  void SetupMode() const;
  void ResetMode() const;
  bool ResetSimObstacle(const std::string& scenario_id);

  // Change current mode, launch, map, vehicle and driving mode.
  void ChangeMode(const std::string& mode_name);
  bool ChangeMap(const std::string& map_name,
                 bool restart_dynamic_model = true);
  void ChangeVehicle(const std::string& vehicle_name);
  void ChangeScenarioSet(const std::string& scenario_set_id);
  void ChangeRecord(const std::string& record_id);
  void ChangeDynamicModel(const std::string& dynamic_model_name);
  void ChangeScenario(const std::string& scenario_id);
  bool ChangeDrivingMode(const apollo::canbus::Chassis::DrivingMode mode);

  bool LoadScenarios();

  bool LoadDynamicModels();

  void DeleteScenarioSet(const std::string& scenario_set_id);
  void DeleteRecord(const std::string& record_id);
  void DeleteDynamicModel(const std::string& dynamic_model_name);

  void GetScenarioResourcePath(std::string* scenario_resource_path);
  void GetRecordPath(std::string* record_path);

  bool RePlayRecord(const std::string& record_id);

  // Start / stop a module.
  void StartModule(const std::string& module) const;
  void StopModule(const std::string& module) const;
  bool StopModuleByCommand(const std::string& stop_command) const;
  void StopRecordPlay();

  void ResetComponentStatusTimer();

  /**
   * @brief load the mode which is self defined by vehicles.
   * @param mode_config_path: the mode config path
   * @param current_vehicle_path: current selected vehicle conf absolute path.
   * @param self_defined_mode: the pointer to store vehicle defined mode config.
   * @return If vehicle has self-defined mode conf and load it successfully.
   */
  bool LoadVehicleDefinedMode(const std::string& mode_config_path,
                              const std::string& current_vehicle_path,
                              HMIMode* self_defined_mode);

  /**
   * @brief merge the mode's modules and monitored components
   * to current_mode_.
   * @param mode The mode to be merged.
   */
  void MergeToCurrentMode(HMIMode* mode);
  /**
   * @brief update the current_mode_'s modules and monitored components
   * to  hmi status.
   */
  void UpdateModeModulesAndMonitoredComponents();

  HMIConfig config_;

  // HMI status maintenance.
  HMIStatus status_;
  std::atomic<double> last_status_received_s_;
  bool monitor_timed_out_{true};
  HMIMode current_mode_;
  bool status_changed_ = false;
  size_t last_status_fingerprint_{};
  bool stop_ = false;
  mutable boost::shared_mutex status_mutex_;
  mutable size_t record_count_ = 0;
  std::future<void> thread_future_;
  std::vector<StatusUpdateHandler> status_update_handlers_;

  // Cyber members.
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Writer<HMIStatus>> status_writer_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::ActionCommand,
                            apollo::external_command::CommandStatus>>
      action_command_client_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::LaneFollowCommand,
                            apollo::external_command::CommandStatus>>
      lane_follow_command_client_;
  std::shared_ptr<cyber::Writer<apollo::audio::AudioEvent>> audio_event_writer_;
  std::shared_ptr<cyber::Writer<apollo::common::DriveEvent>>
      drive_event_writer_;
  DvCallback callback_api_;
};

}  // namespace dreamview
}  // namespace apollo
