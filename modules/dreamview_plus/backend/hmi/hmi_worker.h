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

#include "modules/common_msgs/audio_msgs/audio_event.pb.h"
#include "modules/common_msgs/basic_msgs/drive_event.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_config.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_mode.pb.h"
#include "modules/common_msgs/dreamview_msgs/hmi_status.pb.h"
#include "modules/common_msgs/external_command_msgs/action_command.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/monitor_msgs/system_status.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/dreamview_plus/backend/record_player/record_player_factory.h"

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
  explicit HMIWorker(
      apollo::common::monitor::MonitorLogBuffer monitor_log_buffer)
      : HMIWorker(cyber::CreateNode("HMI"), monitor_log_buffer) {}
  HMIWorker(
      const std::shared_ptr<apollo::cyber::Node>& node,
      const apollo::common::monitor::MonitorLogBuffer& monitor_log_buffer);
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

  bool UpdateDynamicModelToStatus(const std::string& dynamic_model_name);
  void UpdateComponentStatus();
  // bool UpdateRecordToStatus(const std::string& record_id,
  //                     const std::string& record_status);
  bool UpdateMapToStatus(const std::string& map_name);
  bool LoadRecords();
  bool LoadRecordAndChangeStatus(const std::string& record_name);
  bool LoadRecord(const std::string& record_name,
                  const std::string& record_file_path, double* total_time_s);
  bool RecordIsLoaded(const std::string& record_id);
  bool RePlayRecord();
  bool LoadRtkRecords();
  void UpdateRtkRecordToStatus(const std::string &new_name);
  /**
   * @brief control the nohup play record process by action_type
   * @param action_type The action to control the nohup play record process
   * optional values like pause and continue.
   * @return If the action executed successfully.
   */
  bool handlePlayRecordProcess(const std::string& action_type);
  /**
   * @brief reset the play record progress to jump.
   * @param progress The progress to jump.
   * @return If the action executed successfully.
   */
  bool ResetRecordProgress(const double& progress);
  bool ReloadVehicles();
  void GetScenarioSetPath(const std::string& scenario_set_id,
                          std::string* scenario_set_path);
  void UpdateCameraSensorChannelToStatus(const std::string& channel_name);
  void UpdatePointCloudChannelToStatus(const std::string& channel_name);

  // Start / Stop Data Recorder
  bool StartDataRecorder();
  bool StopDataRecorder();
  int SaveDataRecorder(const std::string& new_name);
  bool DeleteDataRecorder();

  // Start / Stop rtk Data Recorder
  bool StartRtkDataRecorder();
  bool StopRtkDataRecorder();
  bool DeleteRtkDataRecorder();
  int SaveRtkDataRecorder(const std::string& new_name);
  bool StopPlayRtkRecorder();
  nlohmann::json StartPlayRtkRecorder();

  void ChangeMapVal(const std::string& map_name);

  // kv database update interface
  bool AddOrModifyObjectToDB(const std::string& key, const std::string& value);
  bool DeleteObjectToDB(const std::string& key);
  std::string GetObjectFromDB(const std::string& key);
  std::vector<std::pair<std::string, std::string>> GetTuplesWithTypeFromDB(
      const std::string& type);

  bool StartTerminal();

  /**
   * @brief Check if a process exists
   * @param process_name The name of the process to check
   * @return True if the process exists
   */
  bool isProcessRunning(const std::string& process_name);

 private:
  void InitReadersAndWriters();
  void InitStatus();
  void StatusUpdateThreadLoop();

  // Start / reset current mode.
  void SetupMode();
  void ResetMode();

  // Change current mode, launch, map, vehicle and driving mode.
  void ChangeMode(const std::string& mode_name);
  bool ChangeMap(const std::string& map_name);
  bool SelectAndReloadMap(const std::string& map_name);
  void ChangeVehicle(const std::string& vehicle_name);
  void ChangeRecord(const std::string& record_id);
  void ChangeRtkRecord(const std::string& record_id);
  void ChangeOperation(const std::string& operation_str);
  void ChangeDynamicModel(const std::string& dynamic_model_name);
  bool ChangeDrivingMode(const apollo::canbus::Chassis::DrivingMode mode);
  void ClearRecordInfo();
  void ClearRtkRecordInfo();
  void ClearInvalidRecordStatus(const HMIModeOperation& operation);
  void ReloadMaps();
  /**
   * @brief clear invalid selected resource under different operations.
   * operation is strong associated with resources,for some resources are only
   * used under special operation.when change operations should clear invalid
   * selected resources.
   * @param operation: the operation to be changed
   */
  void ClearInvalidResourceUnderChangeOperation(
      const HMIModeOperation operation);


  bool LoadDynamicModels();
  void DeleteMap(const std::string& map_name);
  void DeleteScenarioSet(const std::string& scenario_set_id);
  void DeleteRecord(const std::string& record_id);
  void DeleteDynamicModel(const std::string& dynamic_model_name);
  void DeleteVehicleConfig(const std::string& vehicle_name);
  // Delete the v2x configuration file from the corresponding
  // vehicle configuration file.
  void DeleteV2xConfig(const std::string& vehicle_name);
  void GetScenarioResourcePath(std::string* scenario_resource_path);
  void GetRecordPath(std::string* record_path);
  void GetRtkRecordPath(std::string* record_path);

  // Start / stop a module.
  void StartModule(const std::string& module);
  void StopModule(const std::string& module);
  void LockModule(const std::string& module, const bool& lock_flag);
  // Stop play current record process but current record not changed
  void StopRecordPlay(const std::string& record_id = "");

  void ResetComponentStatusTimer();

  bool ReadRecordInfo(const std::string& file, double* total_time_s) const;
  // Periodically check whether the monitor is turned on.
  void OnTimer(const double &overtime_time);

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

  /**
   * @brief Increase the expected number of modules to open
   */
  void AddExpectedModules(const HMIAction& action);

  /**
   * @brief Check if the package has been installed
   */
  bool PackageExist(const std::string& package_name);

  HMIConfig config_;

  // HMI status maintenance.
  HMIStatus status_;
  std::atomic<double> last_status_received_s_;
  bool monitor_timed_out_{true};
  HMIMode current_mode_;
  bool status_changed_ = false;
  size_t last_status_fingerprint_{};
  bool stop_ = false;
  // Used to detect the cycle time of monitor running.
  uint32_t time_interval_ms_;
  // The timeout period for monitor not returning a message.
  double overtime_time_;
  mutable boost::shared_mutex status_mutex_;
  mutable size_t record_count_ = 0;
  std::future<void> thread_future_;
  std::vector<StatusUpdateHandler> status_update_handlers_;
  std::map<std::string, bool> previous_modules_lock_;
  // Cyber members.
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::monitor::SystemStatus>>
      monitor_reader_;
  std::shared_ptr<cyber::Writer<HMIStatus>> status_writer_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::ActionCommand,
                            apollo::external_command::CommandStatus>>
      action_command_client_;
  std::shared_ptr<cyber::Writer<apollo::audio::AudioEvent>> audio_event_writer_;
  std::shared_ptr<cyber::Writer<apollo::common::DriveEvent>>
      drive_event_writer_;
  DvCallback callback_api_;
  std::unique_ptr<cyber::Timer> monitor_timer_;
  apollo::common::monitor::MonitorLogBuffer monitor_log_buffer_;
};

}  // namespace dreamview
}  // namespace apollo
