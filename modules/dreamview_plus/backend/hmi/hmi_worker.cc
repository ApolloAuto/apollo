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

#include "modules/dreamview_plus/backend/hmi/hmi_worker.h"

#include <cstdio>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"

#include "cyber/proto/dag_conf.pb.h"
#include "cyber/proto/record.pb.h"
#include "modules/dreamview/proto/scenario.pb.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/kv_db/kv_db.h"
#include "modules/common/util/future.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/message_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/fuel_monitor/data_collection_monitor.h"
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor_gflags.h"
#include "modules/dreamview/backend/common/fuel_monitor/fuel_monitor_manager.h"
#include "modules/dreamview/backend/common/fuel_monitor/preprocess_monitor.h"
#include "modules/dreamview/backend/common/vehicle_manager/vehicle_manager.h"
#include "modules/dreamview/backend/common/sim_control_manager/sim_control_manager.h"
#include "modules/dreamview/backend/common/util/hmi_util.h"

DEFINE_string(cyber_recorder_play_command, "cyber_recorder play -p 1 -f ",
              "Cyber recorder play command");

namespace apollo {
namespace dreamview {
namespace {

using apollo::audio::AudioEvent;
using apollo::canbus::Chassis;
using apollo::common::DriveEvent;
using apollo::common::KVDB;
using apollo::control::DrivingAction;
using apollo::cyber::Clock;
using apollo::cyber::Node;
using apollo::cyber::proto::DagConfig;
using apollo::cyber::proto::RecordInfo;
using apollo::dreamview::SimTicket;
using apollo::dreamview::UserAdsGroup;
using apollo::external_command::ActionCommand;
using apollo::external_command::CommandStatus;
using apollo::localization::LocalizationEstimate;
using apollo::monitor::ComponentStatus;
using apollo::monitor::SystemStatus;
using RLock = boost::shared_lock<boost::shared_mutex>;
using WLock = boost::unique_lock<boost::shared_mutex>;
using Json = nlohmann::json;

constexpr char kNavigationModeName[] = "Navigation";
// operations based on sim control is enabled, note that it is
// consistent with the field name in hmistatus
const std::vector<HMIModeOperation> OperationBasedOnSimControl = {
    HMIModeOperation::Scenario_Sim,
    HMIModeOperation::Sim_Control,
};

template <class FlagType, class ValueType>
void SetGlobalFlag(std::string_view flag_name, const ValueType &value,
                   FlagType *flag) {
  constexpr char kGlobalFlagfile[] =
      "/apollo/modules/common/data/global_flagfile.txt";
  if (*flag != value) {
    *flag = value;
    // Overwrite global flagfile.
    std::ofstream fout(kGlobalFlagfile, std::ios_base::app);
    ACHECK(fout) << "Fail to open global flagfile " << kGlobalFlagfile;
    fout << "\n--" << flag_name << "=" << value << std::endl;
  }
}

void System(std::string_view cmd) {
  const int ret = std::system(cmd.data());
  if (ret == 0) {
    AINFO << "SUCCESS: " << cmd;
  } else {
    AERROR << "FAILED(" << ret << "): " << cmd;
  }
}

}  // namespace

HMIWorker::HMIWorker(
    const std::shared_ptr<Node> &node,
    const apollo::common::monitor::MonitorLogBuffer& monitor_log_buffer)
    : config_(util::HMIUtil::LoadConfig(FLAGS_dv_plus_hmi_modes_config_path)),
      node_(node),
      monitor_log_buffer_(monitor_log_buffer) {
  InitStatus();
  time_interval_ms_ = 3000;
  overtime_time_ = 3;
  monitor_timer_.reset(new cyber::Timer(
      time_interval_ms_, [this]() { this->OnTimer(overtime_time_); }, false));
  monitor_timer_->Start();
}

void HMIWorker::Start(DvCallback callback_api) {
  callback_api_ = callback_api;
  InitReadersAndWriters();
  RegisterStatusUpdateHandler(
      [this](const bool status_changed, HMIStatus *status) {
        apollo::common::util::FillHeader("HMI", status);
        status_writer_->Write(*status);
        status->clear_header();
      });
  ResetComponentStatusTimer();
  thread_future_ = cyber::Async(&HMIWorker::StatusUpdateThreadLoop, this);
}

void HMIWorker::Stop() {
  stop_ = true;
  std::system(FLAGS_terminal_stop_cmd.data());
  if (thread_future_.valid()) {
    thread_future_.get();
  }
}

bool HMIWorker::LoadVehicleDefinedMode(const std::string &mode_config_path,
                                       const std::string &current_vehicle_path,
                                       HMIMode *self_defined_mode) {
  const std::string mode_file_name =
      cyber::common::GetFileName(mode_config_path);
  const std::string vehicle_mode_config_path =
      current_vehicle_path + "/dreamview_conf/hmi_modes/" + mode_file_name;
  if (!cyber::common::PathExists(vehicle_mode_config_path)) {
    return false;
  }
  ACHECK(cyber::common::GetProtoFromFile(vehicle_mode_config_path,
                                         self_defined_mode))
      << "Unable to parse vehicle self defined HMIMode from file "
      << vehicle_mode_config_path;
  util::HMIUtil::TranslateCyberModules(self_defined_mode);
  return true;
}

void HMIWorker::InitStatus() {
  static constexpr char kDockerImageEnv[] = "DOCKER_IMG";
  status_.set_docker_image(cyber::common::GetEnv(kDockerImageEnv));
  status_.set_utm_zone_id(FLAGS_local_utm_zone_id);

  // Populate modes and current_mode.
  const auto &modes = config_.modes();
  for (const auto &iter : modes) {
    const std::string &mode = iter.first;
    status_.add_modes(mode);
    if (mode == FLAGS_vehicle_calibration_mode) {
      FuelMonitorManager::Instance()->RegisterFuelMonitor(
          mode, std::make_unique<DataCollectionMonitor>());
      FuelMonitorManager::Instance()->RegisterFuelMonitor(
          mode, std::make_unique<PreprocessMonitor>());
    } else if (mode == FLAGS_lidar_calibration_mode) {
      FuelMonitorManager::Instance()->RegisterFuelMonitor(
          mode, std::make_unique<PreprocessMonitor>("lidar_to_gnss"));
    } else if (mode == FLAGS_camera_calibration_mode) {
      FuelMonitorManager::Instance()->RegisterFuelMonitor(
          mode, std::make_unique<PreprocessMonitor>("camera_to_lidar"));
    }
  }

  // Populate maps and current_map.
  for (const auto &map_entry : config_.maps()) {
    status_.add_maps(map_entry.first);

    // If current FLAG_map_dir is available, set it as current_map.
    if (map_entry.second == FLAGS_map_dir) {
      status_.set_current_map(map_entry.first);
    }
  }

  // Populate vehicles and current_vehicle.
  for (const auto &vehicle : config_.vehicles()) {
    status_.add_vehicles(vehicle.first);
  }

  // Initial HMIMode by priority:
  //   1. NavigationMode if --use_navigation_mode is specified explicitly.
  //   2. CachedMode if it's stored in KV database.
  //   3. default_hmi_mode if it is available.
  //   4. Pick the first available mode.
  const std::string cached_mode =
      KVDB::Get(FLAGS_current_mode_db_key).value_or("");
  if (FLAGS_use_navigation_mode && ContainsKey(modes, kNavigationModeName)) {
    ChangeMode(kNavigationModeName);
  } else if (ContainsKey(modes, cached_mode)) {
    ChangeMode(cached_mode);
  } else if (ContainsKey(modes, FLAGS_default_hmi_mode)) {
    ChangeMode(FLAGS_default_hmi_mode);
  } else {
    ChangeMode(modes.begin()->first);
  }
}

void HMIWorker::InitReadersAndWriters() {
  status_writer_ = node_->CreateWriter<HMIStatus>(FLAGS_hmi_status_topic);
  action_command_client_ = node_->CreateClient<ActionCommand, CommandStatus>(
      FLAGS_action_command_topic);
  audio_event_writer_ =
      node_->CreateWriter<AudioEvent>(FLAGS_audio_event_topic);
  drive_event_writer_ =
      node_->CreateWriter<DriveEvent>(FLAGS_drive_event_topic);

  monitor_reader_ = node_->CreateReader<SystemStatus>(
      FLAGS_system_status_topic,
      [this](const std::shared_ptr<SystemStatus> &system_status) {
        this->ResetComponentStatusTimer();

        WLock wlock(status_mutex_);

        const bool is_realtime_msg =
            FLAGS_use_sim_time
                ? system_status->is_realtime_in_simulation()
                : Clock::NowInSeconds() -
                          system_status->header().timestamp_sec() <
                      FLAGS_system_status_lifetime_seconds;
        // Update modules running status from realtime SystemStatus.
        if (is_realtime_msg) {
          for (auto &iter : *status_.mutable_modules()) {
            bool previous_second = iter.second;
            auto *status = FindOrNull(system_status->hmi_modules(), iter.first);
            iter.second =
                status != nullptr && status->status() == ComponentStatus::OK;
            if (previous_second != iter.second) {
              LockModule(iter.first, false);
            }
          }
          // system_status is true, indicating that the monitor immediately
          // detected the module startup status
          if (system_status->detect_immediately()) {
            status_.set_expected_modules(status_.expected_modules() - 1);
          }
        }
        // Update monitored components status.
        for (auto &iter : *status_.mutable_monitored_components()) {
          auto *status = FindOrNull(system_status->components(), iter.first);
          if (status != nullptr) {
            iter.second = status->summary();
          } else {
            iter.second.set_status(ComponentStatus::UNKNOWN);
            iter.second.set_message("Status not reported by Monitor.");
          }
        }

        // Update other components status.
        for (auto &iter : *status_.mutable_other_components()) {
          auto *status =
              FindOrNull(system_status->other_components(), iter.first);
          if (status != nullptr) {
            iter.second.CopyFrom(*status);
          } else {
            iter.second.set_status(ComponentStatus::UNKNOWN);
            iter.second.set_message("Status not reported by Monitor.");
          }
        }
        // For global components.
        for (auto &iter : *system_status->mutable_global_components()) {
          (*status_.mutable_global_components())[iter.first] = iter.second;
        }

        // Check if the status is changed.
        const size_t new_fingerprint =
            apollo::common::util::MessageFingerprint(status_);
        if (last_status_fingerprint_ != new_fingerprint) {
          status_changed_ = true;
          last_status_fingerprint_ = new_fingerprint;
        }
      });

  node_->CreateReader<RecordInfo>(
      FLAGS_record_info_topic,
      [this](const std::shared_ptr<RecordInfo> &record_info) {
        WLock wlock(status_mutex_);
        if (record_info->record_name() ==
            status_.current_record_status().current_record_id()) {
          status_.mutable_current_record_status()->set_curr_time_s(
              record_info.get()->curr_time_s());
          status_changed_ = true;
        }
      });

  localization_reader_ =
      node_->CreateReader<LocalizationEstimate>(FLAGS_localization_topic);
  // Received Chassis, trigger action if there is high beam signal.
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [this](const std::shared_ptr<Chassis> &chassis) {
        if (Clock::NowInSeconds() - chassis->header().timestamp_sec() <
            FLAGS_system_status_lifetime_seconds) {
          if (chassis->signal().high_beam()) {
            // Currently we do nothing on high_beam signal.
            const bool ret = Trigger(HMIAction::NONE);
            AERROR_IF(!ret) << "Failed to execute high_beam action.";
          }
        }
      });
}

bool HMIWorker::Trigger(const HMIAction action) {
  AERROR << "HMIAction " << HMIAction_Name(action) << " was triggered!";
  switch (action) {
    case HMIAction::NONE:
      break;
    case HMIAction::SETUP_MODE:
      SetupMode();
      AddExpectedModules(action);
      break;
    case HMIAction::ENTER_AUTO_MODE:
      return ChangeDrivingMode(Chassis::COMPLETE_AUTO_DRIVE);
    case HMIAction::DISENGAGE:
      return ChangeDrivingMode(Chassis::COMPLETE_MANUAL);
    case HMIAction::RESET_MODE:
      ResetMode();
      break;
    case HMIAction::LOAD_DYNAMIC_MODELS:
      LoadDynamicModels();
      break;
    case HMIAction::LOAD_RECORDS:
      LoadRecords();
      break;
    case HMIAction::LOAD_RTK_RECORDS:
      LoadRtkRecords();
      break;
    case HMIAction::STOP_RECORD:
      StopRecordPlay();
      break;
    case HMIAction::LOAD_MAPS:
      ReloadMaps();
      break;
    default:
      AERROR << "HMIAction not implemented, yet!";
      return false;
  }
  return true;
}

bool HMIWorker::Trigger(const HMIAction action, const std::string &value) {
  AINFO << "HMIAction " << HMIAction_Name(action) << "(" << value
        << ") was triggered!";
  bool ret = true;
  switch (action) {
    case HMIAction::CHANGE_MODE:
      ChangeMode(value);
      break;
    case HMIAction::CHANGE_MAP:
      ret = ChangeMap(value);
      // after change map;if sim_control is enabled,restart adc position
      // with new map and sim control
      SimControlManager::Instance()->Restart();
      break;
    case HMIAction::CHANGE_VEHICLE:
      ChangeVehicle(value);
      break;
    case HMIAction::START_MODULE:
      StartModule(value);
      AddExpectedModules(action);
      break;
    case HMIAction::STOP_MODULE:
      StopModule(value);
      break;
    case HMIAction::DELETE_SCENARIO_SET:
      DeleteScenarioSet(value);
      break;
    case HMIAction::CHANGE_DYNAMIC_MODEL:
      ChangeDynamicModel(value);
      break;
    case HMIAction::DELETE_DYNAMIC_MODEL:
      DeleteDynamicModel(value);
      break;
    case HMIAction::DELETE_RECORD:
      DeleteRecord(value);
      break;
    case HMIAction::CHANGE_RECORD:
      ChangeRecord(value);
      break;
    case HMIAction::CHANGE_RTK_RECORD:
      ChangeRtkRecord(value);
      break;
    case HMIAction::CHANGE_OPERATION:
      ChangeOperation(value);
      break;
    case HMIAction::DELETE_VEHICLE_CONF:
      DeleteVehicleConfig(value);
    case HMIAction::DELETE_V2X_CONF:
      DeleteV2xConfig(value);
      break;
    case HMIAction::DELETE_MAP:
      DeleteMap(value);
      break;
    case HMIAction::LOAD_RECORD:
      LoadRecordAndChangeStatus(value);
      break;
    default:
      AERROR << "HMIAction not implemented, yet!";
      return false;
  }
  return ret;
}

void HMIWorker::SubmitAudioEvent(const uint64_t event_time_ms,
                                 const int obstacle_id, const int audio_type,
                                 const int moving_result,
                                 const int audio_direction,
                                 const bool is_siren_on) {
  std::shared_ptr<AudioEvent> audio_event = std::make_shared<AudioEvent>();
  apollo::common::util::FillHeader("HMI", audio_event.get());
  // Here we reuse the header time field as the event occurring time.
  // A better solution might be adding an event time field to DriveEvent proto
  // to make it clear.
  audio_event->mutable_header()->set_timestamp_sec(
      static_cast<double>(event_time_ms) / 1000.0);
  audio_event->set_id(obstacle_id);
  audio_event->set_audio_type(
      static_cast<apollo::audio::AudioType>(audio_type));
  audio_event->set_moving_result(
      static_cast<apollo::audio::MovingResult>(moving_result));
  audio_event->set_audio_direction(
      static_cast<apollo::audio::AudioDirection>(audio_direction));
  audio_event->set_siren_is_on(is_siren_on);

  // Read the current localization pose
  localization_reader_->Observe();
  if (localization_reader_->Empty()) {
    AERROR << "Failed to get localization associated with the audio event: "
           << audio_event->DebugString() << "\n Localization reader is empty!";
    return;
  }

  const std::shared_ptr<LocalizationEstimate> localization =
      localization_reader_->GetLatestObserved();
  audio_event->mutable_pose()->CopyFrom(localization->pose());
  AINFO << "AudioEvent: " << audio_event->DebugString();

  audio_event_writer_->Write(audio_event);
}

void HMIWorker::SubmitDriveEvent(const uint64_t event_time_ms,
                                 const std::string &event_msg,
                                 const std::vector<std::string> &event_types,
                                 const bool is_reportable) {
  std::shared_ptr<DriveEvent> drive_event = std::make_shared<DriveEvent>();
  apollo::common::util::FillHeader("HMI", drive_event.get());
  // TODO(xiaoxq): Here we reuse the header time field as the event occurring
  // time. A better solution might be adding the field to DriveEvent proto to
  // make it clear.
  drive_event->mutable_header()->set_timestamp_sec(
      static_cast<double>(event_time_ms) / 1000.0);
  drive_event->set_event(event_msg);
  drive_event->set_is_reportable(is_reportable);
  for (const auto &type_name : event_types) {
    DriveEvent::Type type;
    if (DriveEvent::Type_Parse(type_name, &type)) {
      drive_event->add_type(type);
    } else {
      AERROR << "Failed to parse drive event type:" << type_name;
    }
  }
  drive_event_writer_->Write(drive_event);
}

void HMIWorker::SensorCalibrationPreprocess(const std::string &task_type) {
  std::string start_command = absl::StrCat(
      "nohup bash /apollo/modules/tools/sensor_calibration/extract_data.sh -t ",
      task_type, " &");
  System(start_command);
}

void HMIWorker::VehicleCalibrationPreprocess() {
  std::string start_command = absl::StrCat(
      "nohup bash /apollo/modules/tools/vehicle_calibration/preprocess.sh "
      "--vehicle_type=\"",
      status_.current_vehicle(), "\" --record_num=", record_count_, " &");
  System(start_command);
}

bool HMIWorker::ChangeDrivingMode(const Chassis::DrivingMode mode) {
  // Always reset to MANUAL mode before changing to other mode.
  const std::string mode_name = Chassis::DrivingMode_Name(mode);
  if (mode != Chassis::COMPLETE_MANUAL) {
    if (!ChangeDrivingMode(Chassis::COMPLETE_MANUAL)) {
      AERROR << "Failed to reset to MANUAL before changing to " << mode_name;
      return false;
    }
  }

  auto command = std::make_shared<ActionCommand>();

  switch (mode) {
    case Chassis::COMPLETE_MANUAL:
      command->set_command(
          apollo::external_command::ActionCommandType::SWITCH_TO_MANUAL);
      break;
    case Chassis::COMPLETE_AUTO_DRIVE:
      command->set_command(
          apollo::external_command::ActionCommandType::SWITCH_TO_AUTO);
      break;
    default:
      AFATAL << "Change driving mode to " << mode_name << " not implemented!";
      return false;
  }

  static constexpr int kMaxTries = 3;
  static constexpr auto kTryInterval = std::chrono::milliseconds(500);
  for (int i = 0; i < kMaxTries; ++i) {
    // Send driving action periodically until entering target driving mode.
    common::util::FillHeader("HMI", command.get());
    action_command_client_->SendRequest(command);

    std::this_thread::sleep_for(kTryInterval);

    chassis_reader_->Observe();
    if (chassis_reader_->Empty()) {
      AERROR << "No Chassis message received!";
    } else if (chassis_reader_->GetLatestObserved()->driving_mode() == mode) {
      return true;
    }
  }
  AERROR << "Failed to change driving mode to " << mode_name;
  return false;
}

bool HMIWorker::ChangeMap(const std::string &map_name) {
  if (status_.current_map() == map_name) {
    // Returns true if the map is switched to the same one.
    return true;
  }
  return SelectAndReloadMap(map_name);
}

bool HMIWorker::SelectAndReloadMap(const std::string &map_name) {
  const std::string *map_dir = FindOrNull(config_.maps(), map_name);
  if (map_dir == nullptr) {
    AERROR << "Unknown map " << map_name;
    return false;
  }

  // Load the map first and then change the currently selected map;this will
  // cause interactive waiting for loading a large map takes a long time
  SetGlobalFlag("map_dir", *map_dir, &FLAGS_map_dir);
  ResetMode();
  Json callback_res = callback_api_("MapServiceReloadMap", {});
  {
    // Update current_map status.
    WLock wlock(status_mutex_);
    status_.set_current_map(map_name);
    status_changed_ = true;
  }
  return callback_res["result"];
}

void HMIWorker::UpdateModeModulesAndMonitoredComponents() {
  status_.clear_modules();
  status_.clear_modules_lock();
  previous_modules_lock_.clear();
  for (const auto &iter : current_mode_.modules()) {
    status_.mutable_modules()->insert({iter.first, false});
    status_.mutable_modules_lock()->insert({iter.first, false});
    previous_modules_lock_.insert({iter.first, false});
  }

  // Update monitored components of current mode.
  status_.clear_monitored_components();
  for (const auto &iter : current_mode_.monitored_components()) {
    status_.mutable_monitored_components()->insert({iter.first, {}});
  }
}

void HMIWorker::ChangeVehicle(const std::string &vehicle_name) {
  const std::string *vehicle_dir = FindOrNull(config_.vehicles(), vehicle_name);
  if (vehicle_dir == nullptr) {
    AERROR << "Unknown vehicle " << vehicle_name;
    return;
  }
  std::string current_mode;
  {
    // Update current_vehicle status.
    WLock wlock(status_mutex_);
    current_mode = status_.current_mode();
    if (status_.current_vehicle() == vehicle_name) {
      return;
    }
    try {
      // try to get vehicle type from calibration data directory
      // TODO(jinping): add vehicle config specs and move to vehicle config
      const std::string vehicle_type_file_path = *vehicle_dir + "/vehicle_type";
      std::string vehicle_type_str;
      cyber::common::GetContent(vehicle_type_file_path, &vehicle_type_str);
      int vehicle_type = std::stoi(vehicle_type_str);
      status_.set_current_vehicle_type(vehicle_type);
    } catch (const std::exception &e) {
      AWARN << "get vehicle type config failed: " << e.what();
      status_.clear_current_vehicle_type();
    }
    status_.set_current_vehicle(vehicle_name);
    status_changed_ = true;
  }
  ResetMode();
  // before reset mode
  HMIMode vehicle_defined_mode;
  const std::string mode_config_path = config_.modes().at(current_mode);
  if (LoadVehicleDefinedMode(mode_config_path, *vehicle_dir,
                             &vehicle_defined_mode)) {
    MergeToCurrentMode(&vehicle_defined_mode);
  } else {
    // modules may have been modified the last time selected a vehicle
    // need to be recovery by load mode
    current_mode_ = util::HMIUtil::LoadMode(mode_config_path);
  }
  {
    WLock wlock(status_mutex_);
    UpdateModeModulesAndMonitoredComponents();
  }
  ACHECK(VehicleManager::Instance()->UseVehicle(*vehicle_dir));
  // Restart Fuel Monitor
  auto *monitors = FuelMonitorManager::Instance()->GetCurrentMonitors();
  if (monitors != nullptr) {
    for (const auto &monitor : *monitors) {
      if (monitor.second->IsEnabled()) {
        monitor.second->Restart();
      }
    }
  }
}

void HMIWorker::MergeToCurrentMode(HMIMode *mode) {
  current_mode_.clear_modules();
  current_mode_.clear_cyber_modules();
  current_mode_.clear_monitored_components();
  current_mode_.mutable_modules()->swap(*(mode->mutable_modules()));
  current_mode_.mutable_monitored_components()->swap(
      *(mode->mutable_monitored_components()));
}

void HMIWorker::ChangeMode(const std::string &mode_name) {
  if (!ContainsKey(config_.modes(), mode_name)) {
    AERROR << "Cannot change to unknown mode " << mode_name;
    return;
  }

  {
    RLock rlock(status_mutex_);
    // Skip if mode doesn't actually change.
    if (status_.current_mode() == mode_name) {
      return;
    }
  }
  ResetMode();
  std::string default_operation_str = "";
  {
    WLock wlock(status_mutex_);
    status_.set_current_mode(mode_name);
    current_mode_ = util::HMIUtil::LoadMode(config_.modes().at(mode_name));
    // for vehicle self-defined module
    HMIMode vehicle_defined_mode;
    const std::string *vehicle_dir =
        FindOrNull(config_.vehicles(), status_.current_vehicle());
    if (vehicle_dir != nullptr &&
        LoadVehicleDefinedMode(config_.modes().at(mode_name), *vehicle_dir,
                               &vehicle_defined_mode)) {
      MergeToCurrentMode(&vehicle_defined_mode);
    }
    UpdateModeModulesAndMonitoredComponents();

    status_.clear_other_components();
    for (const auto &iter : current_mode_.other_components()) {
      status_.mutable_other_components()->insert({iter.first, {}});
    }

    // update operations of current mode
    status_.clear_operations();
    status_.mutable_operations()->CopyFrom(current_mode_.operations());

    // Temporarily used for simulation plugins, removed when simulation is used
    // as a standalone mode.
    if ((mode_name == "Pnc" || mode_name == "Default") &&
        PackageExist("simulator-plugin")) {
      status_.add_operations(HMIModeOperation::Scenario_Sim);
    }

    if (current_mode_.has_default_operation()) {
      default_operation_str =
          HMIModeOperation_Name(current_mode_.default_operation());
    }
    status_changed_ = true;
  }
  // Because the change operation involves the state change of the sim control
  // therefore, the function change operation is used when loading the mode;
  // not only the Hmi status is changed; the sim control is also switched on and
  // off.
  if (!default_operation_str.empty()) {
    ChangeOperation(default_operation_str);
  }
  FuelMonitorManager::Instance()->SetCurrentMode(mode_name);
  KVDB::Put(FLAGS_current_mode_db_key, mode_name);
}

void HMIWorker::StartModule(const std::string &module) {
  const Module *module_conf = FindOrNull(current_mode_.modules(), module);
  if (module_conf != nullptr) {
    {
      WLock wlock(status_mutex_);
      LockModule(module, true);
    }
    System(module_conf->start_command());
  } else {
    AERROR << "Cannot find module " << module;
  }
}

void HMIWorker::StopModule(const std::string &module) {
  const Module *module_conf = FindOrNull(current_mode_.modules(), module);
  if (module_conf != nullptr) {
    {
      WLock wlock(status_mutex_);
      LockModule(module, true);
    }
    System(module_conf->stop_command());
  } else {
    AERROR << "Cannot find module " << module;
  }
}

HMIStatus HMIWorker::GetStatus() const {
  RLock rlock(status_mutex_);
  return status_;
}

void HMIWorker::SetupMode() {
  {
    WLock wlock(status_mutex_);
    status_.set_backend_shutdown(false);
  }
  for (const auto &iter : current_mode_.modules()) {
    {
      WLock wlock(status_mutex_);
      auto modules = status_.modules();
      if (!modules[iter.first]) {
        LockModule(iter.first, true);
      }
    }
    System(iter.second.start_command());
  }
}

void HMIWorker::ResetMode() {
  {
    // Clearing the module and lock will cause the backend to trigger automatic
    // shutdown.
    WLock wlock(status_mutex_);
    status_.set_backend_shutdown(true);
  }
  for (const auto &iter : current_mode_.modules()) {
    {
      WLock wlock(status_mutex_);
      auto modules = status_.modules();
      if (modules[iter.first]) {
        LockModule(iter.first, true);
      }
    }
    System(iter.second.stop_command());
  }
  record_count_ = 0;
}

void HMIWorker::StatusUpdateThreadLoop() {
  constexpr int kLoopIntervalMs = 200;
  while (!stop_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kLoopIntervalMs));
    UpdateComponentStatus();
    bool status_changed = false;
    {
      WLock wlock(status_mutex_);
      status_changed = status_changed_;
      status_changed_ = false;
    }
    // If status doesn't change, check if we reached update interval.
    if (!status_changed) {
      static double next_update_time = 0;
      const double now = Clock::NowInSeconds();
      if (now < next_update_time) {
        continue;
      }
      next_update_time = now + FLAGS_status_publish_interval;
    }

    // Trigger registered status change handlers.
    HMIStatus status = GetStatus();
    for (const auto handler : status_update_handlers_) {
      handler(status_changed, &status);
    }
  }
}

void HMIWorker::ResetComponentStatusTimer() {
  last_status_received_s_ = Clock::NowInSeconds();
  last_status_fingerprint_ = 0;
}

void HMIWorker::UpdateComponentStatus() {
  const double now = Clock::NowInSeconds();
  if (now - last_status_received_s_.load() > FLAGS_monitor_timeout_threshold) {
    if (!monitor_timed_out_) {
      WLock wlock(status_mutex_);

      const uint64_t now_ms = static_cast<uint64_t>(now * 2e3);
      static constexpr bool kIsReportable = true;
      SubmitDriveEvent(now_ms, "Monitor timed out", {"PROBLEM"}, kIsReportable);
      AWARN << "System fault. Auto disengage.";
      Trigger(HMIAction::DISENGAGE);

      for (auto &monitored_component :
           *status_.mutable_monitored_components()) {
        monitored_component.second.set_status(ComponentStatus::UNKNOWN);
        monitored_component.second.set_message(
            "Status not reported by Monitor.");
      }
      status_changed_ = true;
    }
    monitor_timed_out_ = true;
  } else {
    monitor_timed_out_ = false;
  }
}

void HMIWorker::GetScenarioResourcePath(std::string *scenario_resource_path) {
  CHECK_NOTNULL(scenario_resource_path);
  const std::string home = cyber::common::GetEnv("HOME");
  *scenario_resource_path = home + FLAGS_resource_scenario_path;
}

void HMIWorker::ChangeDynamicModel(const std::string &dynamic_model_name) {
  // To avoid toggle sim control and always choose simulation perfect control
  // {
  //   RLock rlock(status_mutex_);
  //   // Skip if mode doesn't actually change.
  //   if (status_.current_dynamic_model() == dynamic_model_name) {
  //     return;
  //   }
  // }
  if (dynamic_model_name.empty()) {
    AERROR << "Failed to change empty dynamic model!";
    return;
  }
  auto sim_control_manager = SimControlManager::Instance();
  if (!sim_control_manager->IsEnabled()) {
    AERROR << "Sim control is not enabled!";
    return;
  }

  if (!sim_control_manager->ChangeDynamicModel(dynamic_model_name)) {
    // badcase1：sim control is not enabled. badcase2：miss params
    // badcase3：change dynamic model is not registered. resolution：return with
    // no action,keep sim control not enabled or use original dynamic model!
    AERROR << "Failed to change dynamic model! Please check if the param is "
              "valid!";
    return;
  }
  std::string current_dynamic_model_name = dynamic_model_name;
  {
    WLock wlock(status_mutex_);
    status_.set_current_dynamic_model(dynamic_model_name);
    status_changed_ = true;
  }
  return;
}

bool HMIWorker::UpdateDynamicModelToStatus(
    const std::string &dynamic_model_name) {
  auto sim_control_manager = SimControlManager::Instance();
  if (!sim_control_manager->IsEnabled()) {
    AERROR << "Sim control is not enabled or missing dynamic model name "
              "param!";
    return false;
  }

  if (!sim_control_manager->AddDynamicModel(dynamic_model_name)) {
    AERROR << "Failed to add dynamic model to local dynamic model list for "
              "register failed!";
    return false;
  }
  {
    WLock wlock(status_mutex_);
    for (const auto &iter : status_.dynamic_models()) {
      if (iter == dynamic_model_name) {
        AERROR << "Do not need to add new dynamic model for is duplicate!";
        return true;
      }
    }
    status_.add_dynamic_models(dynamic_model_name);
    status_changed_ = true;
  }
  return true;
}

bool HMIWorker::LoadDynamicModels() {
  auto sim_control_manager = SimControlManager::Instance();
  nlohmann::json load_res;
  if (sim_control_manager->IsEnabled()) {
    load_res = sim_control_manager->LoadDynamicModels();
  } else {
    AERROR << "Sim control is not enabled!";
    return false;
  }
  if (!load_res.contains("result") || !load_res["result"]) {
    return false;
  }

  {
    WLock wlock(status_mutex_);
    auto dynamic_models = status_.mutable_dynamic_models();
    // clear old data
    for (auto iter = dynamic_models->begin(); iter != dynamic_models->end();) {
      iter = dynamic_models->erase(iter);
    }
    for (const auto &dynamic_model : load_res["loaded_dynamic_models"]) {
      status_.add_dynamic_models(dynamic_model);
    }
    status_changed_ = true;
  }
  return true;
}

void HMIWorker::DeleteScenarioSet(const std::string &scenario_set_id) {
  if (scenario_set_id.empty()) {
    return;
  }
  std::string directory_path;
  GetScenarioResourcePath(&directory_path);
  directory_path = directory_path + scenario_set_id;
  if (!cyber::common::PathExists(directory_path)) {
    AERROR << "Failed to find scenario_set!";
    return;
  }
  std::string command = "rm -fr " + directory_path;
  // use cyber::common::removeFiles do not support sub-directory
  // use rmdir do not support not empty directory
  if (std::system(command.data()) != 0) {
    AERROR << "Failed to delete scenario set directory for: "
           << std::strerror(errno);
    return;
  }
  return;
}

void HMIWorker::DeleteDynamicModel(const std::string &dynamic_model_name) {
  if (dynamic_model_name.empty()) {
    AERROR << "Invalid param:empty dynamic model name!";
    return;
  }
  {
    RLock rlock(status_mutex_);
    // do not allowed remove changed current dynamic model
    if (dynamic_model_name == status_.current_dynamic_model()) {
      AERROR << "Cannot delete current dynamic model!";
      return;
    }
    if (dynamic_model_name == FLAGS_sim_perfect_control) {
      AERROR << "Cannot delete default sim control:SimPerfectControl!";
      return;
    }
  }
  auto sim_control_manager = SimControlManager::Instance();
  if (!sim_control_manager->IsEnabled()) {
    AERROR << "Sim control is not enabled!";
    return;
  }
  if (!sim_control_manager->DeleteDynamicModel(dynamic_model_name)) {
    // badcase1: sim control is not enable. badcase2: miss param
    // badcase3: Failed to delete file
    AERROR << "Failed to delete dynamic model!";
    return;
  }
  {
    WLock wlock(status_mutex_);
    auto iter = status_.dynamic_models().begin();
    while (iter != status_.dynamic_models().end()) {
      if (*iter == dynamic_model_name) {
        break;
      }
      iter++;
    }
    if (iter != status_.dynamic_models().end()) {
      status_.mutable_dynamic_models()->erase(iter);
      status_changed_ = true;
    } else {
      AWARN << "Can not find dynamic model to delete!";
    }
  }
  return;
}

void HMIWorker::GetRecordPath(std::string *record_path) {
  CHECK_NOTNULL(record_path);
  const std::string home = cyber::common::GetEnv("HOME");
  *record_path = home + FLAGS_resource_record_path;
}

bool HMIWorker::handlePlayRecordProcess(const std::string &action_type) {
  std::string record_id;
  PlayRecordStatus current_play_record_status;
  PlayRecordStatus expected_play_record_status;
  PlayRecordStatus reasonable_play_record_status;
  {
    RLock rlock(status_mutex_);
    auto record_status = status_.current_record_status();
    record_id = record_status.current_record_id();
    current_play_record_status = record_status.play_record_status();
    if (record_id.empty()) {
      AERROR << "Failed to pause record for record has not been selected"
                "or process is not exists!";
      return false;
    }
    if (!RecordIsLoaded(record_id)) {
      AERROR << "Failed to pause record for record has not been loaded!";
      return false;
    }
  }
  if (action_type.compare("continue") == 0) {
    expected_play_record_status = PlayRecordStatus::RUNNING;
    reasonable_play_record_status = PlayRecordStatus::PAUSED;
  } else if (action_type.compare("pause") == 0) {
    expected_play_record_status = PlayRecordStatus::PAUSED;
    reasonable_play_record_status = PlayRecordStatus::RUNNING;
  } else {
    AERROR << "Invalid action type for play record.";
    return false;
  }
  if (reasonable_play_record_status != current_play_record_status) {
    return false;
  }
  auto *record_player_factory = RecordPlayerFactory::Instance();
  auto player_ptr = record_player_factory->GetRecordPlayer(record_id);
  if (!player_ptr) {
    AERROR << "Failed to get record player.";
    return false;
  }
  player_ptr->HandleNohupThreadStatus();

  {
    WLock wlock(status_mutex_);
    status_.mutable_current_record_status()->set_play_record_status(
        expected_play_record_status);
    status_changed_ = true;
  }
  record_player_factory->IncreaseRecordPriority(record_id);
  return true;
}

bool HMIWorker::RePlayRecord() {
  std::string record_id;
  {
    RLock rlock(status_mutex_);
    auto record_status = status_.current_record_status();
    record_id = record_status.current_record_id();
    if (record_status.current_record_id().empty()) {
      AERROR << "Failed to play record for record has not been selected!";
      return false;
    }
    if (!RecordIsLoaded(record_id)) {
      AERROR << "Failed to play record for record has not been loaded!";
      return false;
    }
    if (record_status.play_record_status() == PlayRecordStatus::RUNNING) {
      AERROR << "Record has been played,ignore dumplicate request.";
      return false;
    }
  }
  std::string record_path;
  GetRecordPath(&record_path);
  record_path = record_path + record_id + ".record";

  if (!cyber::common::PathExists(record_path)) {
    AERROR << "Failed to find record!";
    return false;
  }
  // play the record
  auto *record_player_factory = RecordPlayerFactory::Instance();
  auto player_ptr = record_player_factory->GetRecordPlayer(record_id);
  bool play_record_res = (player_ptr != nullptr);
  PlayRecordStatus play_record_status;
  if (!play_record_res) {
    AERROR << "Failed to get record related player";
    play_record_status = PlayRecordStatus::CLOSED;
  } else {
    player_ptr->PreloadPlayRecord();
    player_ptr->NohupPlayRecord();
    // get channel msgs for frontend after start play record
    callback_api_("GetDataHandlerConf", {});
    play_record_status = PlayRecordStatus::RUNNING;
  }
  {
    WLock wlock(status_mutex_);
    auto record_status = status_.mutable_current_record_status();
    record_status->set_play_record_status(play_record_status);
    status_changed_ = true;
  }
  record_player_factory->IncreaseRecordPriority(record_id);
  return play_record_res;
}

bool HMIWorker::ResetRecordProgress(const double &progress) {
  std::string record_id;
  double total_time_s;
  PlayRecordStatus last_record_status;
  {
    RLock rlock(status_mutex_);
    auto record_status = status_.current_record_status();
    record_id = record_status.current_record_id();
    last_record_status = record_status.play_record_status();
    if (record_status.current_record_id().empty()) {
      AERROR << "Failed to reset record progress for record has not been "
                "selected!";
      return false;
    }
    if (!RecordIsLoaded(record_id)) {
      AERROR << "Failed to reset record progress for record has not been "
                "loaded!";
      return false;
    }
    total_time_s = status_.records().at(record_id).total_time_s();
  }
  AERROR << "total :  " << total_time_s;
  if (progress > total_time_s) {
    AERROR << "Failed to reset record progress for progress exceeds the "
              "record's total time.";
    return false;
  }
  auto *record_player_factory = RecordPlayerFactory::Instance();
  auto player_ptr = record_player_factory->GetRecordPlayer(record_id);
  if (!player_ptr) {
    AERROR << "Failed to get record player.";
    return false;
  }
  player_ptr->Reset();
  player_ptr->PreloadPlayRecord(
      progress, (last_record_status != PlayRecordStatus::RUNNING));
  player_ptr->NohupPlayRecord();
  // When a record is selected and the progress is reset; we consider
  // the record to be stuck at the current progress in a paused state
  // to keep logic consistent
  if (last_record_status == PlayRecordStatus::CLOSED) {
    last_record_status = PlayRecordStatus::PAUSED;
  }
  // get channel msgs for frontend after start play record
  callback_api_("GetDataHandlerConf", {});
  {
    WLock wlock(status_mutex_);
    auto record_status = status_.mutable_current_record_status();
    record_status->set_play_record_status(last_record_status);
    record_status->set_curr_time_s(progress);
    status_changed_ = true;
  }
  record_player_factory->IncreaseRecordPriority(record_id);
  return true;
}

void HMIWorker::StopRecordPlay(const std::string &record_id) {
  std::string curr_record_id = "";
  {
    RLock rlock(status_mutex_);
    curr_record_id = status_.current_record_status().current_record_id();
  }
  if (!record_id.empty()) {
    curr_record_id = record_id;
  }
  if (!RecordIsLoaded(curr_record_id)) {
    AERROR << "Failed to stop record player under unloaded status!";
    return;
  }
  auto *record_player_factory = RecordPlayerFactory::Instance();
  auto player_ptr = record_player_factory->GetRecordPlayer(curr_record_id);
  if (!player_ptr) {
    AERROR << "Failed to get record player to reset.";
    return;
  }
  player_ptr->Reset();
  {
    WLock wlock(status_mutex_);
    auto record_status = status_.mutable_current_record_status();
    record_status->set_play_record_status(PlayRecordStatus::CLOSED);
    record_status->clear_curr_time_s();
  }
  status_changed_ = true;
}

bool HMIWorker::RecordIsLoaded(const std::string &record_id) {
  {
    RLock rlock(status_mutex_);
    auto iter = status_.records().find(record_id);
    return iter != status_.records().end() &&
           (iter->second.load_record_status() == LoadRecordStatus::LOADED);
  }
}

void HMIWorker::ChangeRecord(const std::string &record_id) {
  std::string last_record_id;
  {
    RLock rlock(status_mutex_);
    auto status_records = status_.mutable_records();
    auto iter = status_records->find(record_id);
    if (iter == status_records->end()) {
      AERROR << "Cannot change to unknown record!";
      return;
    }
    if (!RecordIsLoaded(record_id)) {
      AERROR << "Cannot change to unloaded record, load this record before!";
      return;
    }
    last_record_id = status_.current_record_status().current_record_id();
  }
  if (!last_record_id.empty()) {
    StopRecordPlay(last_record_id);
  }
  // clear last channel msgs info for frontend after change record
  callback_api_("ClearDataHandlerConfChannelMsgs", {});
  WLock wlock(status_mutex_);
  status_.mutable_current_record_status()->set_current_record_id(record_id);
  auto *record_player_factory = RecordPlayerFactory::Instance();
  record_player_factory->SetCurrentRecord(record_id);
  record_player_factory->IncreaseRecordPriority(record_id);
  status_changed_ = true;
  return;
}

void HMIWorker::ClearInvalidResourceUnderChangeOperation(
    const HMIModeOperation operation) {
  ClearInvalidRecordStatus(operation);
  HMIModeOperation old_operation;
  {
    RLock rlock(status_mutex_);
    old_operation = status_.current_operation();
  }
    // dynamic model resource clear and sim control manager start/stop
    // sim control status changed when operation change
    auto sim_control_manager = SimControlManager::Instance();
    auto iter = std::find(OperationBasedOnSimControl.begin(),
                          OperationBasedOnSimControl.end(), operation);
    if (iter != OperationBasedOnSimControl.end()) {
      sim_control_manager->Start();
    } else {
      sim_control_manager->Stop();
      // clear DynamicModel related info
      {
        WLock wlock(status_mutex_);
        status_.set_current_dynamic_model("");
        status_.clear_dynamic_models();
        status_changed_ = true;
      }
    }
    if (old_operation == HMIModeOperation::Record) {
      // change from record need to clear record info.
      ClearRecordInfo();
    }
    if (old_operation == HMIModeOperation::Waypoint_Follow) {
      // clear selected rtk record
      ClearRtkRecordInfo();
    }
}

void HMIWorker::ChangeOperation(const std::string &operation_str) {
  HMIModeOperation operation;
  if (!HMIModeOperation_Parse(operation_str, &operation)) {
    AERROR << "Invalid HMI operation string: " << operation_str;
    return;
  }
  {
    RLock rlock(status_mutex_);
    if (status_.current_mode().empty()) {
      AERROR << "Please select mode!";
      return;
    }
    auto status_operations = status_.operations();
    auto iter = status_operations.begin();
    for (; iter != status_operations.end(); iter++) {
      if (*iter == operation) {
        break;
      }
    }
    if (iter == status_operations.end()) {
      AERROR << "Cannot change to unknown operation!";
      return;
    }
  }
  ClearInvalidResourceUnderChangeOperation(operation);
  {
    WLock wlock(status_mutex_);
    status_.set_current_operation(operation);
    status_changed_ = true;
  }
  return;
}

bool HMIWorker::ReadRecordInfo(const std::string &file,
                                double *total_time_s) const {
  cyber::record::RecordFileReader file_reader;
  if (!file_reader.Open(file)) {
    AERROR << "open record file error. file: " << file;
    return false;
  }
  cyber::proto::Header hdr = file_reader.GetHeader();
  if (!hdr.is_complete()) {
    AERROR << "record is not complete, can not be used. file: " << file;
    return false;
  }
  auto begin_time_s = static_cast<double>(hdr.begin_time()) / 1e9;
  auto end_time_s = static_cast<double>(hdr.end_time()) / 1e9;
  double loop_time_s = end_time_s - begin_time_s;
  *total_time_s = std::round(loop_time_s * 1e2) / 1e2;
  return true;
}

bool HMIWorker::UpdateMapToStatus(const std::string &map_tar_name) {
  if (map_tar_name.empty()) {
    ReloadMaps();
    return true;
  }
  std::string map_dir = FLAGS_maps_data_path + "/";
  std::string map_name_prefix;
  int index = map_tar_name.rfind(".tar.xz");
  if (index != -1 && map_tar_name[0] != '.') {
    map_name_prefix = map_tar_name.substr(0, index);
    map_dir = map_dir + map_name_prefix;
  } else {
    AERROR << "The map name does not meet the standard!" << map_tar_name;
    return false;
  }
  if (!cyber::common::PathExists(map_dir)) {
    AERROR << "Failed to find maps!";
    return false;
  }
  map_name_prefix = util::HMIUtil::TitleCase(map_name_prefix);
  {
    WLock wlock(status_mutex_);
    auto iter = status_.maps().begin();
    for (; iter != status_.maps().end(); iter++) {
      if (*iter == map_name_prefix) {
        break;
      }
    }
    if (iter != status_.maps().end()) {
      return true;
    }
    status_.add_maps(map_name_prefix);
    status_changed_ = true;
  }
  return true;
}

bool HMIWorker::LoadRecordAndChangeStatus(const std::string &record_name) {
    std::string record_file_path;
    {
      RLock rlock(status_mutex_);
      auto &status_records = status_.records();
      auto iter = status_records.find(record_name);
      if (iter == status_records.end()) {
        AERROR << "Cannot load unknown record!";
        return false;
      }
      if (RecordIsLoaded(record_name)) {
        AERROR << "Cannot load already loaded record.";
        return false;
      }
      if (iter->second.record_file_path().empty()) {
        AERROR << "Cannot load record without record file path!";
        return false;
      }
      record_file_path = iter->second.record_file_path();
    }
    {
      WLock wlock(status_mutex_);
      auto status_records = status_.mutable_records();
      (*status_records)[record_name].set_load_record_status(
          LoadRecordStatus::LOADING);
    }
    double total_time_s;
    if (LoadRecord(record_name, record_file_path, &total_time_s)) {
      {
        WLock wlock(status_mutex_);
        auto status_records = status_.mutable_records();
        (*status_records)[record_name].set_load_record_status(
            LoadRecordStatus::LOADED);
        (*status_records)[record_name].set_total_time_s(total_time_s);
      }
      RecordPlayerFactory::Instance()->IncreaseRecordPriority(record_name);
    } else {
      {
        WLock wlock(status_mutex_);
        auto status_records = status_.mutable_records();
        (*status_records)[record_name].set_load_record_status(
            LoadRecordStatus::NOT_LOAD);
      }
    }
    return true;
}

bool HMIWorker::LoadRecord(const std::string &record_name,
                           const std::string &record_file_path,
                           double *total_time_s) {
  if (RecordIsLoaded(record_name)) {
    AERROR << "Record is loaded,no need to load";
    return false;
  }
  // check if record player factory can continue load
  auto record_player_factory = RecordPlayerFactory::Instance();
  if (!record_player_factory->EnableContinueLoad()) {
    // can not load record,need to remove lru record
    std::string remove_record_id;
    if (record_player_factory->RemoveLRURecord(&remove_record_id)) {
      // remove successfully, change removed record status
      {
        WLock wlock(status_mutex_);
        auto status_records = status_.mutable_records();
        (*status_records)[remove_record_id].set_load_record_status(
            LoadRecordStatus::NOT_LOAD);
        (*status_records)[remove_record_id].clear_total_time_s();
      }
    }
  }
  if (!ReadRecordInfo(record_file_path, total_time_s)) {
    return false;
  }
  if (!record_player_factory->RegisterRecordPlayer(record_name,
                                                   record_file_path)) {
    return false;
  }
  return true;
}

bool HMIWorker::LoadRecords() {
  std::string directory_path;
  auto *record_player_factory = RecordPlayerFactory::Instance();
  GetRecordPath(&directory_path);
  if (!cyber::common::PathExists(directory_path)) {
    AERROR << "Failed to find records!";
    return false;
  }
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open record directory" << directory_path;
    return false;
  }
  struct dirent *file;
  google::protobuf::Map<std::string, LoadRecordInfo> new_records;
  while ((file = readdir(directory)) != nullptr) {
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }
    if (file->d_type == DT_DIR) {
      continue;
    }
    const std::string record_id = file->d_name;
    const int index = record_id.rfind(".record");
    // Skip format that dv cannot parse: record not ending in record
    size_t record_suffix_length = 7;
    if (record_id.length() - index != record_suffix_length) {
        continue;
    }
    if (index != -1 && record_id[0] != '.') {
      const std::string local_record_resource = record_id.substr(0, index);
      const std::string record_file_path = directory_path + "/" + record_id;
      double total_time_s;
      // Already loaded, no need to change status
      if (RecordIsLoaded(local_record_resource)) {
        continue;
      }
      // already reach max preload num,no load only get basic info
      if (!record_player_factory->EnableContinuePreload()) {
        new_records[local_record_resource].set_load_record_status(
            LoadRecordStatus::NOT_LOAD);
        new_records[local_record_resource].set_record_file_path(
            record_file_path);
        continue;
      }
      // failed to load record，continue
      if (!LoadRecord(local_record_resource, record_file_path, &total_time_s)) {
        continue;
      }
      // Successfully load record, change hmi status
      new_records[local_record_resource].set_total_time_s(total_time_s);
      new_records[local_record_resource].set_load_record_status(
          LoadRecordStatus::LOADED);
      new_records[local_record_resource].set_record_file_path(record_file_path);
    }
  }
  closedir(directory);
  {
    WLock wlock(status_mutex_);
    auto status_records = status_.mutable_records();
    for (auto iter = new_records.begin(); iter != new_records.end(); iter++) {
      (*status_records)[iter->first] = iter->second;
    }
    status_changed_ = true;
  }
  return true;
}

void HMIWorker::DeleteMap(const std::string &map_name) {
  std::string title_map_name = util::HMIUtil::TitleCase(map_name);
  if (map_name.empty()) {
    return;
  }
  std::string map_dir = FLAGS_maps_data_path + "/";
  std::string map_abs_path = map_dir + map_name;
  if (!cyber::common::PathExists(map_abs_path)) {
    AERROR << "Failed to get map path: " << map_abs_path;
    return;
  }
  {
    WLock wlock(status_mutex_);
    auto iter = status_.maps().begin();
    for (; iter != status_.maps().end(); iter++) {
      if (*iter == title_map_name) {
        break;
      }
    }
    if (iter == status_.maps().end()) {
      AERROR << "Faile to find map name";
      return;
    }
    if (status_.current_map() == title_map_name) {
      AERROR << "Cann't delete current selected map";
      return;
    }
    status_.mutable_maps()->erase(iter);
    status_changed_ = true;
  }

  std::string cmd = "rm -rf " + map_abs_path;
  if (std::system(cmd.data()) != 0) {
    AERROR << "Failed to delete map for: " << std::strerror(errno);
    return;
  }
  std::string tar_abs_path = map_dir + map_name + ".tar.xz";

  if (cyber::common::PathExists(tar_abs_path)) {
    cmd = "rm -rf " + tar_abs_path;
    if (std::system(cmd.data()) != 0) {
      AERROR << "Failed to delete map tar file for: " << std::strerror(errno);
    }
  }
  return;
}

void HMIWorker::DeleteRecord(const std::string &record_id) {
  if (record_id.empty()) {
    return;
  }
  std::string record_path;
  GetRecordPath(&record_path);
  const std::string record_abs_path = record_path + record_id + ".record";
  if (!cyber::common::PathExists(record_abs_path)) {
    AERROR << "Failed to get record path: " << record_abs_path;
    return;
  }
  // find the delete record if exist and judge the record whether playing now
  {
    RLock rlock(status_mutex_);
    auto &status_records = status_.records();
    if (status_records.find(record_id) == status_records.end()) {
      AERROR << "Failed to find record id: " << record_id;
      return;
    }
    if (record_id == status_.current_record_status().current_record_id()) {
      AERROR << "Cann't delete current selected record";
      return;
    }
  }
  // unregister record player from player factory
  auto *record_player_factory = RecordPlayerFactory::Instance();
  record_player_factory->UnregisterRecordPlayer(record_id);
  {
    WLock wlock(status_mutex_);
    status_.mutable_records()->erase(record_id);
    status_changed_ = true;
  }

  // delete record from disk
  std::string command = "rm -rf " + record_abs_path;
  if (std::system(command.data()) != 0) {
    AERROR << "Failed to delete record for: " << std::strerror(errno);
    return;
  }
  // record may unzip from same name tar xz file,delete record file also need
  // to delete record tar xz file
  const std::string tar_abs_path = record_path + record_id + ".tar.xz";
  if (cyber::common::PathExists(tar_abs_path)) {
    command = "rm -rf " + tar_abs_path;
    if (std::system(command.data()) != 0) {
      AERROR << "Failed to delete record tar file for: "
             << std::strerror(errno);
      return;
    }
  }
  return;
}

void HMIWorker::ReloadMaps() {
  WLock wlock(status_mutex_);
  config_.mutable_maps()->clear();
  *(config_.mutable_maps()) =
      util::HMIUtil::ListDirAsDict(FLAGS_maps_data_path);
  status_.clear_maps();
  for (const auto &map : config_.maps()) {
    status_.add_maps(map.first);
  }
  if (status_.current_map() != "") {
    // if current map is not empty, check whether it exists in config
    if (config_.maps().find(status_.current_map()) == config_.maps().end()) {
      // if not, set current map to empty string
      AINFO << "Current map is not in status. Reset it.";
      status_.set_current_map("");
    }
  }
  status_changed_ = true;
  return;
}

bool HMIWorker::ReloadVehicles() {
  HMIConfig config =
      util::HMIUtil::LoadConfig(FLAGS_dv_plus_hmi_modes_config_path);
  std::string msg;
  config.SerializeToString(&msg);

  WLock wlock(status_mutex_);
  config_.ParseFromString(msg);
  // status_.clear_modes();
  // status_.clear_maps();
  status_.clear_vehicles();
  // InitStatus();
  // Populate vehicles and current_vehicle.
  for (const auto &vehicle : config_.vehicles()) {
    status_.add_vehicles(vehicle.first);
  }
  status_changed_ = true;
  return true;
}

void HMIWorker::UpdateCameraSensorChannelToStatus(
    const std::string &channel_name) {
  {
    WLock wlock(status_mutex_);
    if (status_.current_camera_sensor_channel() == channel_name) {
      AINFO << "Input channel name is current camera sensor channel";
      return;
    }
    status_.set_current_camera_sensor_channel(channel_name);
    status_changed_ = true;
  }
}

void HMIWorker::UpdatePointCloudChannelToStatus(
    const std::string &channel_name) {
  {
    WLock wlock(status_mutex_);
    if (status_.current_point_cloud_channel() == channel_name) {
      AINFO << "Input channel name is current camera sensor channel";
      return;
    }
    status_.set_current_point_cloud_channel(channel_name);
    status_changed_ = true;
  }
}

void HMIWorker::DeleteVehicleConfig(const std::string &vehicle_name) {
  if (vehicle_name.empty()) {
    return;
  }
  const std::string *vehicle_dir =
      FindOrNull(config_.vehicles(), vehicle_name);
  if (vehicle_dir == nullptr) {
    AERROR << "Unknow vehicle name" << vehicle_name;
    return;
  }
  {
    RLock rlock(status_mutex_);
    if (status_.current_vehicle() == vehicle_name) {
      AERROR << "The deleted vehicle profile is the one in use.";
      return;
    }
  }
  if (!cyber::common::DeleteFile(*vehicle_dir)) {
    AERROR << "Delete vehicle profile [" << vehicle_name << "] failed!";
    return;
  }
  if (!ReloadVehicles()) {
    AERROR << "Update vehicle profile [" << vehicle_name << "] failed!";
    return;
  }
}

void HMIWorker::DeleteV2xConfig(const std::string &vehicle_name) {
  if (vehicle_name.empty()) {
    return;
  }
  const std::string *vehicle_dir =
      FindOrNull(config_.vehicles(), util::HMIUtil::TitleCase(vehicle_name));
  if (vehicle_dir == nullptr) {
    AERROR << "Unknow vehicle name " << vehicle_name;
    return;
  }
  const std::string v2x_dir = *vehicle_dir + "/v2x_conf";
  if (!cyber::common::PathExists(v2x_dir)) {
    AINFO << "The directory does not exist or the directory has been deleted";
  }
  if (!cyber::common::DeleteFile(v2x_dir)) {
    AERROR << "Delete v2x config [" << vehicle_name << "/v2x_conf] failed!";
    return;
  }
}

bool HMIWorker::StartDataRecorder() {
  std::string start_cmd =
      "/apollo/scripts/record_bag.py --start --all --dreamview "
      "--default_name " +
      FLAGS_data_record_default_name;
  int ret = std::system(start_cmd.data());
  if (ret == 0) {
    auto *monitors = FuelMonitorManager::Instance()->GetCurrentMonitors();
    if (monitors != nullptr) {
      auto iter = monitors->find(FLAGS_data_collection_monitor_name);
      if (iter != monitors->end()) {
        auto *data_collection_monitor = iter->second.get();
        if (data_collection_monitor->IsEnabled() && record_count_ == 0) {
          data_collection_monitor->Restart();
        }
      }
      ++record_count_;
    }
    return true;
  } else {
    AERROR << "Failed to start the cyber_recorder process";
    return false;
  }
}

bool HMIWorker::StopDataRecorder() {
  std::string stop_cmd = "/apollo/scripts/record_bag.py --stop";
  int ret = std::system(stop_cmd.data());
  if (ret == 0) {
    return true;
  } else {
    AERROR << "Failed to stop the cyber_recorder process";
    return false;
  }
}

int HMIWorker::SaveDataRecorder(const std::string &new_name) {
  std::string data_record_default_path =
      std::string(cyber::common::GetEnv("HOME", "/home/apollo")) +
      "/.apollo/resources/records/" + FLAGS_data_record_default_name;
  if (!cyber::common::PathExists(data_record_default_path)) {
    AERROR << "Failed to save the record, the dreamview recording record does "
              "not exist, please record through dreamview.";
    return -2;
  }
  std::string save_cmd = "/apollo/scripts/record_bag.py --default_name " +
                         FLAGS_data_record_default_name + " --rename " +
                         new_name;
  int ret = std::system(save_cmd.data());
  if (ret == 0) {
    LoadRecords();
    return 1;
  } else {
    AERROR << "Failed to save the record, a file with the same name exists";
    return -1;
  }
}

bool HMIWorker::DeleteDataRecorder() {
  std::string delete_cmd =
      "/apollo/scripts/record_bag.py --delete --default_name " +
      FLAGS_data_record_default_name;
  int ret = std::system(delete_cmd.data());
  if (ret == 0) {
    return true;
  } else {
    AERROR << "Failed to delete the record";
    return false;
  }
}

void HMIWorker::ClearRecordInfo() {
  std::string last_record_id;
  {
    RLock rlock(status_mutex_);
    if (status_.current_operation() != HMIModeOperation::Record) {
      return;
    }
    last_record_id = status_.current_record_status().current_record_id();
  }
  if (!last_record_id.empty()) {
    StopRecordPlay(last_record_id);
  }
  // clear last channel msgs info for frontend after change record
  callback_api_("ClearDataHandlerConfChannelMsgs", {});
  WLock wlock(status_mutex_);
  status_.mutable_current_record_status()->clear_current_record_id();
  status_changed_ = true;
  return;
}

void HMIWorker::ClearRtkRecordInfo() {
  std::string last_rtk_record_id;
  {
    RLock rlock(status_mutex_);
    if (status_.current_operation() != HMIModeOperation::Waypoint_Follow) {
      return;
    }
    last_rtk_record_id = status_.current_rtk_record_id();
  }
  if (!last_rtk_record_id.empty()) {
    StopPlayRtkRecorder();
  }
  {
    WLock wlock(status_mutex_);
    status_.set_current_rtk_record_id("");
    status_changed_ = true;
  }
  return;
}

void HMIWorker::AddExpectedModules(const HMIAction& action) {
  WLock wlock(status_mutex_);
  int expected_modules = 1;
  if (action == HMIAction::SETUP_MODE) {
    expected_modules = status_.modules_size();
  }
  status_.set_expected_modules(status_.expected_modules() + expected_modules);
  status_changed_ = true;
}

void HMIWorker::OnTimer(const double& overtime_time) {
  if (monitor_reader_ != nullptr) {
    auto delay_sec = monitor_reader_->GetDelaySec();
    if (delay_sec < 0 || delay_sec > overtime_time) {
      AERROR << "Running time error: monitor is not turned on!";
      {
        WLock wlock(status_mutex_);
        for (auto& iter : *status_.mutable_modules_lock()) {
          iter.second = false;
        }
      }
    }
  }
  {
    // When there is a problem with the module and it cannot be opened or
    // closed, unlock the loading lock state
    WLock wlock(status_mutex_);
    auto modules = status_.mutable_modules();
    auto modules_lock = status_.mutable_modules_lock();
    for (const auto &iter : current_mode_.modules()) {
      if (previous_modules_lock_[iter.first] && (*modules_lock)[iter.first] &&
          !isProcessRunning(iter.second.start_command())) {
        (*modules)[iter.first] = false;
        LockModule(iter.first, false);
      }
      previous_modules_lock_[iter.first] = (*modules_lock)[iter.first];
    }
  }
}

void HMIWorker::LockModule(const std::string& module, const bool& lock_flag) {
  auto modules_lock = status_.mutable_modules_lock();
  (*modules_lock)[module] = lock_flag;
}

bool HMIWorker::AddOrModifyObjectToDB(const std::string& key,
                                      const std::string& value) {
  return KVDB::Put(key, value);
}

bool HMIWorker::DeleteObjectToDB(const std::string& key) {
  return KVDB::Delete(key);
}

std::string HMIWorker::GetObjectFromDB(const std::string& key) {
  return KVDB::Get(key).value_or("");
}

std::vector<std::pair<std::string, std::string>>
HMIWorker::GetTuplesWithTypeFromDB(const std::string& type) {
  return KVDB::GetWithStart(type);
}

bool HMIWorker::StartTerminal() {
  return std::system(FLAGS_terminal_start_cmd.data()) == 0;
}

void HMIWorker::GetRtkRecordPath(std::string *record_path) {
  CHECK_NOTNULL(record_path);
  *record_path = FLAGS_resource_rtk_record_path;
}

bool HMIWorker::LoadRtkRecords() {
  {
    WLock wLock(status_mutex_);
    status_.clear_rtk_records();
  }
  std::string directory_path;
  GetRtkRecordPath(&directory_path);
  if (!cyber::common::PathExists(directory_path)) {
    AERROR << "Failed to find rtk records!";
    return false;
  }
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open rtk record directory" << directory_path;
    return false;
  }
  struct dirent *file;
  std::map<std::string, double> new_records;
  while ((file = readdir(directory)) != nullptr) {
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }
    if (file->d_type == DT_DIR) {
      continue;
    }
    const std::string record_id = file->d_name;
    const int index = record_id.rfind(".csv");
    if (index != -1 && record_id[0] != '.') {
      const std::string local_record_resource = record_id.substr(0, index);
      {
        WLock wLock(status_mutex_);
        status_.add_rtk_records(local_record_resource);
      }
    }
  }
  closedir(directory);
  {
    WLock wlock(status_mutex_);
    status_changed_ = true;
  }
  return true;
}

bool HMIWorker::StartRtkDataRecorder() {
  std::string start_cmd = "nohup /apollo/scripts/rtk_recorder.sh start &";
  int ret = std::system(start_cmd.data());
  if (ret == 0) {
    return true;
  } else {
    AERROR << "Failed to start the rtk_recorder process";
    return false;
  }
}

bool HMIWorker::StopRtkDataRecorder() {
  std::string stop_cmd = "/apollo/scripts/rtk_recorder.sh stop";
  int ret = std::system(stop_cmd.data());
  if (ret == 9) {
    return true;
  } else {
    AERROR << "Failed to stop the rtk_recorder process";
    return false;
  }
}

Json HMIWorker::StartPlayRtkRecorder() {
  Json result;
  if (!ChangeDrivingMode(Chassis::COMPLETE_AUTO_DRIVE)) {
    AERROR << "Failed to play rtk: Failed to enter auto drive.";
    result["error"] = "Failed to enter auto drive";
    result["isOk"] = false;
    return result;
  }
  std::string record_id;
  {
    RLock rlock(status_mutex_);
    record_id = status_.current_rtk_record_id();
  }
  std::string start_cmd =
      "nohup /apollo/scripts/rtk_player.sh start " + record_id + " &";
  int ret = std::system(start_cmd.data());
  if (ret == 0) {
    AINFO << "Start the rtk_recorder process Successful.";
    result["isOk"] = true;
  } else {
    AERROR << "Failed to play rtk: Failed to start the rtk_recorder process.";
    result["error"] = "Failed to start the rtk_recorder process";
    result["isOk"] = false;
  }
  return result;
}

bool HMIWorker::StopPlayRtkRecorder() {
  std::string stop_cmd = "/apollo/scripts/rtk_player.sh stop";
  int ret = std::system(stop_cmd.data());
  if (ret == 9) {
    return true;
  } else {
    AERROR << "Failed to stop the rtk_recorder process";
    return false;
  }
}

int HMIWorker::SaveRtkDataRecorder(const std::string &new_name) {
  std::string new_rtk_record_file =
      FLAGS_default_rtk_record_path + new_name + ".csv";
  if (cyber::common::PathExists(new_rtk_record_file)) {
    AERROR << "Failed to save the ret record, a file with the same name exists";
    return -1;
  }
  if (std::rename(FLAGS_default_rtk_record_file.data(),
                  new_rtk_record_file.data()) == 0) {
    UpdateRtkRecordToStatus(new_name);
    return 1;
  } else {
    AERROR << "Failed to save the ret record, save command execution failed";
    return -3;
  }
}

bool HMIWorker::DeleteRtkDataRecorder() {
  if (!cyber::common::PathExists(FLAGS_default_rtk_record_file)) {
    AERROR << "Failed to delete the ret record, file not exist";
    return false;
  }
  if (std::remove(FLAGS_default_rtk_record_file.data()) == 0) {
      return true;
    } else {
      AERROR << "Failed to delete the record, delete command execution failed";
      return false;
    }
  }

void HMIWorker::ChangeRtkRecord(const std::string &record_id) {
  if (!StopPlayRtkRecorder()) {
    AWARN << "Warning to change rtk record: Failed to stop the rtk_recorder "
             "process.";
  }
  std::string last_record_id;
  {
    RLock rlock(status_mutex_);
    last_record_id = status_.current_rtk_record_id();
  }
  if (last_record_id == record_id) {
    AERROR << "Failed to change rtk record: rtk record is same!";
    return;
  }
  WLock wlock(status_mutex_);
  status_.set_current_rtk_record_id(record_id);
  status_changed_ = true;
  return;
}

void HMIWorker::UpdateRtkRecordToStatus(const std::string &new_name) {
  WLock wlock(status_mutex_);
  status_.add_rtk_records(new_name);
  status_changed_ = true;
}

void HMIWorker::ClearInvalidRecordStatus(const HMIModeOperation &operation) {
  HMIModeOperation last_operation;
  {
    RLock rlock(status_mutex_);
    last_operation = status_.current_operation();
  }
  if (operation == last_operation) {
    return;
  } else if (last_operation == HMIModeOperation::Waypoint_Follow) {
    StopRtkDataRecorder();
    DeleteRtkDataRecorder();
  } else if (operation == HMIModeOperation::Waypoint_Follow) {
    StopDataRecorder();
    DeleteDataRecorder();
  }
}

bool HMIWorker::isProcessRunning(const std::string &process_name) {
  std::stringstream commandStream;
  commandStream << "pgrep -f " << process_name;
  std::string command = commandStream.str();

  FILE *fp = popen(command.c_str(), "r");
  if (fp) {
    char result[128];
    if (fgets(result, sizeof(result), fp) != nullptr) {
      AINFO << process_name << " is running";
      pclose(fp);
      return true;
    } else {
      AINFO << process_name << " is not running";
    }
    pclose(fp);
  }
  return false;
}

bool HMIWorker::PackageExist(const std::string& package_name) {
  std::string package_meta_info_path =
      FLAGS_apollo_package_meta_info_path_prefix + package_name +
      "/cyberfile.xml";
  AINFO << "package_meta_info_path: " << package_meta_info_path;
  return (cyber::common::PathExists(package_meta_info_path));
}

}  // namespace dreamview
}  // namespace apollo
