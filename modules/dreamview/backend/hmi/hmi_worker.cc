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

#include "modules/dreamview/backend/hmi/hmi_worker.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"

#include "cyber/proto/dag_conf.pb.h"
#include "modules/common_msgs/monitor_msgs/system_status.pb.h"
#include "modules/dreamview/proto/scenario.pb.h"

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
#include "modules/dreamview/backend/common/util/hmi_util.h"

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
using apollo::dreamview::SimTicket;
using apollo::dreamview::UserAdsGroup;
using apollo::external_command::ActionCommand;
using apollo::external_command::LaneFollowCommand;
using apollo::external_command::CommandStatus;
using apollo::localization::LocalizationEstimate;
using apollo::monitor::ComponentStatus;
using apollo::monitor::SystemStatus;
using google::protobuf::util::JsonStringToMessage;
using RLock = boost::shared_lock<boost::shared_mutex>;
using WLock = boost::unique_lock<boost::shared_mutex>;
using Json = nlohmann::json;

constexpr char kNavigationModeName[] = "Navigation";

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

HMIWorker::HMIWorker(const std::shared_ptr<Node> &node)
    : config_(util::HMIUtil::LoadConfig(FLAGS_dv_hmi_modes_config_path)),
      node_(node) {
  InitStatus();
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
      current_vehicle_path + "/dreamview_conf/hmi_modes/" +
      mode_file_name;
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
  lane_follow_command_client_ =
      node_->CreateClient<LaneFollowCommand, CommandStatus>(
          FLAGS_lane_follow_command_topic);
  audio_event_writer_ =
      node_->CreateWriter<AudioEvent>(FLAGS_audio_event_topic);
  drive_event_writer_ =
      node_->CreateWriter<DriveEvent>(FLAGS_drive_event_topic);

  node_->CreateReader<SystemStatus>(
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
            auto *status = FindOrNull(system_status->hmi_modules(), iter.first);
            iter.second =
                status != nullptr && status->status() == ComponentStatus::OK;
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

        // Check if the status is changed.
        const size_t new_fingerprint =
            apollo::common::util::MessageFingerprint(status_);
        if (last_status_fingerprint_ != new_fingerprint) {
          status_changed_ = true;
          last_status_fingerprint_ = new_fingerprint;
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
      break;
    case HMIAction::ENTER_AUTO_MODE:
      return ChangeDrivingMode(Chassis::COMPLETE_AUTO_DRIVE);
    case HMIAction::DISENGAGE:
      return ChangeDrivingMode(Chassis::COMPLETE_MANUAL);
    case HMIAction::RESET_MODE:
      ResetMode();
      break;
    case HMIAction::LOAD_SCENARIOS:
      LoadScenarios();
      break;
    case HMIAction::LOAD_DYNAMIC_MODELS:
      LoadDynamicModels();
      break;
    case HMIAction::LOAD_RECORDS:
      LoadRecords();
      break;
    case HMIAction::STOP_RECORD:
      StopRecordPlay();
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
  switch (action) {
    case HMIAction::CHANGE_MODE:
      ChangeMode(value);
      break;
    case HMIAction::CHANGE_MAP:
      ChangeMap(value);
      break;
    case HMIAction::CHANGE_VEHICLE:
      ChangeVehicle(value);
      break;
    case HMIAction::START_MODULE:
      StartModule(value);
      break;
    case HMIAction::STOP_MODULE:
      StopModule(value);
      break;
    case HMIAction::CHANGE_SCENARIO_SET:
      ChangeScenarioSet(value);
      break;
    case HMIAction::DELETE_SCENARIO_SET:
      DeleteScenarioSet(value);
      ChangeScenario("");
      break;
    case HMIAction::CHANGE_SCENARIO:
      ChangeScenario(value);
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
    default:
      AERROR << "HMIAction not implemented, yet!";
      return false;
  }
  return true;
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

bool HMIWorker::ChangeMap(const std::string &map_name,
                          const bool restart_dynamic_model) {
  if (status_.current_map() == map_name && restart_dynamic_model == false) {
    return true;
  }
  const std::string *map_dir = FindOrNull(config_.maps(), map_name);
  if (map_dir == nullptr) {
    AERROR << "Unknown map " << map_name;
    return false;
  }

  if (map_name != status_.current_map()) {
    {
      // Update current_map status.
      WLock wlock(status_mutex_);
      status_.set_current_map(map_name);
      status_changed_ = true;
    }
    SetGlobalFlag("map_dir", *map_dir, &FLAGS_map_dir);
    ResetMode();
  }

  // true : restart dynamic model on change map
  // false : change scenario not restart
  // 1. 场景切换到空场景
  // 2. 场景切换到其他地图
  if (restart_dynamic_model) {
    callback_api_("RestartDynamicModel", {});
    // 场景id不为空进行切换地图需要停止sim_obstacle
    StopModuleByCommand(FLAGS_sim_obstacle_stop_command);
    {
      WLock wlock(status_mutex_);
      status_.set_current_scenario_id("");
      status_changed_ = true;
    }
  }
  return true;
}

void HMIWorker::UpdateModeModulesAndMonitoredComponents() {
  status_.clear_modules();
  for (const auto &iter : current_mode_.modules()) {
    status_.mutable_modules()->insert({iter.first, false});
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
  if (LoadVehicleDefinedMode(mode_config_path,
                             *vehicle_dir, &vehicle_defined_mode)) {
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
    status_changed_ = true;
  }

  FuelMonitorManager::Instance()->SetCurrentMode(mode_name);
  KVDB::Put(FLAGS_current_mode_db_key, mode_name);

  // Toggle Mode Set scenario to empty
  {
    WLock wlock(status_mutex_);
    status_.set_current_scenario_id("");
    status_changed_ = true;
  }
  StopModuleByCommand(FLAGS_sim_obstacle_stop_command);
}

void HMIWorker::StartModule(const std::string &module) const {
  const Module *module_conf = FindOrNull(current_mode_.modules(), module);
  if (module_conf != nullptr) {
    System(module_conf->start_command());
  } else {
    AERROR << "Cannot find module " << module;
  }

  if (module == "Recorder") {
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
  }
}

void HMIWorker::StopModule(const std::string &module) const {
  const Module *module_conf = FindOrNull(current_mode_.modules(), module);
  if (module_conf != nullptr) {
    System(module_conf->stop_command());
  } else {
    AERROR << "Cannot find module " << module;
  }
}

HMIStatus HMIWorker::GetStatus() const {
  RLock rlock(status_mutex_);
  return status_;
}

void HMIWorker::SetupMode() const {
  for (const auto &iter : current_mode_.modules()) {
    System(iter.second.start_command());
  }
}

void HMIWorker::ResetMode() const {
  for (const auto &iter : current_mode_.modules()) {
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
  constexpr double kSecondsTillTimeout(2.5);
  const double now = Clock::NowInSeconds();
  if (now - last_status_received_s_.load() > kSecondsTillTimeout) {
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

void HMIWorker::ChangeScenarioSet(const std::string &scenario_set_id) {
  {
    RLock rlock(status_mutex_);
    auto &scenario_set = status_.scenario_set();
    if ((!scenario_set_id.empty()) &&
        (scenario_set.find(scenario_set_id) == scenario_set.end())) {
      AERROR << "Cannot change to unknown scenario set!";
      return;
    }
    if (status_.current_scenario_set_id() == scenario_set_id) {
      return;
    }
  }

  {
    WLock wlock(status_mutex_);
    status_.set_current_scenario_set_id(scenario_set_id);
    status_changed_ = true;
  }
  return;
}

void HMIWorker::GetScenarioResourcePath(std::string *scenario_resource_path) {
  CHECK_NOTNULL(scenario_resource_path);
  const std::string home = cyber::common::GetEnv("HOME");
  *scenario_resource_path = home + FLAGS_resource_scenario_path;
}

void HMIWorker::GetScenarioSetPath(const std::string &scenario_set_id,
                                   std::string *scenario_set_path) {
  CHECK_NOTNULL(scenario_set_path);
  GetScenarioResourcePath(scenario_set_path);
  *scenario_set_path = *scenario_set_path + scenario_set_id;
  return;
}

bool HMIWorker::StopModuleByCommand(const std::string &stop_command) const {
  int ret = std::system(stop_command.data());
  if (ret < 0 || !WIFEXITED(ret)) {
    // 256 does not means failure
    AERROR << "Failed to stop sim obstacle";
    return false;
  }
  return true;
}

bool HMIWorker::ResetSimObstacle(const std::string &scenario_id) {
  bool startObstacle = true;
  std::string cur_scenario_id;
  if (scenario_id.empty()) {
    startObstacle = false;
    cur_scenario_id = status_.current_scenario_id();
  } else {
    cur_scenario_id = scenario_id;
  }

  std::string absolute_path = FLAGS_sim_obstacle_path;

  // found sim_obstacle binary
  const std::string command = "which sim_obstacle";
  std::string result = GetCommandRes(command);
  if (result != "") {
    absolute_path = result;
  } else if (!cyber::common::PathExists(absolute_path)) {
    AWARN << "Not found sim obstacle";
    startObstacle = false;
  }
  AINFO << "sim_obstacle binary path : " << absolute_path;

  StopModuleByCommand(FLAGS_sim_obstacle_stop_command);
  std::string scenario_set_id;
  {
    RLock rlock(status_mutex_);
    scenario_set_id = status_.current_scenario_set_id();
  }
  std::string scenario_set_path;
  GetScenarioSetPath(scenario_set_id, &scenario_set_path);
  const std::string scenario_path =
      scenario_set_path + "/scenarios/" + cur_scenario_id + ".json";
  AINFO << "scenario path : " << scenario_path;
  if (!cyber::common::PathExists(scenario_path)) {
    AERROR << "Failed to find scenario!";
    return false;
  }
  std::string map_name = "";
  double x;
  double y;
  bool need_to_change_map = true;
  {
    RLock rlock(status_mutex_);
    auto &scenario_set = status_.scenario_set();
    if (scenario_set.find(scenario_set_id) == scenario_set.end()) {
      AERROR << "Failed to find scenario set!";
      return false;
    }
    for (auto &scenario : scenario_set.at(scenario_set_id).scenarios()) {
      if (scenario.scenario_id() == cur_scenario_id) {
        map_name = scenario.map_name();
        x = scenario.start_point().x();
        y = scenario.start_point().y();
        break;
      }
    }
    if (map_name.empty()) {
      AERROR << "Failed to find scenario and get map dir!";
      return false;
    }
    need_to_change_map = (status_.current_map() != map_name);
  }
  // resetmodule before save open modules
  std::vector<std::string> modules_open;
  auto modulesMap = status_.modules();
  for (auto it = modulesMap.begin(); it != modulesMap.end(); ++it) {
    if (it->second == true) {
      modules_open.push_back(it->first);
    }
  }
  if (need_to_change_map) {
    if (!ChangeMap(map_name, false)) {
      AERROR << "Failed to change map!";
      return false;
    }
    callback_api_("MapServiceReloadMap", {});
  }
  // TODO(huanguang): if not changing map don't need to reset module
  // for (const auto &module : modules_open) {
  //   StartModule(module);
  // }
  // After changing the map, reset the start point from the scenario by
  // sim_control
  Json info;
  info["x"] = x;
  info["y"] = y;
  callback_api_("SimControlRestart", info);
  if (startObstacle) {
    // 启动sim obstacle
    const std::string start_command = "nohup " + absolute_path + " " +
                                      scenario_path + FLAGS_gflag_command_arg +
                                      " &";
    AINFO << "start sim_obstacle command : " << start_command;
    int ret = std::system(start_command.data());
    if (ret != 0) {
      AERROR << "Failed to start sim obstacle";
      return false;
    }
  }

  return true;
}

std::string HMIWorker::GetCommandRes(const std::string &cmd) {
  if (cmd.size() > 128) {
    AERROR << "command size exceeds 128";
    return "";
  }
  const char *cmdPtr = cmd.c_str();
  char buffer[128];
  std::string result = "";

  FILE *pipe = popen(cmdPtr, "r");
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }
  if (result.back() == '\n') {
    result.pop_back();
  }
  pclose(pipe);
  return result;
}

void HMIWorker::ChangeScenario(const std::string &scenario_id) {
  {
    RLock rlock(status_mutex_);
    // Skip if mode doesn't actually change.
    if (status_.current_scenario_id() == scenario_id) {
      return;
    }
    if (scenario_id.empty()) {
      // stop sim obstacle
      // todo： add check status
      // directly think pkill successful
      StopModuleByCommand(FLAGS_sim_obstacle_stop_command);
    } else {
      auto scenario_set = status_.mutable_scenario_set();
      auto &scenario_set_id = status_.current_scenario_set_id();
      if (scenario_set->find(scenario_set_id) == scenario_set->end()) {
        AERROR << "Current scenario set is invalid!";
        return;
      }
      bool find_res = false;
      for (auto &scenario : (*scenario_set)[scenario_set_id].scenarios()) {
        if (scenario.scenario_id() == scenario_id) {
          find_res = true;
          break;
        }
      }
      if (!find_res) {
        AERROR << "Cannot change to unknown scenario!";
        return;
      }
    }
  }

  // restart sim obstacle
  // move sim obstacle position for rlock wlock together will result to dead
  // lock
  if (!ResetSimObstacle(scenario_id)) {
    AERROR << "Cannot start sim obstacle by new scenario!";
    return;
  }

  {
    WLock wlock(status_mutex_);
    status_.set_current_scenario_id(scenario_id);
    status_changed_ = true;
  }
  return;
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
  Json param_json({});
  param_json["dynamic_model_name"] = dynamic_model_name;
  Json callback_res = callback_api_("ChangeDynamicModel", param_json);
  if (!callback_res.contains("result") || !callback_res["result"]) {
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

bool HMIWorker::UpdateScenarioSetToStatus(
    const std::string &scenario_set_id, const std::string &scenario_set_name) {
  ScenarioSet new_scenario_set;
  if (!UpdateScenarioSet(scenario_set_id, scenario_set_name,
                         &new_scenario_set)) {
    AERROR << "Failed to update scenario_set!";
    return false;
  }
  {
    WLock wlock(status_mutex_);
    auto scenario_set = status_.mutable_scenario_set();
    scenario_set->erase(scenario_set_id);
    (*scenario_set)[scenario_set_id] = new_scenario_set;
    status_changed_ = true;
  }
  return true;
}

bool HMIWorker::UpdateDynamicModelToStatus(
    const std::string &dynamic_model_name) {
  Json param_json({});
  param_json["dynamic_model_name"] = dynamic_model_name;
  Json callback_res = callback_api_("AddDynamicModel", param_json);
  if (!callback_res.contains("result") || !callback_res["result"]) {
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

bool HMIWorker::UpdateScenarioSet(const std::string &scenario_set_id,
                                  const std::string &scenario_set_name,
                                  ScenarioSet *new_scenario_set) {
  std::string scenario_set_directory_path;
  GetScenarioSetPath(scenario_set_id, &scenario_set_directory_path);
  scenario_set_directory_path = scenario_set_directory_path + "/scenarios/";
  new_scenario_set->set_scenario_set_name(scenario_set_name);
  if (!cyber::common::PathExists(scenario_set_directory_path)) {
    AERROR << "Scenario set has no scenarios!";
    return true;
  }
  DIR *directory = opendir(scenario_set_directory_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << scenario_set_directory_path;
    return false;
  }

  struct dirent *file;
  while ((file = readdir(directory)) != nullptr) {
    // skip directory_path/. and directory_path/..
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }
    const std::string file_name = file->d_name;
    if (!absl::EndsWith(file_name, ".json")) {
      continue;
    }
    const int index = file_name.rfind(".json");
    if (index == 0) {
      // name: ".json" is invalid.
      continue;
    }
    const std::string scenario_id = file_name.substr(0, index);
    const std::string file_path = scenario_set_directory_path + file_name;
    SimTicket new_sim_ticket;
    if (!cyber::common::GetProtoFromJsonFile(file_path, &new_sim_ticket)) {
      AERROR << "Cannot parse this scenario:" << file_path;
      return false;
    }
    if (!new_sim_ticket.has_scenario()) {
      AERROR << "Cannot get scenario.";
      return false;
    }
    if (!new_sim_ticket.description_en_tokens_size()) {
      AERROR << "Cannot get scenario name.";
      return false;
    }
    if (!new_sim_ticket.scenario().has_map_dir()) {
      AERROR << "Cannot get scenario map dir.";
      return false;
    }
    if (!new_sim_ticket.scenario().has_start()) {
      AERROR << "Cannot get scenario start_point.";
      return false;
    }
    auto &scenario_start_point = new_sim_ticket.scenario().start();
    if (!scenario_start_point.has_x() || !scenario_start_point.has_y()) {
      AERROR << "Scenario start_point is invalid!";
      return false;
    }
    std::string scenario_name = new_sim_ticket.description_en_tokens(0);
    for (int i = 1; i < new_sim_ticket.description_en_tokens_size(); i++) {
      scenario_name =
          scenario_name + "_" + new_sim_ticket.description_en_tokens(i);
    }
    ScenarioInfo *scenario_info = new_scenario_set->add_scenarios();
    scenario_info->set_scenario_id(scenario_id);
    scenario_info->set_scenario_name(scenario_name);
    // change scenario json map dir to map name
    // format:modules/map/data/${map_name}
    const std::string map_dir = new_sim_ticket.scenario().map_dir();
    size_t idx = map_dir.find_last_of('/');
    if (idx == map_dir.npos) {
      AERROR << "Cannot get scenario map name.";
      return false;
    }
    const std::string map_name = map_dir.substr(idx + 1);
    if (map_name.empty()) {
      AERROR << "Cannot get scenario map name.";
      return false;
    }
    // replay engine use xx_xx like:apollo_map
    // dv need Apollo Map
    scenario_info->set_map_name(util::HMIUtil::TitleCase(map_name));
    auto start_point = scenario_info->mutable_start_point();
    start_point->set_x(scenario_start_point.x());
    start_point->set_y(scenario_start_point.y());
  }
  closedir(directory);
  return true;
}

bool HMIWorker::LoadScenarios() {
  std::string directory_path;
  GetScenarioResourcePath(&directory_path);
  if (!cyber::common::PathExists(directory_path)) {
    AERROR << "Failed to find scenario_set!";
    return false;
  }
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    AERROR << "Cannot open directory " << directory_path;
    return false;
  }
  struct dirent *file;
  std::map<std::string, ScenarioSet> scenario_sets;
  while ((file = readdir(directory)) != nullptr) {
    // skip directory_path/. and directory_path/..
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }
    if (file->d_type != DT_DIR) {
      continue;
    }
    const std::string scenario_set_id = file->d_name;
    const std::string scenario_set_json_path =
        directory_path + scenario_set_id + "/scenario_set.json";
    // scenario_set.json use message:UserAdsGroup
    UserAdsGroup user_ads_group_info;
    if (!cyber::common::GetProtoFromJsonFile(scenario_set_json_path,
                                             &user_ads_group_info)) {
      AERROR << "Unable to parse UserAdsGroup from file "
             << scenario_set_json_path;
      continue;
    }
    if (!user_ads_group_info.has_name()) {
      AERROR << "Failed to get ads group name!";
      continue;
    }
    const std::string scenario_set_name = user_ads_group_info.name();
    ScenarioSet new_scenario_set;
    if (!UpdateScenarioSet(scenario_set_id, scenario_set_name,
                           &new_scenario_set)) {
      AERROR << "Failed to update scenario_set!";
      continue;
    }
    scenario_sets[scenario_set_id] = new_scenario_set;
  }
  closedir(directory);
  {
    WLock wlock(status_mutex_);
    auto scenario_set = status_.mutable_scenario_set();
    // clear old data
    scenario_set->clear();
    for (auto iter = scenario_sets.begin(); iter != scenario_sets.end();
         iter++) {
      (*scenario_set)[iter->first] = iter->second;
    }
    status_changed_ = true;
  }
  return true;
}

bool HMIWorker::LoadDynamicModels() {
  Json load_res = callback_api_("LoadDynamicModels", {});
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
  return load_res["result"];
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

  {
    RLock rlock(status_mutex_);
    auto &scenario_set = status_.scenario_set();
    if (scenario_set.find(scenario_set_id) == scenario_set.end()) {
      AERROR << "Cannot find unknown scenario set!";
      return;
    }
    // do not allowed remove changed current scenario set
    if (scenario_set_id == status_.current_scenario_set_id()) {
      AERROR << "Cannotdelete current scenario set!";
      return;
    }
  }

  {
    WLock wlock(status_mutex_);
    status_.mutable_scenario_set()->erase(scenario_set_id);
    status_changed_ = true;
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
  Json param_json({});
  param_json["dynamic_model_name"] = dynamic_model_name;
  Json callback_res = callback_api_("DeleteDynamicModel", param_json);
  if (!callback_res.contains("result") || !callback_res["result"]) {
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

bool HMIWorker::RePlayRecord(const std::string &record_id) {
  std::string record_path;
  GetRecordPath(&record_path);
  record_path = record_path + record_id + ".record";

  if (!cyber::common::PathExists(record_path)) {
    AERROR << "Failed to find record!";
    return false;
  }
  // play the record
  const std::string play_command =
      absl::StrCat("nohup cyber_recorder play -l -f ", record_path, " &");
  int ret = std::system(play_command.data());
  if (ret != 0) {
    AERROR << "Failed to start cyber play command";
    return false;
  }
  return true;
}
void HMIWorker::StopRecordPlay() {
  WLock wlock(status_mutex_);
  { status_.set_current_record_id(""); }
  if (!StopModuleByCommand(FLAGS_cyber_recorder_stop_command)) {
    AERROR << "stop record failed";
  }
  status_changed_ = true;
}
void HMIWorker::ChangeRecord(const std::string &record_id) {
  StopRecordPlay();
  {
    RLock rlock(status_mutex_);
    auto status_records = status_.mutable_records();
    if (status_records->find(record_id) == status_records->end()) {
      AERROR << "Cannot change to unknown record!";
      return;
    }
    if (!RePlayRecord(record_id)) {
      return;
    }
  }
  WLock wlock(status_mutex_);
  status_.set_current_record_id(record_id);
  status_changed_ = true;
  return;
}
bool HMIWorker::LoadRecords() {
  std::string directory_path;
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
  std::map<std::string, LoadRecordInfo> new_records;
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
      // compatible records with dv and dv_plus
      new_records[local_record_resource] = {};
      new_records[local_record_resource].set_download_status(1);
    }
  }
  closedir(directory);
  {
    WLock wlock(status_mutex_);
    auto status_records = status_.mutable_records();
    status_records->clear();
    for (auto iter = new_records.begin(); iter != new_records.end(); iter++) {
      (*status_records)[iter->first] = iter->second;
    }
    status_changed_ = true;
  }
  return true;
}

void HMIWorker::DeleteRecord(const std::string &record_id) {
  StopRecordPlay();
  if (record_id.empty()) {
    return;
  }
  std::string record_path;
  GetRecordPath(&record_path);
  record_path = record_path + record_id + ".record";
  if (!cyber::common::PathExists(record_path)) {
    return;
  }
  // find the delete record if exist and judge the record whether playing now
  {
    RLock rlock(status_mutex_);
    auto &status_records = status_.records();
    if (status_records.find(record_id) == status_records.end()) {
      return;
    }
    if (record_id == status_.current_record_id()) {
      return;
    }
  }
  {
    WLock wlock(status_mutex_);
    status_.mutable_records()->erase(record_id);
    status_changed_ = true;
  }

  // delete record from disk
  std::string command = "rm -rf " + record_path;
  if (std::system(command.data()) != 0) {
    AERROR << "Failed to delete record for: " << std::strerror(errno);
    return;
  }
  return;
}
bool HMIWorker::ReloadVehicles() {
  AINFO << "load config";
  HMIConfig config = util::HMIUtil::LoadConfig(FLAGS_dv_hmi_modes_config_path);
  std::string msg;
  AINFO << "serialize new config";
  config.SerializeToString(&msg);

  WLock wlock(status_mutex_);
  AINFO << "parse new config";
  config_.ParseFromString(msg);
  AINFO << "init status";
  // status_.clear_modes();
  // status_.clear_maps();
  AINFO << "clear vehicles";
  status_.clear_vehicles();
  // InitStatus();
  // Populate vehicles and current_vehicle.
  AINFO << "reload vehicles";
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
}  // namespace dreamview
}  // namespace apollo
