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
#include "modules/control/controllers/lon_based_pid_plus_controller/lon_plus_controller.h"

#include <algorithm>
#include <utility>

#include "absl/strings/str_cat.h"

#include "modules/control/control_component/proto/pid_conf.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Time;
using apollo::external_command::CommandStatusType;
using apollo::planning::ADCTrajectory;
using apollo::planning::StopReasonCode;

constexpr double GRA_ACC = 9.8;

LonPlusController::LonPlusController() : name_("PID-basesd Longitudinal Plus Controller") {
    // node_.reset(new apollo::cyber::Node("lon_plus_controller"));
    if (FLAGS_enable_csv_debug) {
        time_t rawtime;
        char name_buffer[80];
        std::time(&rawtime);
        std::tm time_tm;
        localtime_r(&rawtime, &time_tm);
        strftime(name_buffer, 80, "/tmp/speed_log__%F_%H%M%S.csv", &time_tm);
        speed_log_file_ = fopen(name_buffer, "w");
        if (speed_log_file_ == nullptr) {
            AERROR << "Fail to open file:" << name_buffer;
            FLAGS_enable_csv_debug = false;
        }
        if (speed_log_file_ != nullptr) {
            fprintf(speed_log_file_,
                    "station_reference,"
                    "station_error,"
                    "station_error_limited,"
                    "preview_station_error,"
                    "speed_reference,"
                    "speed_error,"
                    "speed_error_limited,"
                    "preview_speed_reference,"
                    "preview_speed_error,"
                    "preview_acceleration_reference,"
                    "acceleration_cmd_closeloop,"
                    "acceleration_cmd,"
                    "acceleration_lookup,"
                    "acceleration_lookup_limit,"
                    "speed_lookup,"
                    "calibration_value,"
                    "throttle_cmd,"
                    "brake_cmd,"
                    "is_full_stop,"
                    "\r\n");

            fflush(speed_log_file_);
        }
        AINFO << name_ << " used.";
    }
}

void LonPlusController::CloseLogFile() {
    if (FLAGS_enable_csv_debug) {
        if (speed_log_file_ != nullptr) {
            fclose(speed_log_file_);
            speed_log_file_ = nullptr;
        }
    }
}
void LonPlusController::Stop() {
    CloseLogFile();
}

LonPlusController::~LonPlusController() {
    CloseLogFile();
}

Status LonPlusController::Init(std::shared_ptr<DependencyInjector> injector) {
    if (!ControlTask::LoadConfig<LonBasedPidPlusControllerConf>(&lon_based_pidcontroller_conf_)) {
        AERROR << "failed to load control default conf";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load lon control_conf");
    }

    if (!ControlTaskExtend::LoadParamsPipelineConfig<ParamsPipeline>(&params_pipeline_)) {
        AERROR << "failed to load lon pid params pipeline config";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load lon pid params pipeline config");
    }

    if (!LoadParams(&params_pipeline_)) {
        AERROR << "failed to load lon pid controller params.";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load lon pid params pipeline config");
    }

    injector_ = injector;

    station_pid_controller_.InitPID();
    speed_pid_controller_.InitPID();

    station_leadlag_controller_.InitLeadlag();
    speed_leadlag_controller_.InitLeadlag();

    vehicle_param_.CopyFrom(common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());

    SetDigitalFilterPitchAngle();

    controller_initialized_ = true;

    return Status::OK();
}

bool LonPlusController::LoadParams(ParamsPipeline *params_pipeline) {
    for (int i = 0; i < params_pipeline->params_declare_size(); i++) {
        LonBasedPidPlusControllerConf param_config;
        auto param_config_path = params_pipeline->params_declare(i).config_path();
        std::string param_config_path_relative = absl::StrCat("conf", "/", param_config_path);
        AINFO << "Param config_path_relative is " << param_config_path_relative;
        std::string param_config_path_absolute
                = apollo::cyber::plugin_manager::PluginManager::Instance()->GetPluginConfPath<ControlTask>(
                        "apollo::control::LonPlusController", param_config_path_relative);
        AINFO << "Param config_path_absolute is " << param_config_path_absolute;
        if (!apollo::cyber::common::GetProtoFromFile(param_config_path_absolute, &param_config)) {
            AERROR << "Load config: " << params_pipeline->params_declare(i).config_name() << " param failed!";
            return false;
        }
        AINFO << "Load the config file successfully, param file name: " << param_config_path;
        params_list_.push_back(std::make_pair(params_pipeline->params_declare(i).config_name(), param_config));
    }
    for (int i = 0; i < params_pipeline->calibration_table_declare_size(); i++) {
        calibration_table calibration_table_conf;
        auto calibration_table_path = params_pipeline->calibration_table_declare(i).config_path();
        if (!LoadCalibrationTable(calibration_table_path, &calibration_table_conf)) {
            AERROR << "Load config: " << params_pipeline->calibration_table_declare(i).config_name()
                   << " calibration table failed!";
            return false;
        }
        AINFO << "Load the calibration table successfully, calibration table file name: " << calibration_table_path;
        calibration_table_list_.push_back(
                std::make_pair(params_pipeline->calibration_table_declare(i).config_name(), calibration_table_conf));
        for (const auto &calibration_table : calibration_table_list_) {
            AINFO << "calibration table name is " << calibration_table.first;
            InitControlCalibrationTable(calibration_table.second);
        }
    }

    return true;
}

void LonPlusController::SetDigitalFilterPitchAngle() {
    double cutoff_freq = lon_based_pidcontroller_conf_.pitch_angle_filter_conf().cutoff_freq();
    double ts = lon_based_pidcontroller_conf_.ts();
    SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}

void LonPlusController::InitControlCalibrationTable(calibration_table calibration_table) {
    AINFO << "Control calibration table size is " << calibration_table.calibration_size();
    Interpolation2D::DataType xyz;
    for (const auto &calibration : calibration_table.calibration()) {
        xyz.push_back(std::make_tuple(calibration.speed(), calibration.acceleration(), calibration.command()));
    }
    control_interpolation_.reset(new Interpolation2D);
    ACHECK(control_interpolation_->Init(xyz)) << "Fail to load control calibration table";
}

bool LonPlusController::SetCurrentParam(
        const std::string param_config_name,
        LonBasedPidPlusControllerConf &current_lon_based_pidcontroller_conf) {
    bool set_success = false;
    for (const auto &param : params_list_) {
        if (param.first == param_config_name) {
            current_lon_based_pidcontroller_conf.CopyFrom(param.second);
            set_success = true;
            ADEBUG << "find the right params, set the currernt param_config_name is " << param_config_name;
            break;
        }
    }
    if (!set_success) {
        AERROR << "Fail to find lon current param[" << param_config_name << "]";
    }
    return set_success;
}

bool LonPlusController::ChooseCurrentParam(
        const planning::ADCTrajectory *trajectory_msg,
        const double vehicle_speed,
        LonBasedPidPlusControllerConf &current_lon_based_pidcontroller_conf) {
    std::string param_config_name = "";

    if (trajectory_msg->trajectory_type() != ADCTrajectory::OPEN_SPACE) {
        param_config_name = "public_road_forward_param";
    } else {
        if (!IsForwardModel()) {
            param_config_name = "open_space_backward_param";
        } else {
            param_config_name = "open_space_forward_param";
        }
    }

    if (current_lon_based_pidcontroller_conf.use_check_pit()
        && CheckInPit(
                lon_based_pidcontroller_conf_.pit_replan_check_time(),
                lon_based_pidcontroller_conf_.pit_replan_check_count(),
                std::fabs(vehicle_speed),
                trajectory_message_->is_replan())) {
        if (!IsForwardModel()) {
            param_config_name = "pit_backward_param";
        } else {
            param_config_name = "pit_forward_param";
        }
    }

    if (trajectory_msg->trajectory_type() == ADCTrajectory::OPEN_SPACE
        && trajectory_msg->debug().planning_data().scenario().scenario_plugin_type() == "PRECISE_PARKING") {
        if (!IsForwardModel()) {
            param_config_name = "precise_backward_param";
        } else {
            param_config_name = "precise_forward_param";
        }
    }

    if (trajectory_msg->debug().planning_data().scenario().scenario_plugin_type() == "ZONE_COVER") {
        if (!IsForwardModel()) {
            param_config_name = "zone_cover_backward_param";
        } else {
            param_config_name = "zone_cover_forward_param";
        }
    }

    if (!SetCurrentParam(param_config_name, current_lon_based_pidcontroller_conf)) {
        AERROR << "Failed to set current lon param config: " << param_config_name;
        return false;
    } else {
        if (param_config_name != current_param_config_name_) {
            AINFO << "Update the current lon param config name is " << param_config_name;
            current_param_config_name_ = param_config_name;
        }
        return true;
    }
}

bool LonPlusController::SetCurrentCalibrationTable(
        const std::string calibration_table_config_name,
        calibration_table &current_calibration_table_conf) {
    bool set_success = false;
    for (const auto &calibration_table : calibration_table_list_) {
        if (calibration_table.first == calibration_table_config_name) {
            current_calibration_table_conf.CopyFrom(calibration_table.second);
            set_success = true;
            break;
        }
    }
    if (!set_success) {
        AERROR << "Fail to find lon current calibration table[" << calibration_table_config_name << "]";
    }
    return set_success;
}

bool LonPlusController::ChooseCurrentCalibrationTable(calibration_table &current_calibration_table_conf) {
    std::string calibration_table_config_name = "default_calibration_table";
    if (chassis_->battery_soc_percentage() > current_lon_based_pidcontroller_conf_.battery_threshold()) {
        calibration_table_config_name = "high_battery_calibration_table";
    } else {
        calibration_table_config_name = "normal_battery_calibration_table";
    }

    if (!SetCurrentCalibrationTable(calibration_table_config_name, current_calibration_table_conf)) {
        AERROR << "Failed to set current calibration table: " << calibration_table_config_name;
        return false;
    } else {
        if (calibration_table_config_name != current_calibration_table_config_name_) {
            AINFO << "Update the current calibration table config name is " << calibration_table_config_name;
            current_calibration_table_config_name_ = calibration_table_config_name;
        }
        return true;
    }
}

Status LonPlusController::ComputeControlCommand(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis,
        const planning::ADCTrajectory *planning_published_trajectory,
        control::ControlCommand *cmd) {
    localization_ = localization;
    chassis_ = chassis;
    trajectory_message_ = planning_published_trajectory;

    auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();
    debug->Clear();
    auto lon_debug_info = injector_->mutable_control_debug_info()->mutable_simple_lon_debug();
    lon_debug_info->Clear();

    // speed filter
    double vehicle_speed_ori = injector_->vehicle_state()->linear_velocity();
    lon_debug_info->set_vehicle_speed(vehicle_speed_ori);

    double current_speed = GetVehicleSpeed(injector_, FLAGS_use_speed_filter, FLAGS_speed_smoothing_factor);
    lon_debug_info->set_vehicle_speed_filter(current_speed);

    if (!VehicleStatusIdentificationUpdate(localization_, chassis_, trajectory_message_)) {
        AERROR << "Fail to update the vehicle status identification!";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Fail to update the vehicle status identification!");
    }

    if (trajectory_analyzer_ == nullptr
        || trajectory_analyzer_->seq_num() != trajectory_message_->header().sequence_num()) {
        trajectory_analyzer_.reset(new TrajectoryAnalyzer(trajectory_message_));
    }

    if (!ChooseCurrentParam(trajectory_message_, current_speed, current_lon_based_pidcontroller_conf_)) {
        AERROR << "Fail to load current lon control param!";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Fail to load current lon control param!");
    }

    if (!ChooseCurrentCalibrationTable(calibration_table_)) {
        AERROR << "Fail to load the current lon control calibration table!";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Fail to load the current lon control calibration table!!");
    }

    if (calibration_table_.calibration_size() != previous_calibration_table_size_) {
        AINFO << "Update the calibration table, init the new calibration table.";
        InitControlCalibrationTable(calibration_table_);
        previous_calibration_table_size_ = calibration_table_.calibration_size();
    }

    if (!control_interpolation_) {
        AERROR << "Fail to initialize calibration table.";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Fail to initialize calibration table.");
    }

    double brake_cmd = 0.0;
    double throttle_cmd = 0.0;
    double ts = current_lon_based_pidcontroller_conf_.ts();
    double preview_time = current_lon_based_pidcontroller_conf_.preview_window() * ts;
    bool enable_leadlag = current_lon_based_pidcontroller_conf_.enable_reverse_leadlag_compensation();

    standstill_narmal_acceleration_
            = -std::fabs(current_lon_based_pidcontroller_conf_.standstill_narmal_acceleration());
    stop_gain_acceleration_ = -std::fabs(current_lon_based_pidcontroller_conf_.stop_gain_acceleration());

    if (preview_time < 0.0) {
        const auto error_msg = absl::StrCat("Preview time set as: ", preview_time, " less than 0");
        AERROR << error_msg;
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
    }

    station_leadlag_controller_.SetLeadlagConf(current_lon_based_pidcontroller_conf_.speed_leadlag_conf());
    speed_leadlag_controller_.SetLeadlagConf(current_lon_based_pidcontroller_conf_.speed_leadlag_conf());

    CheckFinishToElse(planning_published_trajectory);
    ComputeLongitudinalErrors(trajectory_analyzer_.get(), preview_time, ts, current_speed, debug);

    double station_error_limit = current_lon_based_pidcontroller_conf_.station_error_limit();
    double station_error_limited = 0.0;
    if (current_lon_based_pidcontroller_conf_.enable_speed_station_preview()) {
        station_error_limited
                = common::math::Clamp(debug->preview_station_error(), -station_error_limit, station_error_limit);
    } else {
        station_error_limited = common::math::Clamp(debug->station_error(), -station_error_limit, station_error_limit);
    }

    auto station_pid_conf = current_lon_based_pidcontroller_conf_.station_pid_conf();
    CalcultePidParams(std::fabs(current_speed), station_pid_conf);
    station_pid_controller_.SetPID(station_pid_conf);
    auto speed_pid_conf = current_lon_based_pidcontroller_conf_.speed_pid_conf();
    CalcultePidParams(std::fabs(current_speed), speed_pid_conf);
    speed_pid_controller_.SetPID(speed_pid_conf);

    current_trajectory_timestamp_ = planning_published_trajectory->header().timestamp_sec();
    if ((current_trajectory_timestamp_ - pre_trajectory_timestamp_ > 1e-4)
        && planning_published_trajectory->is_replan()) {
        station_pid_controller_.Reset_integral();
        speed_pid_controller_.Reset_integral();
        ADEBUG << "Reset the pid controller integral because of re-planning.";
    }
    pre_trajectory_timestamp_ = current_trajectory_timestamp_;

    double speed_offset = station_pid_controller_.Control(station_error_limited, ts);
    if (enable_leadlag) {
        speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
    }

    double speed_controller_input = 0.0;
    double speed_controller_input_limit = current_lon_based_pidcontroller_conf_.speed_controller_input_limit();
    double speed_controller_input_limited = 0.0;
    if (current_lon_based_pidcontroller_conf_.enable_speed_station_preview()) {
        speed_controller_input = speed_offset + debug->preview_speed_error();
    } else {
        speed_controller_input = speed_offset + debug->speed_error();
    }
    speed_controller_input_limited
            = common::math::Clamp(speed_controller_input, -speed_controller_input_limit, speed_controller_input_limit);

    double acceleration_cmd_closeloop = 0.0;

    acceleration_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input_limited, ts);
    debug->set_pid_saturation_status(speed_pid_controller_.IntegratorSaturationStatus());
    if (enable_leadlag) {
        acceleration_cmd_closeloop = speed_leadlag_controller_.Control(acceleration_cmd_closeloop, ts);
        debug->set_leadlag_saturation_status(speed_leadlag_controller_.InnerstateSaturationStatus());
    }

    if (chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
        speed_pid_controller_.Reset_integral();
        station_pid_controller_.Reset_integral();
    }

    double vehicle_pitch_rad = injector_->vehicle_state()->pitch();
    double vehicle_pitch = vehicle_pitch_rad * 180 / M_PI + FLAGS_pitch_offset_deg;
    ADEBUG << "[LON]vehicle_pitch is " << vehicle_pitch;
    debug->set_vehicle_pitch(vehicle_pitch);
    // TODO(ALL): confirm the slope_offset_compensation whether is positive or not
    // when vehicle move uphill
    // Resume: uphill: + , downhill: -
    double slope_offset_compensation = current_lon_based_pidcontroller_conf_.use_opposite_slope_compensation() * GRA_ACC
            * std::sin(vehicle_pitch_rad + FLAGS_pitch_offset_deg * M_PI / 180);

    if (std::isnan(slope_offset_compensation)) {
        slope_offset_compensation = 0;
    }

    debug->set_slope_offset_compensation(slope_offset_compensation);
    double acceleration_cmd = acceleration_cmd_closeloop + debug->preview_acceleration_reference();

    // Check the steer command in reverse trajectory if the current steer target
    // is larger than previous target, free the acceleration command, wait for
    // the current steer target
    double current_steer_interval = cmd->steering_target() - chassis->steering_percentage();
    if (current_lon_based_pidcontroller_conf_.use_steering_check()
        && (trajectory_message_->trajectory_type() == apollo::planning::ADCTrajectory::UNKNOWN)
        && std::abs(current_steer_interval) > current_lon_based_pidcontroller_conf_.steer_cmd_interval()) {
        ADEBUG << "steering_target is " << cmd->steering_target() << " steering_percentage is "
               << chassis->steering_percentage();
        ADEBUG << "Steer cmd interval is larger than " << current_lon_based_pidcontroller_conf_.steer_cmd_interval();

        speed_pid_controller_.Reset_integral();
        station_pid_controller_.Reset_integral();
        acceleration_cmd = 0;
        debug->set_is_wait_steer(true);
    } else {
        debug->set_is_wait_steer(false);
    }
    debug->set_current_steer_interval(current_steer_interval);

    // At near-stop stage, replace the brake control command with the standstill
    // acceleration if the former is even softer than the latter
    debug->set_is_full_stop(false);
    debug->set_is_full_stop_soft(false);
    // previous_full_stop will determine whether the max path remain will be added.
    auto previous_full_stop = injector_->Get_previous_lon_debug_info()->is_full_stop()
            || injector_->Get_previous_lon_debug_info()->is_full_stop_soft();
    GetPathRemain(debug);
    IsStopByDestination(debug);
    IsPedestrianStopLongTerm(debug);
    if (current_lon_based_pidcontroller_conf_.use_preview_reference_check()
        && (std::fabs(debug->preview_acceleration_reference()) <= FLAGS_max_acceleration_when_stopped)
        && std::fabs(debug->preview_speed_reference()) <= vehicle_param_.max_abs_speed_when_stopped()
        && trajectory_message_->trajectory_type() != ADCTrajectory::OPEN_SPACE) {
        if (debug->is_stop_reason_by_destination() || debug->is_stop_reason_by_prdestrian()) {
            debug->set_is_full_stop(true);
            ADEBUG << "Into full stop within preview acc and reference speed, " << "is_full_stop is "
                   << debug->is_full_stop();
        } else {
            debug->set_is_full_stop_soft(true);
            ADEBUG << "Into full stop soft within preview acc and reference speed, " << "is_full_stop_soft is "
                   << debug->is_full_stop_soft();
        }
    }

    double path_remain_add = 0.0;
    if (vehicle_pitch < current_lon_based_pidcontroller_conf_.pitch_path_remain_param_min()
        && fabs(current_speed) > current_lon_based_pidcontroller_conf_.downhill_full_stop_min_speed()
        && trajectory_message_->trajectory_type() != ADCTrajectory::OPEN_SPACE) {
        path_remain_add = fabs(
                sin(vehicle_pitch * M_PI / 180) * current_lon_based_pidcontroller_conf_.pitch_path_remain_param());
    }

    if (!previous_full_stop) {
        max_path_remain_when_stopped_ = current_lon_based_pidcontroller_conf_.path_remain() + path_remain_add;
    } else {
        max_path_remain_when_stopped_ = current_lon_based_pidcontroller_conf_.path_remain()
                + current_lon_based_pidcontroller_conf_.full_stop_path_remain_gain() + path_remain_add;
    }
    ADEBUG << "current max_path_remain_when_stopped_ is " << max_path_remain_when_stopped_;

    if (((IsForwardModel() && debug->path_remain() < max_path_remain_when_stopped_)
         || (!IsForwardModel() && debug->path_remain() > -max_path_remain_when_stopped_))) {
        ADEBUG << "Into full stop decision by path remain.";
        if (debug->is_stop_reason_by_destination() || debug->is_stop_reason_by_prdestrian()) {
            debug->set_is_full_stop(true);
            ADEBUG << "Current path remain distance: " << debug->path_remain()
                   << " is within max_path_remain threshold: " << max_path_remain_when_stopped_
                   << ", into full stop because vehicle is in destination: " << debug->is_stop_reason_by_destination()
                   << " or pedestrian is in long time stop: " << debug->is_stop_reason_by_prdestrian()
                   << "is_full_stop flag: " << debug->is_full_stop();
        } else {
            debug->set_is_full_stop_soft(true);
            ADEBUG << "Current path remain distance: " << debug->path_remain()
                   << " is within max_path_remain threshold: " << max_path_remain_when_stopped_
                   << ", but not into full stop because stop not in destination: "
                   << debug->is_stop_reason_by_destination()
                   << " or pedestrian is not long time stop: " << debug->is_stop_reason_by_prdestrian()
                   << "is_full_stop_soft flag: " << debug->is_full_stop_soft();
        }
        if ((std::fabs(current_speed) < vehicle_param_.max_abs_speed_when_stopped())
            && debug->is_stop_reason_by_prdestrian()) {
            ADEBUG << "Current stop is for long time pedestrian stop, " << debug->is_stop_reason_by_prdestrian();
            debug->set_is_full_stop(true);
        }
    } else {
        ADEBUG << "Not into full stop decision by path remain.";
    }

    if (!debug->is_full_stop() && !debug->is_full_stop_soft()) {
        is_full_stop_wait_time_diff_ = 0;
        ResetNewFullStop();
    } else {
        if (!previous_is_full_stop_ && (debug->is_full_stop() || debug->is_full_stop_soft())) {
            ADEBUG << "from go into full stop";
            previous_is_full_stop_ = true;
            form_go_to_full_stop_ = true;
        }
    }

    double throttle_lowerbound = std::max(
            vehicle_param_.throttle_deadzone(), current_lon_based_pidcontroller_conf_.throttle_minimum_action());
    double brake_lowerbound
            = std::max(vehicle_param_.brake_deadzone(), current_lon_based_pidcontroller_conf_.brake_minimum_action());
    double calibration_value = 0.0;
    double acceleration_lookup = (!IsForwardModel()) ? -acceleration_cmd : acceleration_cmd;

    double acceleration_lookup_limited = vehicle_param_.max_acceleration()
            + current_lon_based_pidcontroller_conf_.enable_slope_offset() * debug->slope_offset_compensation();
    double acceleration_lookup_limit = 0.0;

    if (current_lon_based_pidcontroller_conf_.use_acceleration_lookup_limit()) {
        acceleration_lookup_limit = (acceleration_lookup > acceleration_lookup_limited) ? acceleration_lookup_limited
                                                                                        : acceleration_lookup;
    }

    if (FLAGS_use_preview_speed_for_table) {
        if (current_lon_based_pidcontroller_conf_.use_acceleration_lookup_limit()) {
            calibration_value = control_interpolation_->Interpolate(
                    std::make_pair(debug->preview_speed_reference(), acceleration_lookup_limit));
        } else {
            calibration_value = control_interpolation_->Interpolate(
                    std::make_pair(debug->preview_speed_reference(), acceleration_lookup));
        }
    } else {
        if (current_lon_based_pidcontroller_conf_.use_acceleration_lookup_limit()) {
            calibration_value = control_interpolation_->Interpolate(
                    std::make_pair(std::fabs(current_speed), acceleration_lookup_limit));
        } else {
            calibration_value = control_interpolation_->Interpolate(
                    std::make_pair(std::fabs(current_speed), acceleration_lookup));
        }
    }

    if (acceleration_lookup >= 0) {
        if (calibration_value >= 0) {
            throttle_cmd = std::max(calibration_value, throttle_lowerbound);
        } else {
            throttle_cmd = throttle_lowerbound;
        }
        brake_cmd = 0.0;
    } else {
        throttle_cmd = 0.0;
        if (calibration_value >= 0) {
            brake_cmd = brake_lowerbound;
        } else {
            brake_cmd = std::max(-calibration_value, brake_lowerbound);
        }
    }

    if (form_go_to_full_stop_ && (debug->is_full_stop() || debug->is_full_stop_soft())) {
        ADEBUG << "from go to full stop in to control";
        ADEBUG << "now brake  = " << cmd->brake();
        throttle_cmd = 0.0;
        openspace_speed_control_throttle_cmd_ = 0.0;
        if (debug->acceleration_reference() < current_lon_based_pidcontroller_conf_.emergency_quit_fullstop_mindec()) {
            ADEBUG << "[NewFullStop] emergency stop, use acc before fullstop.";
        } else {
            if (((std::fabs(current_speed) < current_lon_based_pidcontroller_conf_.release_end_brake_speed())
                 || (std::fabs(debug->path_remain()) < current_lon_based_pidcontroller_conf_.fullstop_pathremain()))
                && !begin_end_brake_) {
                begin_end_brake_ = true;
                fullstop_in_brake_ = cmd->brake();
                double start_brake = current_lon_based_pidcontroller_conf_.full_stop_pitch_brake_param()
                        * fabs(std::sin(vehicle_pitch_rad + FLAGS_pitch_offset_deg * M_PI / 180));
                if (start_brake > fullstop_in_brake_) {
                    fullstop_in_brake_ = start_brake;
                }
                hillup_fullstop_brake_rate_ = std::max(
                        current_lon_based_pidcontroller_conf_.full_stop_max_brake()
                                / (current_lon_based_pidcontroller_conf_.fullstop_window()
                                   - current_lon_based_pidcontroller_conf_.full_stop_pitch_window_param()
                                           * fabs(std::sin(vehicle_pitch_rad + FLAGS_pitch_offset_deg * M_PI / 180))),
                        0.0);
                ADEBUG << "======brake rate  = " << hillup_fullstop_brake_rate_;
                ADEBUG << "brake cal window = "
                       << current_lon_based_pidcontroller_conf_.fullstop_window()
                                - fabs(std::sin(vehicle_pitch_rad + FLAGS_pitch_offset_deg * M_PI / 180));
            }
            if (IsForwardModel()) {
                double station_err = (debug->station_error() < 0) ? debug->station_error() : 0.0;
                hillup_fullstop_brake_rate_ = hillup_fullstop_brake_rate_
                        + fabs(station_err) * current_lon_based_pidcontroller_conf_.full_stop_station_err_rate();
                if (hillup_fullstop_brake_rate_
                    > current_lon_based_pidcontroller_conf_.max_brk_downhill_change_rate()) {
                    hillup_fullstop_brake_rate_ = current_lon_based_pidcontroller_conf_.max_brk_downhill_change_rate();
                }
            }

            if (begin_end_brake_) {
                fullstop_in_brake_ = fullstop_in_brake_ + hillup_fullstop_brake_rate_;
                if ((fullstop_window_count_ > current_lon_based_pidcontroller_conf_.fullstop_window())
                    && (fullstop_in_brake_ > current_lon_based_pidcontroller_conf_.full_stop_max_brake()
                                + current_lon_based_pidcontroller_conf_.full_stop_pitch_brake_param()
                                        * fabs(std::sin(vehicle_pitch_rad + FLAGS_pitch_offset_deg * M_PI / 180)))) {
                    hillup_fullstop_brake_rate_ = 0;
                }
                if (fullstop_in_brake_ >= 100.0) {
                    fullstop_in_brake_ = 100.0;
                }
                brake_cmd = fullstop_in_brake_;
                ADEBUG << "=======now brkcmd===AL== = " << fullstop_in_brake_;
                fullstop_window_count_++;
            }
        }

        speed_pid_controller_.Reset_integral();
        station_pid_controller_.Reset_integral();
    }

    if (FLAGS_use_vehicle_epb) {
        ADEBUG << "Into use vehicle epb.";
        if ((debug->is_full_stop() && IsFullStopLongTerm(debug))
            || (debug->is_full_stop() && debug->is_stop_reason_by_prdestrian())) {
            cmd->set_parking_brake(true);
            if (chassis->parking_brake()) {
                brake_cmd = 0.0;
                ResetNewFullStop();
            }
        } else {
            cmd->set_parking_brake(false);
            if (chassis->parking_brake()) {
                ADEBUG << "Into park brake release.";
                SetParkingBrake(&current_lon_based_pidcontroller_conf_, cmd);
            }
        }
    }

    // if the car is driven by acceleration, disgard the cmd->throttle and brake

    bool is_previous_replan = injector_->previous_control_debug_mutable()->mutable_replan_debug()->is_need_replan();
    if (is_previous_replan) {
        ADEBUG << "NEED REPLAN";
    }
    if ((trajectory_message_->trajectory_type() != ADCTrajectory::OPEN_SPACE) || debug->is_full_stop()
        || debug->is_full_stop_soft() || is_previous_replan) {
        openspace_speed_control_throttle_cmd_ = 0.0;
        openspace_count_ = 0;
    }

    if ((trajectory_message_->trajectory_type() == ADCTrajectory::OPEN_SPACE)
        && current_lon_based_pidcontroller_conf_.use_parking_speed_control()
        && std::fabs(debug->path_remain()) > current_lon_based_pidcontroller_conf_.speed_path_remain()) {
        if (((chassis->gear_location() == canbus::Chassis::GEAR_REVERSE)
             && (debug->acceleration_reference()
                 <= (-1) * current_lon_based_pidcontroller_conf_.speed_itfc_dcc_emergency()))
            || ((chassis->gear_location() == canbus::Chassis::GEAR_DRIVE)
                && (debug->acceleration_reference()
                    >= current_lon_based_pidcontroller_conf_.speed_itfc_dcc_emergency()))) {
            // use speed control
            ADEBUG << "=======use speed control.";
            openspace_count_++;
            if (std::fabs(current_speed) <= current_lon_based_pidcontroller_conf_.openspace_r_std_v()) {
                openspace_speed_control_throttle_cmd_ = openspace_speed_control_throttle_cmd_
                        + current_lon_based_pidcontroller_conf_.openspace_cmd_change();
            }
            if (std::fabs(current_speed) >= current_lon_based_pidcontroller_conf_.openspace_r_std_v()
                        + current_lon_based_pidcontroller_conf_.openspace_r_err()) {
                if ((openspace_speed_control_throttle_cmd_
                     >= current_lon_based_pidcontroller_conf_.openspace_start_throttle())
                    && current_lon_based_pidcontroller_conf_.use_maintain_throttle()) {
                    openspace_speed_control_throttle_cmd_
                            = current_lon_based_pidcontroller_conf_.openspace_maintain_throttle();
                } else {
                    openspace_speed_control_throttle_cmd_ = openspace_speed_control_throttle_cmd_
                            - current_lon_based_pidcontroller_conf_.openspace_cmd_change();
                }
            }
            if ((std::fabs(current_speed) >= current_lon_based_pidcontroller_conf_.openspace_r_std_v()
                         - current_lon_based_pidcontroller_conf_.openspace_r_err())
                && (std::fabs(current_speed) <= current_lon_based_pidcontroller_conf_.openspace_r_std_v()
                            + current_lon_based_pidcontroller_conf_.openspace_r_err())
                && current_lon_based_pidcontroller_conf_.use_maintain_throttle()) {
                openspace_speed_control_throttle_cmd_
                        = current_lon_based_pidcontroller_conf_.openspace_maintain_throttle();
            }
            if (vehicle_pitch <= current_lon_based_pidcontroller_conf_.speed_control_pitch_thres()
                && openspace_speed_control_throttle_cmd_ < current_lon_based_pidcontroller_conf_.pitch_start_throttle()
                && std::fabs(current_speed) < current_lon_based_pidcontroller_conf_.speed_control_min_speed()
                && openspace_count_ < current_lon_based_pidcontroller_conf_.openspace_start_num()) {
                openspace_speed_control_brake_cmd_ = current_lon_based_pidcontroller_conf_.epb_brake();
            } else {
                if (std::fabs(current_speed) >= current_lon_based_pidcontroller_conf_.openspace_r_std_v()
                            + current_lon_based_pidcontroller_conf_.openspace_r_err_big()) {
                    openspace_speed_control_brake_cmd_ = openspace_speed_control_brake_cmd_
                            + current_lon_based_pidcontroller_conf_.openspace_brk_change();
                    if (openspace_speed_control_brake_cmd_
                        > current_lon_based_pidcontroller_conf_.openspace_max_brk()) {
                        openspace_speed_control_brake_cmd_ = current_lon_based_pidcontroller_conf_.openspace_max_brk();
                    }
                } else {
                    openspace_speed_control_brake_cmd_ = 0.0;
                }
            }

            if (openspace_speed_control_throttle_cmd_ <= 0.0) {
                openspace_speed_control_throttle_cmd_ = 0.0;
            }

            brake_cmd = openspace_speed_control_brake_cmd_;
            if (brake_cmd > 0.0) {
                throttle_cmd = 0.0;
            } else {
                throttle_cmd = openspace_speed_control_throttle_cmd_;
            }
        }
    }

    if (trajectory_message_->trajectory_type() == ADCTrajectory::OPEN_SPACE) {
        if (brake_cmd > current_lon_based_pidcontroller_conf_.full_stop_max_brake()) {
            brake_cmd = current_lon_based_pidcontroller_conf_.full_stop_max_brake();
        }
    }

    // a logic to keep car safe,when the chassis gear is different with traj gear.
    if (chassis->gear_location() != trajectory_message_->gear()) {
        speed_pid_controller_.Reset_integral();
        station_pid_controller_.Reset_integral();
        throttle_cmd = 0.0;
        brake_cmd = current_lon_based_pidcontroller_conf_.epb_brake();
    }

    debug->set_throttle_cmd(throttle_cmd);
    debug->set_brake_cmd(brake_cmd);

    throttle_cmd = ThrottleCmdFilter(FLAGS_use_throttle_filter, FLAGS_throttle_smoothing_factor, throttle_cmd);
    lon_debug_info->set_throttle_cmd_filter(throttle_cmd);

    cmd->set_throttle(throttle_cmd);
    cmd->set_brake(brake_cmd);
    if (current_lon_based_pidcontroller_conf_.use_acceleration_lookup_limit()) {
        cmd->set_acceleration(acceleration_lookup_limit);
        previous_acc_output_ = acceleration_lookup_limit;
    } else {
        cmd->set_acceleration(acceleration_cmd);
        previous_acc_output_ = acceleration_cmd;
    }

    if (std::fabs(current_speed) <= vehicle_param_.max_abs_speed_when_stopped()
        || chassis->gear_location() == trajectory_message_->gear()
        || chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL) {
        cmd->set_gear_location(trajectory_message_->gear());
    } else {
        cmd->set_gear_location(chassis->gear_location());
    }

    if (current_lon_based_pidcontroller_conf_.use_speed_itfc()) {
        reference_spd_cmd_ = reference_spd_ + debug->speed_offset();
        if ((reference_spd_ <= current_lon_based_pidcontroller_conf_.speed_itfc_full_stop_speed())
            && (chassis->gear_location() == canbus::Chassis::GEAR_DRIVE)) {
            if ((debug->path_remain() >= current_lon_based_pidcontroller_conf_.speed_itfc_path_remain_min())
                && (debug->preview_acceleration_reference()
                    >= current_lon_based_pidcontroller_conf_.speed_itfc_dcc_emergency())
                && (debug->path_remain() <= current_lon_based_pidcontroller_conf_.speed_itfc_path_remain_max())) {
                if (debug->preview_acceleration_reference()
                    <= current_lon_based_pidcontroller_conf_.speed_itfc_acc_thres()) {
                    if (reference_spd_ > current_lon_based_pidcontroller_conf_.speed_itfc_stop_speed()) {
                        reference_spd_cmd_ = current_lon_based_pidcontroller_conf_.speed_itfc_speed_cmd();
                    }
                }
            }
        }
        cmd->set_speed(reference_spd_cmd_);
        if ((debug->is_full_stop() || debug->is_full_stop_soft())
            && trajectory_message_->trajectory_type() == ADCTrajectory::OPEN_SPACE) {
            cmd->set_speed(0.0);
        }
    }

    debug->set_station_error_limited(station_error_limited);
    debug->set_speed_offset(speed_offset);
    debug->set_speed_controller_input_limited(speed_controller_input_limited);
    debug->set_acceleration_cmd(acceleration_cmd);
    debug->set_acceleration_lookup(acceleration_lookup);
    debug->set_acceleration_lookup_limit(acceleration_lookup_limit);
    debug->set_speed_lookup(current_speed);
    debug->set_calibration_value(calibration_value);
    debug->set_acceleration_cmd_closeloop(acceleration_cmd_closeloop);

    lon_debug_info->set_path_remain_add(path_remain_add);
    lon_debug_info->set_max_path_remain_when_stopped(max_path_remain_when_stopped_);
    lon_debug_info->set_form_go_to_full_stop(form_go_to_full_stop_);
    lon_debug_info->set_begin_end_brake(begin_end_brake_);
    lon_debug_info->set_hillup_fullstop_brake_rate(hillup_fullstop_brake_rate_);
    lon_debug_info->set_control_task_name(name_);

    if (FLAGS_enable_csv_debug && speed_log_file_ != nullptr) {
        fprintf(speed_log_file_,
                "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f,"
                "%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %d,\r\n",
                debug->station_reference(),
                debug->station_error(),
                station_error_limited,
                debug->preview_station_error(),
                debug->speed_reference(),
                debug->speed_error(),
                speed_controller_input_limited,
                debug->preview_speed_reference(),
                debug->preview_speed_error(),
                debug->preview_acceleration_reference(),
                acceleration_cmd_closeloop,
                acceleration_cmd,
                debug->acceleration_lookup(),
                debug->acceleration_lookup_limit(),
                debug->speed_lookup(),
                calibration_value,
                throttle_cmd,
                brake_cmd,
                debug->is_full_stop());
    }

    return Status::OK();
}

Status LonPlusController::Reset() {
    speed_pid_controller_.Reset();
    station_pid_controller_.Reset();
    is_full_stop_wait_time_diff_ = 0;
    wait_time_diff_ = 0;
    from_finish_to_else_ = false;
    previous_is_finish_ = false;
    is_need_restart_pedestrian_init_time_ = true;
    return Status::OK();
}

std::string LonPlusController::Name() const {
    return name_;
}

void LonPlusController::ComputeLongitudinalErrors(
        const TrajectoryAnalyzer *trajectory_analyzer,
        const double preview_time,
        const double ts,
        const double vehicle_speed,
        SimpleLongitudinalDebug *debug) {
    // the decomposed vehicle motion onto Frenet frame
    // s: longitudinal accumulated distance along reference trajectory
    // s_dot: longitudinal velocity along reference trajectory
    // d: lateral distance w.r.t. reference trajectory
    // d_dot: lateral distance change rate, i.e. dd/dt
    double s_matched = 0.0;
    double s_dot_matched = 0.0;
    double d_matched = 0.0;
    double d_dot_matched = 0.0;

    auto vehicle_state = injector_->vehicle_state();
    auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(vehicle_state->x(), vehicle_state->y());

    trajectory_analyzer->ToTrajectoryFrame(
            vehicle_state->x(),
            vehicle_state->y(),
            vehicle_state->heading(),
            vehicle_speed,
            matched_point,
            &s_matched,
            &s_dot_matched,
            &d_matched,
            &d_dot_matched);

    // double current_control_time = Time::Now().ToSecond();
    double current_control_time
            = FLAGS_sim_by_record ? chassis_->header().timestamp_sec() : (::apollo::cyber::Clock::NowInSeconds());
    double preview_control_time = current_control_time + preview_time;

    TrajectoryPoint reference_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(current_control_time);
    TrajectoryPoint preview_point = trajectory_analyzer->QueryNearestPointByAbsoluteTime(preview_control_time);

    debug->mutable_current_matched_point()->mutable_path_point()->set_x(matched_point.x());
    debug->mutable_current_matched_point()->mutable_path_point()->set_y(matched_point.y());
    debug->mutable_current_reference_point()->mutable_path_point()->set_x(reference_point.path_point().x());
    debug->mutable_current_reference_point()->mutable_path_point()->set_y(reference_point.path_point().y());
    debug->mutable_preview_reference_point()->mutable_path_point()->set_x(preview_point.path_point().x());
    debug->mutable_preview_reference_point()->mutable_path_point()->set_y(preview_point.path_point().y());

    ADEBUG << "matched point:" << matched_point.DebugString();
    ADEBUG << "reference point:" << reference_point.DebugString();
    ADEBUG << "preview point:" << preview_point.DebugString();

    double heading_error = common::math::NormalizeAngle(vehicle_state->heading() - matched_point.theta());
    double lon_speed = vehicle_speed * std::cos(heading_error);
    double lon_acceleration = vehicle_state->linear_acceleration() * std::cos(heading_error);
    double one_minus_kappa_lat_error
            = 1 - reference_point.path_point().kappa() * vehicle_speed * std::sin(heading_error);

    debug->set_station_reference(reference_point.path_point().s());
    debug->set_current_station(s_matched);
    debug->set_station_error(reference_point.path_point().s() - s_matched);
    debug->set_speed_reference(reference_point.v());
    debug->set_current_speed(lon_speed);
    debug->set_speed_error(reference_point.v() - s_dot_matched);
    debug->set_acceleration_reference(reference_point.a());
    debug->set_current_acceleration(lon_acceleration);
    debug->set_acceleration_error(reference_point.a() - lon_acceleration / one_minus_kappa_lat_error);
    double jerk_reference = (debug->acceleration_reference() - previous_acceleration_reference_) / ts;
    double lon_jerk = (debug->current_acceleration() - previous_acceleration_) / ts;
    debug->set_jerk_reference(jerk_reference);
    debug->set_current_jerk(lon_jerk);
    debug->set_jerk_error(jerk_reference - lon_jerk / one_minus_kappa_lat_error);
    previous_acceleration_reference_ = debug->acceleration_reference();
    previous_acceleration_ = debug->current_acceleration();

    debug->set_preview_station_error(preview_point.path_point().s() - s_matched);
    debug->set_preview_speed_error(preview_point.v() - s_dot_matched);
    debug->set_preview_speed_reference(preview_point.v());
    debug->set_preview_acceleration_reference(preview_point.a());
    if (current_lon_based_pidcontroller_conf_.use_speed_itfc()) {
        reference_spd_ = reference_point.v();
    }
}

void LonPlusController::SetDigitalFilter(double ts, double cutoff_freq, common::DigitalFilter *digital_filter) {
    std::vector<double> denominators;
    std::vector<double> numerators;
    common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
    digital_filter->set_coefficients(denominators, numerators);
}

// TODO(all): Refactor and simplify
void LonPlusController::GetPathRemain(SimpleLongitudinalDebug *debug) {
    int stop_index = 0;
    static constexpr double kSpeedThreshold = 1e-3;
    static constexpr double kForwardAccThreshold = -1e-2;
    static constexpr double kBackwardAccThreshold = 1e-1;
    static constexpr double kParkingSpeed = 0.1;

    if (trajectory_message_->gear() == canbus::Chassis::GEAR_DRIVE) {
        while (stop_index < trajectory_message_->trajectory_point_size()) {
            auto &current_trajectory_point = trajectory_message_->trajectory_point(stop_index);
            if (std::fabs(current_trajectory_point.v()) < kSpeedThreshold
                && current_trajectory_point.a() > kForwardAccThreshold && current_trajectory_point.a() < 0.0) {
                break;
            }
            ++stop_index;
        }
    } else {
        while (stop_index < trajectory_message_->trajectory_point_size()) {
            auto &current_trajectory_point = trajectory_message_->trajectory_point(stop_index);
            if (current_trajectory_point.v() > -kSpeedThreshold && current_trajectory_point.a() < kBackwardAccThreshold
                && current_trajectory_point.a() > 0.0) {
                break;
            }
            ++stop_index;
        }
    }
    ADEBUG << "stop_index is, " << stop_index;
    if (stop_index == trajectory_message_->trajectory_point_size()) {
        --stop_index;
        if (std::fabs(trajectory_message_->trajectory_point(stop_index).v()) < kParkingSpeed) {
            ADEBUG << "the last point is selected as parking point";
        } else {
            ADEBUG << "the last point found in path and speed > speed_deadzone";
        }
    }
    debug->set_path_remain(
            trajectory_message_->trajectory_point(stop_index).path_point().s() - debug->current_station());
}

bool LonPlusController::IsStopByDestination(SimpleLongitudinalDebug *debug) {
    auto stop_reason = trajectory_message_->decision().main_decision().stop();
    ADEBUG << "Current stop reason is \n" << stop_reason.DebugString();
    ADEBUG << "Planning command status msg is \n" << injector_->get_planning_command_status()->ShortDebugString();

    StopReasonCode stop_reason_code = stop_reason.reason_code();

    if (stop_reason_code == StopReasonCode::STOP_REASON_SIGNAL
        || stop_reason_code == StopReasonCode::STOP_REASON_REFERENCE_END
        || injector_->get_planning_command_status()->status() == CommandStatusType::FINISHED
        || trajectory_message_->decision().main_decision().has_mission_complete()) {
        ADEBUG << "[IsStopByDestination]Current stop reason is in destination.";
        debug->set_is_stop_reason_by_destination(true);
        return true;
    }
    debug->set_is_stop_reason_by_destination(false);
    return false;
}

bool LonPlusController::IsPedestrianStopLongTerm(SimpleLongitudinalDebug *debug) {
    auto stop_reason = trajectory_message_->decision().main_decision().stop();
    ADEBUG << "Current stop reason is \n" << stop_reason.DebugString();
    StopReasonCode stop_reason_code = stop_reason.reason_code();
    // const auto control_debug_info = injector_->control_debug_info();

    if (stop_reason_code == StopReasonCode::STOP_REASON_PEDESTRIAN
        || stop_reason_code == StopReasonCode::STOP_REASON_OBSTACLE) {
        ADEBUG << "[IsPedestrianStopLongTerm]Stop reason for pedestrian.";
        is_stop_by_pedestrian_ = true;
    } else {
        is_stop_by_pedestrian_ = false;
    }

    ADEBUG << "Current is_stop_by_pedestrian: " << is_stop_by_pedestrian_
           << ", is_stop_by_pedestrian_previous: " << is_stop_by_pedestrian_previous_;
    ADEBUG << "is_need_restart_pedestrian_init_time_ is " << is_need_restart_pedestrian_init_time_;
    if (is_stop_by_pedestrian_) {
        if (!(is_stop_by_pedestrian_ && is_stop_by_pedestrian_previous_) || is_need_restart_pedestrian_init_time_
            || from_finish_to_else_) {
            start_time_ = ::apollo::cyber::Clock::NowInSeconds();
            ADEBUG << "Stop reason for pedestrian, start time(s) is " << start_time_;
            is_need_restart_pedestrian_init_time_ = false;
        } else {
            ADEBUG << "Last time stop is already pedestrian, skip start_time init.";
        }
        double end_time = ::apollo::cyber::Clock::NowInSeconds();
        ADEBUG << "Stop reason for pedestrian, current time(s) is " << end_time;
        wait_time_diff_ = end_time - start_time_;
    } else {
        start_time_ = 0.0;
        wait_time_diff_ = 0.0;
    }

    is_stop_by_pedestrian_previous_ = is_stop_by_pedestrian_;

    if (wait_time_diff_ > current_lon_based_pidcontroller_conf_.pedestrian_stop_time()) {
        ADEBUG << "Current pedestrian stop lasting time(s) is " << wait_time_diff_
               << ", larger than threshold: " << current_lon_based_pidcontroller_conf_.pedestrian_stop_time();
        debug->set_is_stop_reason_by_prdestrian(true);
        return true;
    } else {
        ADEBUG << "Current pedestrian stop lasting time(s) is " << wait_time_diff_
               << ", not reach the threshold: " << current_lon_based_pidcontroller_conf_.pedestrian_stop_time();
        debug->set_is_stop_reason_by_prdestrian(false);
        return false;
    }
}

bool LonPlusController::IsFullStopLongTerm(SimpleLongitudinalDebug *debug) {
    if (debug->is_full_stop()) {
        is_full_stop_wait_time_diff_++;
    } else {
        is_full_stop_wait_time_diff_ = 0;
    }

    if (is_full_stop_wait_time_diff_ > current_lon_based_pidcontroller_conf_.full_stop_long_time()) {
        ADEBUG << "Current full stop lasting time(s) is " << is_full_stop_wait_time_diff_
               << ", larger than threshold: " << current_lon_based_pidcontroller_conf_.full_stop_long_time();
        return true;
    } else {
        ADEBUG << "Current full stop lasting time(s) is " << is_full_stop_wait_time_diff_
               << ", not reach the threshold: " << current_lon_based_pidcontroller_conf_.full_stop_long_time();
        return false;
    }
}

void LonPlusController::SetParkingBrake(
        const LonBasedPidPlusControllerConf *conf,
        control::ControlCommand *control_command) {
    if (control_command->parking_brake()) {
        // epb on, parking brake: 0 -> 1
        if (epb_on_change_switch_) {
            ADEBUG << "Epb on, first set parking brake false.";
            control_command->set_parking_brake(false);
            ++epb_change_count_;
            if (epb_change_count_ >= conf->epb_change_count()) {
                epb_on_change_switch_ = false;
                epb_change_count_ = 0;
                ADEBUG << "Epb on, first stage has been done.";
            }
        } else {
            ADEBUG << "Epb on, second set parking brake true.";
            control_command->set_parking_brake(true);
            ++epb_change_count_;
            if (epb_change_count_ >= conf->epb_change_count()) {
                epb_on_change_switch_ = true;
                epb_change_count_ = 0;
                ADEBUG << "Epb on, second stage has been done.";
            }
        }
    } else {
        // epb off, parking brake: 1 -> 0
        if (epb_off_change_switch_) {
            ADEBUG << "Epb off, first set praking brake true.";
            control_command->set_parking_brake(true);
            ++epb_change_count_;
            if (epb_change_count_ >= conf->epb_change_count()) {
                epb_off_change_switch_ = false;
                epb_change_count_ = 0;
                ADEBUG << "Epb off, first stage has been done.";
            }
        } else {
            ADEBUG << "Epb off, second set parking brake false.";
            control_command->set_parking_brake(false);
            ++epb_change_count_;
            if (epb_change_count_ >= conf->epb_change_count()) {
                epb_off_change_switch_ = true;
                epb_change_count_ = 0;
                ADEBUG << "Epb off, second stage has been done.";
            }
        }
    }
}

void LonPlusController::CalcultePidParams(const double &speed, PidPlusConf &pid_plus_conf) {
    double kp = pid_plus_conf.pid_conf().kp()
            * InterpolationPlus1D::interpolation_1d(
                        speed, pid_plus_conf.speed_input(), pid_plus_conf.kp_speed_gain_output());
    pid_plus_conf.mutable_pid_conf()->set_kp(kp);
    double ki = pid_plus_conf.pid_conf().ki()
            * InterpolationPlus1D::interpolation_1d(
                        speed, pid_plus_conf.speed_input(), pid_plus_conf.ki_speed_gain_output());
    pid_plus_conf.mutable_pid_conf()->set_ki(ki);
    double kd = pid_plus_conf.pid_conf().kd()
            * InterpolationPlus1D::interpolation_1d(
                        speed, pid_plus_conf.speed_input(), pid_plus_conf.kd_speed_gain_output());
    pid_plus_conf.mutable_pid_conf()->set_kd(kd);
    ADEBUG << "Current pid params: kp: " << kp << ", ki: " << ki << ", kd: " << kd;
}

void LonPlusController::ResetNewFullStop() {
    previous_is_full_stop_ = false;
    form_go_to_full_stop_ = false;
    begin_end_brake_ = false;
    fullstop_in_brake_ = 0.0;
    fullstop_window_count_ = 0;
    hillup_fullstop_brake_rate_ = 0.0;
}

void LonPlusController::CheckFinishToElse(const planning::ADCTrajectory *trajectory_message) {
    if (previous_is_finish_
        && !(injector_->get_planning_command_status()->status() == CommandStatusType::FINISHED
             || trajectory_message->decision().main_decision().has_mission_complete())) {
        from_finish_to_else_ = true;
        ADEBUG << "From finish to else!!!";
    } else {
        from_finish_to_else_ = false;
    }
    ADEBUG << "from_finish_to_else_: " << from_finish_to_else_;
    if (injector_->get_planning_command_status()->status() == CommandStatusType::FINISHED
        || trajectory_message->decision().main_decision().has_mission_complete()) {
        previous_is_finish_ = true;
    } else {
        previous_is_finish_ = false;
    }
    ADEBUG << "previous_is_finish_: " << previous_is_finish_;
}

}  // namespace control
}  // namespace apollo
