/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/controllers/slope_anti_slip_control_task/anti_slip_control_task.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/control_component/common/control_gflags.h"
#include "absl/strings/str_cat.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using apollo::planning::ADCTrajectory;

constexpr double GRA_ACC = 9.8;

AntiSlipControlTask::AntiSlipControlTask() : name_("Slope Anti-Slip Control Task") {
    AINFO << "Using " << name_;
}

Status AntiSlipControlTask::Init(std::shared_ptr<DependencyInjector> injector) {
    // hook: Apollo License Verification: v_apollo_park
    if (!ControlTask::LoadConfig<AntiSlipControlTaskConf>(&antislip_control_task_conf_)) {
        AERROR << "Failed to load control task conf";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "Failed to load slop anti-slip control task conf");
    }
    if (!ControlTaskExtend::LoadParamsPipelineConfig<ParamsPipeline>(&params_pipeline_)) {
        AERROR << "failed to load anti params pipeline config";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load anti params pipeline config");
    }
    if (!LoadParams(&params_pipeline_)) {
        AERROR << "failed to load anti controller params.";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load anti anti controller params.");
    }
    if (!ControlTask::LoadCalibrationTable(&calibration_table_)) {
        AERROR << "failed to load calibration table";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "lon failed to load calibration table");
    }
    injector_ = injector;
    SetDigitalFilterPitchAngle();
    InitControlCalibrationTable();
    enable_acc_gain_ = antislip_control_task_conf_.enable_slope_offset();
    vehicle_param_.CopyFrom(common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
    hill_start_window_ = antislip_control_task_conf_.hill_start_preview_window();
    return Status::OK();
}

bool AntiSlipControlTask::LoadParams(ParamsPipeline *params_pipeline) {
    for (int i = 0; i < params_pipeline->params_declare_size(); i++) {
        AntiSlipControlTaskConf param_config;
        auto param_config_path = params_pipeline->params_declare(i).config_path();
        std::string param_config_path_relative = absl::StrCat("conf", "/", param_config_path);
        AINFO << "Param config_path_relative is " << param_config_path_relative;
        std::string param_config_path_absolute
                = apollo::cyber::plugin_manager::PluginManager::Instance()->GetPluginConfPath<ControlTask>(
                        "apollo::control::AntiSlipControlTask", param_config_path_relative);
        AINFO << "Param config_path_absolute is " << param_config_path_absolute;
        if (!apollo::cyber::common::GetProtoFromFile(param_config_path_absolute, &param_config)) {
            AERROR << "Load config: " << params_pipeline->params_declare(i).config_name() << " param failed!";
            return false;
        }
        AINFO << "Load the config file successfully, param file name: " << param_config_path;
        params_list_.push_back(std::make_pair(params_pipeline->params_declare(i).config_name(), param_config));
    }
    return true;
}

bool AntiSlipControlTask::SetCurrentParam(
        const std::string param_config_name,
        AntiSlipControlTaskConf &antislip_control_task_conf_) {
    bool set_success = false;
    for (const auto &param : params_list_) {
        if (param.first == param_config_name) {
            antislip_control_task_conf_.CopyFrom(param.second);
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

bool AntiSlipControlTask::ChooseCurrentParam(
        const planning::ADCTrajectory *trajectory_msg,
        AntiSlipControlTaskConf &antislip_control_task_conf_) {
    std::string param_config_name = "";

    if (trajectory_msg->trajectory_type() == ADCTrajectory::EDGE_FOLLOW) {
        param_config_name = "edge_follow_param";
    } else {
        param_config_name = "public_road_param";
    }

    if (!SetCurrentParam(param_config_name, antislip_control_task_conf_)) {
        AERROR << "Failed to set current anti param config: " << param_config_name;
        return false;
    } else {
        if (param_config_name != current_param_config_name_) {
            AINFO << "Update the current anti param config name is " << param_config_name;
            current_param_config_name_ = param_config_name;
        }
        return true;
    }
}

Status AntiSlipControlTask::ComputeControlCommand(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis,
        const planning::ADCTrajectory *planning_published_trajectory,
        ControlCommand *cmd) {
    trajectory_message_ = planning_published_trajectory;
    if (planning_published_trajectory->trajectory_type() == ADCTrajectory::OPEN_SPACE) {
        // if reverse,dont use this task
        return Status::OK();
    }

    if (!ChooseCurrentParam(trajectory_message_, antislip_control_task_conf_)) {
        AERROR << "Fail to load current anti control param!";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Fail to load current anti control param!");
    }

    uphill_too_fast_ = false;
    only_use_brk_downhill_ = false;
    erase_brk_in_go_ = false;
    erase_brk_in_end_ = false;

    // speed filter
    double current_speed = GetVehicleSpeed(injector_, FLAGS_use_speed_filter, FLAGS_speed_smoothing_factor);

    auto anti_slip_debug_info = injector_->mutable_control_debug_info()->mutable_simple_anti_slope_debug();
    anti_slip_debug_info->Clear();
    if (!previous_is_auto_ && chassis->driving_mode() == apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        from_else_to_auto_ = true;
        ADEBUG << "From else to auto!!!";
    } else {
        from_else_to_auto_ = false;
    }
    if (chassis->driving_mode() == apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE
        && gear_drive_count_ > antislip_control_task_conf_.gear_drive_count_num()) {
        previous_is_auto_ = true;
    } else {
        previous_is_auto_ = false;
    }

    if (chassis->gear_location() != canbus::Chassis::GEAR_DRIVE) {
        gear_drive_count_ = 0;
        previous_is_auto_ = false;
    } else {
        gear_drive_count_++;
    }

    auto lon_debug = cmd->mutable_debug()->mutable_simple_lon_debug();
    auto lon_debug_previous = injector_->Get_previous_lon_debug_info();
    double throttle_cmd = cmd->throttle();
    double throttle_lowerbound = vehicle_param_.throttle_deadzone();
    double brake_cmd = cmd->brake();
    double brake_lowerbound = vehicle_param_.brake_deadzone();
    if (brake_lowerbound <= antislip_control_task_conf_.brake_minimum_action()) {
        brake_lowerbound = antislip_control_task_conf_.brake_minimum_action();
    }
    double calibration_value = 0.0;

    double max_path_remain_when_stopped
            = FLAGS_max_path_remain_when_stopped + antislip_control_task_conf_.path_remain_threshold();

    double vehicle_pitch_rad = injector_->vehicle_state()->pitch();
    double vehicle_pitch = vehicle_pitch_rad * 180 / M_PI + FLAGS_pitch_offset_deg;
    lon_debug->set_vehicle_pitch(vehicle_pitch);
    FindMaxSpeedInPreviewWindow(planning_published_trajectory);
    FindMinSpeedInPreviewWindow(planning_published_trajectory);
    if (CheckSlope(vehicle_pitch)) {
        int reverse_flag = 1;
        if (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE) {
            reverse_flag = -1;
        }
        ADEBUG << "Into anti-slip stop process.";
        if (((CheckAccGainPoint(max_path_remain_when_stopped, lon_debug, lon_debug_previous)
              != AntiSlipControlTaskConf::MOVING_DEFAULT_DESTNATION)
             && !lon_debug->is_full_stop() && !lon_debug->is_full_stop_soft())
            || enable_acc_gain_) {
            ADEBUG << "is_full_stop is " << lon_debug->is_full_stop();
            ADEBUG << "Into acc gain process.";
            // TODO(ALL): confirm the slope_offset_compensation whether is positive or
            // not when vehicle move uphill Resume: uphill: + , downhill: -
            double slope_offset_compensation = antislip_control_task_conf_.use_opposite_slope_compensation() * GRA_ACC
                    * std::sin(vehicle_pitch_rad + FLAGS_pitch_offset_deg * M_PI / 180);
            if (reverse_flag == 1 && lon_debug->speed_error() < antislip_control_task_conf_.cut_hill_up_throttle_speed()
                && vehicle_pitch > 0) {
                slope_offset_compensation
                        = slope_offset_compensation * antislip_control_task_conf_.cut_hill_up_throttle_rate();
                // uphill forward too fast
                uphill_too_fast_ = true;
            } else if (
                    reverse_flag == -1
                    && lon_debug->speed_error() > -1 * antislip_control_task_conf_.cut_hill_up_throttle_speed()
                    && vehicle_pitch < 0) {
                slope_offset_compensation
                        = slope_offset_compensation * antislip_control_task_conf_.cut_hill_up_throttle_rate();
                // uphill backward too fast
                uphill_too_fast_ = true;
            }
            if (std::isnan(slope_offset_compensation)) {
                slope_offset_compensation = 0;
            }
            lon_debug->set_slope_offset_compensation(slope_offset_compensation);
            anti_slip_debug_info->set_slope_offset_compensation(slope_offset_compensation);
            double acceleration_cmd = lon_debug->acceleration_cmd_closeloop()
                    + lon_debug->preview_acceleration_reference()
                    + antislip_control_task_conf_.enable_slope_offset() * slope_offset_compensation;
            ADEBUG << "[acc gain process] acceleration_cmd gain to: " << acceleration_cmd;
            ADEBUG << "[acc gain process] close loop: " << lon_debug->acceleration_cmd_closeloop();
            ADEBUG << "[acc gain process] acceleration_preview_acceleration_reference "
                      "to: "
                   << lon_debug->preview_acceleration_reference();
            ADEBUG << "[acc gain process] acceleration_compensation_ to: "
                   << antislip_control_task_conf_.enable_slope_offset() * slope_offset_compensation;
            ADEBUG << "[acc gain process] compensation_ to: " << slope_offset_compensation;
            ADEBUG << "[acc gain process] pitch =  " << vehicle_pitch;

            double acceleration_lookup = (chassis->gear_location() == canbus::Chassis::GEAR_REVERSE) ? -acceleration_cmd
                                                                                                     : acceleration_cmd;
            anti_slip_debug_info->set_anti_acc_lookup(acceleration_lookup);
            // acceleration_lookup > 0: THROTTLE
            // acceleration_lookup < 0: BRAKE
            calibration_value
                    = control_interpolation_->Interpolate(std::make_pair(chassis->speed_mps(), acceleration_lookup));
            anti_slip_debug_info->set_calibration_value_anti(calibration_value);
            if (acceleration_lookup >= 0) {
                if (calibration_value >= 0) {
                    throttle_cmd = std::max(calibration_value, throttle_lowerbound);
                    anti_stage_ = "acc lookup and cali > 0, stage1";
                } else {
                    throttle_cmd = throttle_lowerbound;
                    anti_stage_ = "acc lookup > 0 ,cali < 0, stage2";
                }
                brake_cmd = 0.0;
                previous_downhill_go_brk_ = 0.0;
                pre_cali_down_hill_ = 0.0;
            } else {
                // the car is going downhill,acceleration_lookup < 0DetectObstacle
                throttle_cmd = 0.0;
                if (calibration_value >= 0) {
                    brake_cmd = brake_lowerbound;
                    throttle_cmd = 0.0;
                    previous_downhill_go_brk_ = 0.0;
                    pre_cali_down_hill_ = 0.0;
                    anti_stage_ = "acc lookup < 0 and cali > 0, stage3";
                } else {
                    // use only brake to go downhill
                    only_use_brk_downhill_ = true;
                    if ((std::fabs(lon_debug->acceleration_cmd_closeloop()) < std::fabs(slope_offset_compensation))
                        && (vehicle_pitch * reverse_flag < 0)) {
                        brake_cmd = std::max(-calibration_value, brake_lowerbound);

                        if (reverse_flag == 1 && (vehicle_pitch < 0)) {
                            // TODO:
                            // it is a method to avoid the being too fast or too slow,but it can be cancled ,cause it
                            // may stop the car suddendly

                            // if (lon_debug->speed_error() > antislip_control_task_conf_.downhill_cut_brake_v_err()) {
                            //     // should kill brake ,too slow
                            //     previous_downhill_go_brk_ = previous_downhill_go_brk_
                            //             - antislip_control_task_conf_.downhill_brake_change();
                            // }
                            // if (lon_debug->speed_error()
                            //     < (-1) * antislip_control_task_conf_.downhill_cut_brake_v_err()) {
                            //     // should add brake,car is too fast
                            //     previous_downhill_go_brk_ = previous_downhill_go_brk_
                            //             + antislip_control_task_conf_.downhill_brake_change();
                            // }
                            // if ((lon_debug->speed_error() <= antislip_control_task_conf_.downhill_cut_brake_v_err())
                            //     && (lon_debug->speed_error()
                            //         >= (-1) * antislip_control_task_conf_.downhill_cut_brake_v_err())) {
                            //     previous_downhill_go_brk_ = previous_downhill_go_brk_;
                            // }

                            // else {
                            //     brake_cmd = brake_cmd + previous_downhill_go_brk_;
                            // }
                            if (std::fabs(current_speed) < antislip_control_task_conf_.maintain_min_speed()) {
                                brake_cmd = 0.0;
                                anti_stage_ = "acc lookup < 0  and cali < 0, stage4";
                            }
                            if (lon_debug->speed_error()
                                < -1 * antislip_control_task_conf_.downhill_cut_brake_v_err()) {
                                if (brake_cmd - pre_cali_down_hill_
                                    > antislip_control_task_conf_.downhill_brake_change_max()) {
                                    brake_cmd = pre_cali_down_hill_
                                            + 2 * antislip_control_task_conf_.downhill_brake_change_max();
                                }
                            } else {
                                if (brake_cmd - pre_cali_down_hill_
                                    > antislip_control_task_conf_.downhill_brake_change_max()) {
                                    brake_cmd = pre_cali_down_hill_
                                            + antislip_control_task_conf_.downhill_brake_change_max();
                                }
                            }
                            if (brake_cmd == 0.0) {
                                pre_cali_down_hill_
                                        = pre_cali_down_hill_ * antislip_control_task_conf_.downhill_brake_rate();
                            } else {
                                pre_cali_down_hill_ = brake_cmd;
                            }
                        }

                        // TODO: the method when car go reversely
                        //  if (reverse_flag == -1 && (vehicle_pitch > 0)) {
                        //      if (lon_debug->speed_error()
                        //          < (-1) * antislip_control_task_conf_.downhill_cut_brake_v_err()) {
                        //          brake_cmd = previous_downhill_go_brk_
                        //                  - antislip_control_task_conf_.downhill_brake_change();
                        //      }
                        //      if (lon_debug->speed_error() > antislip_control_task_conf_.downhill_cut_brake_v_err()) {
                        //          brake_cmd = previous_downhill_go_brk_
                        //                  + antislip_control_task_conf_.downhill_brake_change();
                        //      }
                        //      if ((lon_debug->speed_error() <= antislip_control_task_conf_.downhill_cut_brake_v_err())
                        //          && (lon_debug->speed_error()
                        //              >= (-1) * antislip_control_task_conf_.downhill_cut_brake_v_err())) {
                        //          brake_cmd = previous_downhill_go_brk_;
                        //      }
                        //  }
                    }

                    if (brake_cmd <= 0) {
                        anti_stage_ = "brk < 0, set brk 0 stage5";
                        brake_cmd = 0.0;
                    }
                    ADEBUG << "usr brk to go,cmd = " << brake_cmd << "calibration_value = " << calibration_value;
                    ADEBUG << "acceleration_lookup = " << acceleration_lookup;
                    // previous_downhill_go_brk_ = brake_cmd;
                }
            }

            if ((std::fabs(current_speed) < antislip_control_task_conf_.maintain_min_speed())
                && (lon_debug->speed_error() > antislip_control_task_conf_.cut_unusual_brk_speed_error())) {
                if ((!in_hill_start_condition_) && (!in_normal_start_condition_) && (!on_obstacle_)
                    && (planning_published_trajectory->trajectory_type() != ADCTrajectory::OPEN_SPACE)) {
                    brake_cmd = 0.0;
                    erase_brk_in_go_ = true;
                }
            }

            if ((vehicle_pitch <= antislip_control_task_conf_.uphill_start_pitch())
                && (vehicle_pitch >= antislip_control_task_conf_.normal_start_pitch())
                && (planning_published_trajectory->trajectory_type() != ADCTrajectory::OPEN_SPACE)) {
                if (lon_debug->path_remain() > antislip_control_task_conf_.cncl_brake_min_path()
                    && lon_debug->path_remain() <= antislip_control_task_conf_.cncl_brake_max_path()) {
                    if ((!in_hill_start_condition_) && (!in_normal_start_condition_) && (!on_obstacle_)
                        && (min_speed_preview_ < antislip_control_task_conf_.preview_min_speed_threshold())
                        && (lon_debug->preview_acceleration_reference()
                            > antislip_control_task_conf_.emergency_acc())) {
                        if (lon_debug->speed_error() < antislip_control_task_conf_.cut_hill_up_throttle_speed()) {
                            if (cmd->brake() > antislip_control_task_conf_.cncl_max_brake()) {
                                brake_cmd = antislip_control_task_conf_.cncl_max_brake();
                            }
                        } else {
                            brake_cmd = 0.0;
                            erase_brk_in_end_ = true;
                        }
                    }
                }
            }

            if ((!in_hill_start_condition_) && (!in_normal_start_condition_)
                && DetectHillStartCondition(cmd, planning_published_trajectory, current_speed)) {
                ADEBUG << "in hill start condition,pitch = " << vehicle_pitch;
                if (vehicle_pitch >= antislip_control_task_conf_.uphill_start_pitch()
                    && antislip_control_task_conf_.enable_hill_start()) {
                    ADEBUG << "uphill!!!!!!  start condition";
                    in_hill_start_condition_ = true;
                    CalculateHillStartACCAndBrakeRate(cmd, planning_published_trajectory);
                    previous_hill_start_acc_ = hill_start_first_acc_;
                    previous_hill_start_brake_ = antislip_control_task_conf_.epb_brake_cmd();
                }
                if ((vehicle_pitch >= antislip_control_task_conf_.normal_start_pitch())
                    && (vehicle_pitch < antislip_control_task_conf_.uphill_start_pitch())
                    && antislip_control_task_conf_.enable_normal_start()) {
                    in_normal_start_condition_ = true;
                    previous_hill_start_acc_ = hill_start_first_acc_;
                    ADEBUG << "in normal start condition,normal start !!!!!!!!";
                    CalculateNormalStartACC(cmd);
                }
            } else {
                if (FLAGS_use_vehicle_epb) {
                    UseVehicleEpb(chassis, cmd);
                    ADEBUG << "[anti-slip stop in condition]: Into vehicle epb, parking "
                              "brake true.";
                } else if (lon_debug->is_full_stop_soft() || lon_debug->is_full_stop()) {
                    brake_cmd = std::max(brake_cmd, antislip_control_task_conf_.epb_brake_cmd());
                    ADEBUG << "[anti-slip stop in condition]: Into non vehicle epb, "
                              "throttle cmd is 0, brake cmd is "
                           << brake_cmd;
                    lon_debug->set_is_epb_brake(true);
                    ADEBUG << "anti slip brake triggered in slope: " << lon_debug->is_epb_brake();
                } else {
                    lon_debug->set_is_epb_brake(false);
                    ADEBUG << "[anti-slip stop out condition]: anti slip brake is not "
                              "triggered in slope.";
                }
            }

            if (in_hill_start_condition_) {
                if (chassis->gear_location() == canbus::Chassis::GEAR_DRIVE && !chassis->parking_brake()) {
                    // hill start control begin:
                    if (hill_start_acc_count_ > antislip_control_task_conf_.hill_start_acc_window()) {
                        hill_start_acc_gain_rate_ = 0;
                        ADEBUG << "flag over  set acc gain to 0";
                    }
                    slope_start_acc_ = hill_start_acc_gain_rate_ + previous_hill_start_acc_;
                    previous_hill_start_acc_ = slope_start_acc_;
                    double hill_start_throttle = control_interpolation_->Interpolate(
                            std::make_pair(chassis->speed_mps(), slope_start_acc_));
                    throttle_cmd = std::max(hill_start_throttle, throttle_lowerbound);

                    if (hill_start_brake_count_ >= antislip_control_task_conf_.hill_start_brake_window()) {
                        brake_cmd = 0.0;
                    } else if (
                            hill_start_brake_count_ >= antislip_control_task_conf_.hill_start_brake_standby_window()) {
                        brake_cmd = previous_hill_start_brake_ - hill_start_brake_dec_rate_;
                        previous_hill_start_brake_ = brake_cmd;
                    } else {
                        brake_cmd = antislip_control_task_conf_.epb_brake_cmd();
                    }
                    hill_start_acc_count_++;
                    hill_start_brake_count_++;
                }
                if (chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL || chassis->parking_brake()) {
                    brake_cmd = antislip_control_task_conf_.epb_brake_cmd();
                }
            }

            if (in_normal_start_condition_) {
                if (chassis->gear_location() == canbus::Chassis::GEAR_DRIVE && !chassis->parking_brake()) {
                    brake_cmd = 0.0;
                    if (normal_start_gain_count_ >= antislip_control_task_conf_.normal_start_gain_window()) {
                        normal_start_acc_gain_rate_ = 0.0;
                    }
                    slope_start_acc_ = normal_start_acc_gain_rate_ + previous_hill_start_acc_;
                    if (slope_start_acc_ < 0) {
                        slope_start_acc_ = 0.0;
                    }
                    if (slope_start_acc_ >= normal_start_tartget_acc_) {
                        slope_start_acc_ = normal_start_tartget_acc_;
                    }
                    previous_hill_start_acc_ = slope_start_acc_;
                    double hill_start_throttle = control_interpolation_->Interpolate(
                            std::make_pair(chassis->speed_mps(), slope_start_acc_));
                    throttle_cmd = std::max(hill_start_throttle, throttle_lowerbound)
                            * antislip_control_task_conf_.normal_acc_rate();
                    ADEBUG << "set normal throttle_cmd" << throttle_cmd;
                    ADEBUG << "now acc = " << slope_start_acc_;
                    normal_start_gain_count_++;
                    normal_start_maintain_count_++;
                }
            }

            if (DetectQuitHillStart(cmd, current_speed)) {
                ADEBUG << "quit hill start,clear flags";
                in_hill_start_condition_ = false;
                in_normal_start_condition_ = false;
                hill_start_acc_count_ = 0;
                hill_start_brake_count_ = 0;
                previous_hill_start_acc_ = 0.0;
                previous_hill_start_brake_ = 0.0;
                hill_start_acc_gain_rate_ = 0.0;
                hill_start_brake_dec_rate_ = 0.0;
                quit_hill_start_speed_ = 0.0;
                hill_start_first_acc_ = 0.0;
                normal_start_tartget_acc_ = 0.0;
                normal_start_gain_count_ = 0;
                normal_start_maintain_count_ = 0;
                normal_start_acc_gain_rate_ = 0.0;
                preview_length_ = 0;
                slope_start_acc_ = 0.0;
            }

            ADEBUG << "out put acc = " << acceleration_cmd;
            lon_debug->set_acceleration_cmd(acceleration_cmd);
            lon_debug->set_acceleration_lookup(acceleration_lookup);
            if (!lon_debug->is_full_stop() && !lon_debug->is_full_stop_soft()) {
                throttle_cmd
                        = ThrottleCmdFilter(FLAGS_use_throttle_filter, FLAGS_throttle_smoothing_factor, throttle_cmd);
                cmd->set_throttle(throttle_cmd);
                cmd->set_brake(brake_cmd);
            } else {
                previous_is_auto_ = false;
            }
        }
    } else {
        ResetHillStart();
        ADEBUG << "Not into anti-slip stop process because out of enable threshold: "
               << antislip_control_task_conf_.slope_offset_threhold();
    }
    DetectObstacle(cmd, chassis, current_speed);
    if (on_obstacle_) {
        quit_obstacle_count_++;
        ADEBUG << "[obstacle] overcome obstacle,throttle = " << previous_obs_throttle_;
        previous_obs_throttle_ = previous_obs_throttle_ + antislip_control_task_conf_.obs_throttle_add_rate();
        if (!lon_debug->is_full_stop() && !lon_debug->is_full_stop_soft()) {
            // previous_obs_throttle_ = ThrottleCmdFilter(FLAGS_use_throttle_filter, FLAGS_throttle_smoothing_factor,
            // previous_obs_throttle_);
            if (previous_obs_throttle_ > 99.0) {
                previous_obs_throttle_ = 99.0;
            }
            cmd->set_throttle(previous_obs_throttle_);
        }
    }
    previous_park_brake_ = cmd->parking_brake();

    if (chassis->gear_location() == canbus::Chassis::GEAR_NEUTRAL
        || (chassis->gear_location() != trajectory_message_->gear())) {
        brake_cmd = antislip_control_task_conf_.epb_brake_cmd();
        throttle_cmd = 0.0;
        cmd->set_throttle(throttle_cmd);
        cmd->set_brake(brake_cmd);
    }

    auto control_interactive_info = injector_->mutable_control_interactive_info();
    if (antislip_control_task_conf_.judge_safety_check()) {
        if (lon_debug->is_full_stop() || lon_debug->is_full_stop_soft()
            || (lon_debug->acceleration_reference() > antislip_control_task_conf_.cancle_safety_check_acc()
                && current_speed < antislip_control_task_conf_.cancle_safety_check_speed())
            || (std::fabs(vehicle_pitch) > antislip_control_task_conf_.cancle_safety_check_pitch())) {
            // situations dont use safety check
            // 0 use cancle fullstop
            // 1 fullstop(soft)
            // 2 accpreviwe > thres && v < thres(car is starting)
            // 3 vehicle pitch toobig toosmall
            control_interactive_info->set_is_need_safety_check(false);
        } else {
            control_interactive_info->set_is_need_safety_check(true);
        }
    } else {
        control_interactive_info->set_is_need_safety_check(true);
    }

    // set some debug info
    anti_slip_debug_info->set_hill_start_window(hill_start_window_);
    anti_slip_debug_info->set_preview_length(preview_length_);
    anti_slip_debug_info->set_quit_hill_start_speed(quit_hill_start_speed_);
    anti_slip_debug_info->set_normal_start_tartget_acc(normal_start_tartget_acc_);
    anti_slip_debug_info->set_in_hill_start_condition(in_hill_start_condition_);
    anti_slip_debug_info->set_in_normal_start_condition(in_normal_start_condition_);
    anti_slip_debug_info->set_hill_start_acc_gain_rate(hill_start_acc_gain_rate_);
    anti_slip_debug_info->set_normal_start_acc_gain_rate(normal_start_acc_gain_rate_);
    anti_slip_debug_info->set_on_obstacle(on_obstacle_);
    anti_slip_debug_info->set_from_else_to_auto(from_else_to_auto_);
    anti_slip_debug_info->set_previous_is_auto(previous_is_auto_);
    anti_slip_debug_info->set_uphill_too_fast(uphill_too_fast_);
    anti_slip_debug_info->set_only_use_brk_downhill(only_use_brk_downhill_);
    anti_slip_debug_info->set_erase_brk_in_go(erase_brk_in_go_);
    anti_slip_debug_info->set_erase_brk_in_end(erase_brk_in_end_);
    anti_slip_debug_info->set_anti_cmd_brake(brake_cmd);
    anti_slip_debug_info->set_anti_cmd_throttle(throttle_cmd);
    anti_slip_debug_info->set_hill_start_first_acc(hill_start_first_acc_);
    anti_slip_debug_info->set_slope_start_acc(slope_start_acc_);
    anti_slip_debug_info->set_anti_stage(anti_stage_);

    return Status::OK();
}

void AntiSlipControlTask::SetDigitalFilterPitchAngle() {
    double cutoff_freq = antislip_control_task_conf_.pitch_angle_filter_conf().cutoff_freq();
    double ts = antislip_control_task_conf_.ts();
    SetDigitalFilter(ts, cutoff_freq, &digital_filter_pitch_angle_);
}

void AntiSlipControlTask::SetDigitalFilter(double ts, double cutoff_freq, common::DigitalFilter *digital_filter) {
    std::vector<double> denominators;
    std::vector<double> numerators;
    common::LpfCoefficients(ts, cutoff_freq, &denominators, &numerators);
    digital_filter->set_coefficients(denominators, numerators);
}

void AntiSlipControlTask::InitControlCalibrationTable() {
    ADEBUG << "[Anti-slip] Control calibration table size is " << calibration_table_.calibration_size();
    Interpolation2D::DataType xyz;
    for (const auto &calibration : calibration_table_.calibration()) {
        xyz.push_back(std::make_tuple(calibration.speed(), calibration.acceleration(), calibration.command()));
    }
    control_interpolation_.reset(new Interpolation2D);
    ACHECK(control_interpolation_->Init(xyz)) << "Fail to load control calibration table";
}

bool AntiSlipControlTask::CheckSlope(double vehicle_pitch) {
    if (std::fabs(vehicle_pitch) >= antislip_control_task_conf_.slope_offset_threhold()) {
        ADEBUG << "Current vehicle is in slope.";
        return true;
    }

    return false;
}

void AntiSlipControlTask::UseVehicleEpb(const canbus::Chassis *chassis, ControlCommand *control_command) {
    auto lon_debug = control_command->mutable_debug()->mutable_simple_lon_debug();
    double brake_cmd = control_command->brake();
    if (previous_park_brake_ && !control_command->parking_brake()) {
        brake_cmd = antislip_control_task_conf_.epb_brake_cmd();
        is_brake_hold_ = true;
        ADEBUG << "is_brake_hold_ is true.";
    }
    if (is_brake_hold_ && (lon_debug->acceleration_lookup() > lon_debug->slope_offset_compensation())) {
        brake_cmd = 0.0;
        is_brake_hold_ = false;
        ADEBUG << "is_brake_hold_ is false.";
    }
    control_command->set_brake(brake_cmd);
}

bool AntiSlipControlTask::DetectHillStartCondition(
        ControlCommand *control_command,
        const planning::ADCTrajectory *planning_published_trajectory,
        const double &current_speed) {
    ADEBUG << "in detect hill start condition";
    auto lon_debug = control_command->mutable_debug()->mutable_simple_lon_debug();
    ADEBUG << "previous_park_brake_" << previous_park_brake_;
    ADEBUG << "control_command->parking_brake()" << control_command->parking_brake();
    ADEBUG << "lon_debug->vehicle_pitch()" << lon_debug->vehicle_pitch();
    if (((previous_park_brake_ && !control_command->parking_brake()) || from_else_to_auto_)
        && (lon_debug->vehicle_pitch() > antislip_control_task_conf_.hill_start_pitch_threshold())) {
        // command means that parking brake will be released,and the car is on a
        // hill if the follower conditions are true, the car will go into the hill
        // start condition
        ADEBUG << "traj size = " << planning_published_trajectory->trajectory_point_size();
        if (planning_published_trajectory->trajectory_point_size()
            > antislip_control_task_conf_.min_hill_start_hill_start_trajs()) {
            // ADEBUG << "size is over";
            ADEBUG << "v  is " << current_speed;
            ADEBUG << "max speed is " << quit_hill_start_speed_;
            if (quit_hill_start_speed_ > antislip_control_task_conf_.hill_start_min_speed()
                && current_speed < antislip_control_task_conf_.hill_start_still_speed()) {
                ADEBUG << "all is satisfied,in hill start";
                return true;
            }
        }
    }
    return false;
}

bool AntiSlipControlTask::DetectQuitHillStart(ControlCommand *cmd, const double &current_speed) {
    if ((!in_hill_start_condition_) && (!in_normal_start_condition_)) {
        ADEBUG << "not in hill start condition ,return";
        return false;
    }
    auto lon_debug = cmd->mutable_debug()->mutable_simple_lon_debug();
    ADEBUG << "speed  error = " << lon_debug->speed_error();
    ADEBUG << "detect if hill start speed is enough";
    if (((current_speed) >= quit_hill_start_speed_)
        || (hill_start_acc_count_ > antislip_control_task_conf_.hill_start_quit_window())
        || (normal_start_maintain_count_ > antislip_control_task_conf_.normal_start_maintain_window())
        || (lon_debug->speed_error() < antislip_control_task_conf_.quit_hill_start_v_err())) {
        return true;
    } else {
        ADEBUG << "speed not enough ,stay in hill start";
        return false;
    }
}

void AntiSlipControlTask::FindMaxSpeedInPreviewWindow(const planning::ADCTrajectory *planning_published_trajectory) {
    ADEBUG << "in find max speed ";
    int index = 0;
    preview_length_ = std::min(planning_published_trajectory->trajectory_point_size(), hill_start_window_);

    for (index = 0; index < preview_length_; index++) {
        quit_hill_start_speed_
                = std::max(planning_published_trajectory->trajectory_point(index).v(), quit_hill_start_speed_);
    }
    // normal_start_tartget_acc_ = quit_hill_start_speed_ / (preview_length_ * 0.1);
    if (preview_length_ > 50) {
        normal_start_tartget_acc_ = planning_published_trajectory->trajectory_point(49).v() / (49 * 0.02);
    } else {
        normal_start_tartget_acc_
                = planning_published_trajectory->trajectory_point(preview_length_ - 1).v() / (preview_length_ * 0.02);
    }
    ADEBUG << "max speed = " << quit_hill_start_speed_;
}

void AntiSlipControlTask::FindMinSpeedInPreviewWindow(const planning::ADCTrajectory *planning_published_trajectory) {
    ADEBUG << "in find min speed ";
    int index = 0;
    min_speed_preview_length_ = planning_published_trajectory->trajectory_point_size();

    for (index = 0; index < min_speed_preview_length_; index++) {
        min_speed_preview_ = std::min(planning_published_trajectory->trajectory_point(index).v(), min_speed_preview_);
    }
}

void AntiSlipControlTask::CalculateHillStartACCAndBrakeRate(
        ControlCommand *control_command,
        const planning::ADCTrajectory *planning_published_trajectory) {
    ADEBUG << "cal hill start acc and brake";
    auto lon_debug = control_command->mutable_debug()->mutable_simple_lon_debug();
    // window * ts * max_acc * 0.5 = quit_hill_start_speed,
    double max_real_acc = antislip_control_task_conf_.hill_up_acc_rate() * quit_hill_start_speed_
            / (hill_start_window_ * 0.5 * antislip_control_task_conf_.ts());
    ADEBUG << "max_real_acc" << max_real_acc;
    hill_start_first_acc_ = GRA_ACC * std::sin(std::fabs(lon_debug->vehicle_pitch()) * M_PI / 180)
            * antislip_control_task_conf_.gravity_hill_start_gain();
    ADEBUG << "hill_start_first_acc" << hill_start_first_acc_;

    hill_start_acc_gain_rate_ = max_real_acc / hill_start_window_;
    hill_start_brake_dec_rate_ = antislip_control_task_conf_.epb_brake_cmd()
            / (antislip_control_task_conf_.hill_start_brake_window()
               - antislip_control_task_conf_.hill_start_brake_standby_window());
    ADEBUG << "hill_start_acc_gain_rate" << hill_start_acc_gain_rate_;
    ADEBUG << "hill_start_brake_dec_rate" << hill_start_brake_dec_rate_;
}

void AntiSlipControlTask::CalculateNormalStartACC(ControlCommand *control_command) {
    ADEBUG << "cal normal start acc and brake";
    auto lon_debug = control_command->mutable_debug()->mutable_simple_lon_debug();
    hill_start_first_acc_ = GRA_ACC * std::sin(lon_debug->vehicle_pitch() * M_PI / 180)
            * antislip_control_task_conf_.gravity_hill_start_gain();
    if (hill_start_first_acc_ < 0) {
        hill_start_first_acc_ = 0.0;
    }

    normal_start_acc_gain_rate_ = (normal_start_tartget_acc_ + hill_start_first_acc_)
            / antislip_control_task_conf_.normal_start_gain_window();
}

AntiSlipControlTaskConf::VehicleMoveState AntiSlipControlTask::CheckAccGainPoint(
        double path_remain_threshold,
        SimpleLongitudinalDebug *lon_debug,
        const SimpleLongitudinalDebug *lon_debug_previous) {
    if ((std::fabs(lon_debug_previous->path_remain()) < path_remain_threshold)
        && (std::fabs(lon_debug->path_remain()) < path_remain_threshold)) {
        ADEBUG << "[CheckAccGainPoint] Current path remain is " << lon_debug->path_remain();
        ADEBUG << "[CheckAccGainPoint] CLOSING_DESTNATION";
        return AntiSlipControlTaskConf::CLOSING_DESTNATION;

    } else if (
            lon_debug_previous->path_remain() < path_remain_threshold
            && lon_debug->path_remain() > antislip_control_task_conf_.new_dest_path_remain_threshold()) {
        vehicle_start_up_ = true;
        start_path_remian_ = lon_debug->path_remain();
    } else if (vehicle_start_up_) {
        if (lon_debug->path_remain() > (start_path_remian_ - path_remain_threshold)
            && lon_debug->path_remain() < start_path_remian_) {
            ADEBUG << "[CheckAccGainPoint] Current path remain is " << lon_debug->path_remain();
            ADEBUG << "[CheckAccGainPoint] MOVING_START_POINT";
            return AntiSlipControlTaskConf::MOVING_START_POINT;
        } else {
            vehicle_start_up_ = false;
            start_path_remian_ = 0.0;
        }
    }

    ADEBUG << "[CheckAccGainPoint] MOVING_DEFAULT_DESTNATION";
    return AntiSlipControlTaskConf::MOVING_DEFAULT_DESTNATION;
}

void AntiSlipControlTask::ResetHillStart() {
    ADEBUG << "reset h start";
    in_hill_start_condition_ = false;
    in_normal_start_condition_ = false;
    hill_start_acc_count_ = 0;
    hill_start_brake_count_ = 0;
    previous_hill_start_acc_ = 0.0;
    previous_hill_start_brake_ = 0.0;
    hill_start_acc_gain_rate_ = 0.0;
    hill_start_brake_dec_rate_ = 0.0;
    hill_start_first_acc_ = 0.0;
    normal_start_tartget_acc_ = 0.0;
    normal_start_gain_count_ = 0;
    normal_start_maintain_count_ = 0;
    normal_start_acc_gain_rate_ = 0.0;
    preview_length_ = 0;
}

void AntiSlipControlTask::ResetDetectObstacleFlags() {
    unusual_stop_count_ = 0;
    quit_obstacle_count_ = 0;
    on_obstacle_ = false;
    previous_obs_throttle_ = 0;
    speed_stable_count_ = 0;
}
void AntiSlipControlTask::DetectObstacle(
        ControlCommand *cmd,
        const canbus::Chassis *chassis,
        const double &current_speed) {
    auto lon_debug = cmd->mutable_debug()->mutable_simple_lon_debug();

    // obstacal cant be activated when hill start or normal start
    if (in_hill_start_condition_ || in_normal_start_condition_
        || chassis->gear_location() == canbus::Chassis::GEAR_REVERSE) {
        ADEBUG << "[obstacle] in hill start condition or in reverse gear ,return";
        ResetDetectObstacleFlags();
        return;
    }

    ADEBUG << "[obstacle] in detect obstacle";
    // v > threshold, or acc > threshold,
    // or full stop, or brake > 0, than quit
    ADEBUG << "[obstacle] on obstacle,detect whether to quit or not";
    if (lon_debug->is_full_stop() || lon_debug->is_full_stop_soft()
        || quit_hill_start_speed_ < antislip_control_task_conf_.obstacle_speed_safety_threshold()
        || cmd->brake() > antislip_control_task_conf_.obstacle_brake_cmd_threshold()) {
        ADEBUG << "[obstacle] quit obstacle,safety check reason";
        ResetDetectObstacleFlags();
        return;
    }
    if (quit_obstacle_count_ > antislip_control_task_conf_.quit_opbstacle_window()) {
        ADEBUG << "[obstacle] quit obstacle";
        ResetDetectObstacleFlags();
        ADEBUG << "[obstacle] print quit reason:max window satisfied";
        return;
    }

    if (current_speed > antislip_control_task_conf_.quit_opbstacle_safety_high_speed()) {
        ADEBUG << "[obstacle] over high speed, quit obstacle at once";
        ResetDetectObstacleFlags();
        return;
    }

    if (current_speed > antislip_control_task_conf_.speed_stable()) {
        ADEBUG << "[obstacle] over stable speed, count = " << speed_stable_count_;
        speed_stable_count_++;
    } else {
        ADEBUG << "[obstacle] speed jump back to 0 ";
        speed_stable_count_ = 0;
    }

    if (speed_stable_count_ > antislip_control_task_conf_.speed_stable_window()) {
        ADEBUG << "[obstacle] speed is stable,quit";
        ResetDetectObstacleFlags();
    }

    if (chassis->throttle_percentage() > antislip_control_task_conf_.obstacle_throttle_cmd_threshold()
        && current_speed < antislip_control_task_conf_.obstacle_speed_safety_threshold()
        && injector_->vehicle_state()->linear_acceleration() < antislip_control_task_conf_.obstacle_acc_threshold()
        && !lon_debug->is_full_stop() && !lon_debug->is_full_stop_soft()
        && quit_hill_start_speed_ > antislip_control_task_conf_.in_obstacle_preview_speed()
        && (lon_debug->path_remain() >= antislip_control_task_conf_.cncl_brake_max_path())
        && (chassis->gear_location() == canbus::Chassis::GEAR_DRIVE)) {
        ADEBUG << "[obstacle] meet obstacle,count + " << unusual_stop_count_;
        unusual_stop_count_++;
    }

    if (unusual_stop_count_ >= antislip_control_task_conf_.obstacle_stop_window_threshold() && !on_obstacle_) {
        ADEBUG << "[obstacle]=========== =AL= =======begin overcomingggg obstacle ";
        on_obstacle_ = true;
        previous_obs_throttle_ = cmd->throttle() + antislip_control_task_conf_.obs_throttle_add_default();
        min_throttle_count_ = 0;
        speed_stable_count_ = 0;
    }
}

void AntiSlipControlTask::Stop() {}

Status AntiSlipControlTask::Reset() {
    previous_is_auto_ = false;
    in_hill_start_condition_ = false;
    in_normal_start_condition_ = false;
    hill_start_acc_count_ = 0;
    hill_start_brake_count_ = 0;
    previous_hill_start_acc_ = 0.0;
    previous_hill_start_brake_ = 0.0;
    hill_start_acc_gain_rate_ = 0.0;
    hill_start_brake_dec_rate_ = 0.0;
    quit_hill_start_speed_ = 0.0;
    hill_start_first_acc_ = 0.0;
    normal_start_tartget_acc_ = 0.0;
    normal_start_gain_count_ = 0;
    normal_start_maintain_count_ = 0;
    normal_start_acc_gain_rate_ = 0.0;
    preview_length_ = 0;
    slope_start_acc_ = 0.0;
    return Status::OK();
}

std::string AntiSlipControlTask::Name() const {
    return name_;
}

}  // namespace control
}  // namespace apollo
