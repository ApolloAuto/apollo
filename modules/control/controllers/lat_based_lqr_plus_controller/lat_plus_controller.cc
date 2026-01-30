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

#include "modules/control/controllers/lat_based_lqr_plus_controller/lat_plus_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/linear_quadratic_regulator.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;
using Matrix = Eigen::MatrixXd;
using apollo::cyber::Clock;
using apollo::planning::ADCTrajectory;
namespace {

std::string GetLogFileName() {
    time_t raw_time;
    char name_buffer[80];
    std::time(&raw_time);
    std::tm time_tm;
    localtime_r(&raw_time, &time_tm);
    strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv", &time_tm);
    return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
    file_stream << "current_lateral_error," << "current_ref_heading," << "current_heading," << "current_heading_error,"
                << "heading_error_rate," << "lateral_error_rate," << "current_curvature," << "steer_angle,"
                << "steer_angle_feedforward," << "steer_angle_lateral_contribution,"
                << "steer_angle_lateral_rate_contribution," << "steer_angle_heading_contribution,"
                << "steer_angle_heading_rate_contribution," << "steer_angle_feedback," << "steering_position," << "v"
                << std::endl;
}
}  // namespace

LatPlusController::LatPlusController() : name_("LQR-based Lateral Plus Controller") {
    if (FLAGS_enable_csv_debug) {
        steer_log_file_.open(GetLogFileName());
        steer_log_file_ << std::fixed;
        steer_log_file_ << std::setprecision(6);
        WriteHeaders(steer_log_file_);
    }
    AINFO << "Using " << name_;
}

LatPlusController::~LatPlusController() {
    CloseLogFile();
}

bool LatPlusController::InitControlConf(
        const LatBaseLqrPlusControllerConf &current_lat_based_lqr_plus_controller_conf) {
    vehicle_param_ = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

    ts_ = current_lat_based_lqr_plus_controller_conf.ts();
    if (ts_ <= 0.0) {
        AERROR << "[LatPlusController] Invalid control update interval.";
        return false;
    }
    preview_window_ = current_lat_based_lqr_plus_controller_conf.preview_window();
    wheelbase_ = vehicle_param_.wheel_base();
    steer_ratio_ = vehicle_param_.steer_ratio();
    steer_single_direction_max_degree_ = vehicle_param_.max_steer_angle() / M_PI * 180;

    AHC_container_.resize(current_lat_based_lqr_plus_controller_conf.ahc_container_size());
    AHC_compensated_value_ = 0.0;
    AHC_prev_compensated_value_ = 0.0;

    // Matrix init operations.
    const int matrix_size = basic_state_size_ + preview_window_;
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);

    matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);

    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    matrix_b_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bdc_ = Matrix::Zero(matrix_size, 1);

    matrix_state_ = Matrix::Zero(matrix_size, 1);
    matrix_k_ = Matrix::Zero(1, matrix_size);
    matrix_r_ = Matrix::Identity(1, 1);
    matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

    return true;
}

bool LatPlusController::UpdateControlConf(
        const LatBaseLqrPlusControllerConf &current_lat_based_lqr_plus_controller_conf) {
    cf_ = current_lat_based_lqr_plus_controller_conf.cf();
    cr_ = current_lat_based_lqr_plus_controller_conf.cr();
    preview_window_ = current_lat_based_lqr_plus_controller_conf.preview_window();
    lookahead_station_low_speed_ = current_lat_based_lqr_plus_controller_conf.lookahead_station();
    lookback_station_low_speed_ = current_lat_based_lqr_plus_controller_conf.lookback_station();
    lookahead_station_high_speed_ = current_lat_based_lqr_plus_controller_conf.lookahead_station_high_speed();
    lookback_station_high_speed_ = current_lat_based_lqr_plus_controller_conf.lookback_station_high_speed();
    max_lat_acc_ = current_lat_based_lqr_plus_controller_conf.max_lateral_acceleration();
    low_speed_bound_ = current_lat_based_lqr_plus_controller_conf.switch_speed();
    low_speed_window_ = current_lat_based_lqr_plus_controller_conf.switch_speed_window();

    const double mass_fl = current_lat_based_lqr_plus_controller_conf.mass_fl();
    const double mass_fr = current_lat_based_lqr_plus_controller_conf.mass_fr();
    const double mass_rl = current_lat_based_lqr_plus_controller_conf.mass_rl();
    const double mass_rr = current_lat_based_lqr_plus_controller_conf.mass_rr();
    const double mass_front = mass_fl + mass_fr;
    const double mass_rear = mass_rl + mass_rr;
    mass_ = mass_front + mass_rear;

    lf_ = wheelbase_ * (1.0 - mass_front / mass_);
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

    // moment of inertia
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

    lqr_eps_ = current_lat_based_lqr_plus_controller_conf.eps();
    lqr_max_iteration_ = current_lat_based_lqr_plus_controller_conf.max_iteration();

    query_relative_time_ = current_lat_based_lqr_plus_controller_conf.query_relative_time();

    minimum_speed_protection_ = FLAGS_minimum_speed_protection;

    // Matrix init operations.
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

    // matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    const int matrix_size = basic_state_size_ + preview_window_;
    int q_param_size = current_lat_based_lqr_plus_controller_conf.matrix_q_size();
    if (matrix_size != q_param_size) {
        const auto error_msg = absl::StrCat(
                "lateral controller error: matrix_q size: ",
                q_param_size,
                " in parameter file not equal to matrix_size: ",
                matrix_size);
        AERROR << error_msg;
        return false;
    }

    for (int i = 0; i < q_param_size; ++i) {
        matrix_q_(i, i) = current_lat_based_lqr_plus_controller_conf.matrix_q(i);
    }

    matrix_q_updated_ = matrix_q_;
    enable_leadlag_ = current_lat_based_lqr_plus_controller_conf.enable_reverse_leadlag_compensation();
    enable_look_ahead_back_control_ = current_lat_based_lqr_plus_controller_conf.enable_look_ahead_back_control();
    use_new_look_ahead_back_ = current_lat_based_lqr_plus_controller_conf.use_new_look_ahead_back();
    enable_enhance_qparam_ = current_lat_based_lqr_plus_controller_conf.enable_enhance_qparam();
    check_continous_errors_window_ = current_lat_based_lqr_plus_controller_conf.check_continous_errors_window();
    if (!is_lat_turbulence_) {
        com_length_turbulence_ = lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add();
    }

    return true;
}

void LatPlusController::ProcessLogs(const SimpleLateralDebug *debug, const canbus::Chassis *chassis) {
    const std::string log_str = absl::StrCat(
            debug->lateral_error(),
            ",",
            debug->ref_heading(),
            ",",
            debug->heading(),
            ",",
            debug->heading_error(),
            ",",
            debug->heading_error_rate(),
            ",",
            debug->lateral_error_rate(),
            ",",
            debug->curvature(),
            ",",
            debug->steer_angle(),
            ",",
            debug->steer_angle_feedforward(),
            ",",
            debug->steer_angle_lateral_contribution(),
            ",",
            debug->steer_angle_lateral_rate_contribution(),
            ",",
            debug->steer_angle_heading_contribution(),
            ",",
            debug->steer_angle_heading_rate_contribution(),
            ",",
            debug->steer_angle_feedback(),
            ",",
            chassis->steering_percentage(),
            ",",
            injector_->vehicle_state()->linear_velocity());
    if (FLAGS_enable_csv_debug) {
        steer_log_file_ << log_str << std::endl;
    }
    ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatPlusController::LogInitParameters() {
    if (mass_current_ != mass_) {
        AINFO << "Update the vehicle mass.";
        AINFO << "[LatPlusController parameters]" << " mass_: " << mass_ << "," << " iz_: " << iz_ << ","
              << " lf_: " << lf_ << "," << " lr_: " << lr_;
        mass_current_ = mass_;
    }
}

void LatPlusController::InitializeFilters() {
    // Low pass filter
    std::vector<double> den(3, 0.0);
    std::vector<double> num(3, 0.0);
    common::LpfCoefficients(ts_, lat_based_lqr_plus_controller_conf_.cutoff_freq(), &den, &num);
    digital_filter_.set_coefficients(den, num);
    lateral_error_filter_ = common::MeanFilter(
            static_cast<std::uint_fast8_t>(lat_based_lqr_plus_controller_conf_.mean_filter_window_size()));
    heading_error_filter_ = common::MeanFilter(
            static_cast<std::uint_fast8_t>(lat_based_lqr_plus_controller_conf_.mean_filter_window_size()));
}

Status LatPlusController::Init(std::shared_ptr<DependencyInjector> injector) {
    if (!ControlTask::LoadConfig<LatBaseLqrPlusControllerConf>(&lat_based_lqr_plus_controller_conf_)) {
        AERROR << "failed to load control conf in init.";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load lat control_conf init");
    }

    if (!InitControlConf(lat_based_lqr_plus_controller_conf_)) {
        AERROR << "failed to load control conf in init.";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load control_conf init");
    }

    if (!UpdateControlConf(lat_based_lqr_plus_controller_conf_)) {
        AERROR << "failed to update control conf in init.";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to update control_conf init");
    }

    if (!ControlTaskExtend::LoadParamsPipelineConfig<ParamsPipeline>(&params_pipeline_)) {
        AERROR << "failed to load lon pid params pipeline config in init.";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load lon pid params pipeline config");
    }

    if (!LoadParams(&params_pipeline_)) {
        AERROR << "failed to load lat lqr controller params.";
        return Status(ErrorCode::CONTROL_INIT_ERROR, "failed to load lon pid params pipeline config");
    }

    AINFO << name_ << " begin.";
    LogInitParameters();
    InitializeFilters();

    injector_ = injector;
    leadlag_controller_.InitLeadlag();

    enable_mrac_ = lat_based_lqr_plus_controller_conf_.enable_steer_mrac_control();
    if (enable_mrac_) {
        mrac_controller_.Init(
                lat_based_lqr_plus_controller_conf_.steer_mrac_conf(), vehicle_param_.steering_latency_param(), ts_);
    }

    return Status::OK();
}

bool LatPlusController::LoadParams(ParamsPipeline *params_pipeline) {
    AINFO << "Into the lat load params.";
    for (int i = 0; i < params_pipeline->params_declare_size(); i++) {
        LatBaseLqrPlusControllerConf param_config;
        auto param_config_path = params_pipeline->params_declare(i).config_path();
        std::string param_config_path_relative = absl::StrCat("conf", "/", param_config_path);
        AINFO << "Param config_path_relative is " << param_config_path_relative;
        std::string param_config_path_absolute
                = apollo::cyber::plugin_manager::PluginManager::Instance()->GetPluginConfPath<ControlTask>(
                        "apollo::control::LatPlusController", param_config_path_relative);
        AINFO << "Param config_path_absolute is " << param_config_path_absolute;
        if (!apollo::cyber::common::GetProtoFromFile(param_config_path_absolute, &param_config)) {
            AERROR << "Load config: " << params_pipeline->params_declare(i).config_name() << " param failed!";
            return false;
        }
        AINFO << "Load the lat config file successfully, file path: " << param_config_path;
        params_list_.push_back(std::make_pair(params_pipeline->params_declare(i).config_name(), param_config));
    }
    return true;
}

void LatPlusController::CloseLogFile() {
    if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
        steer_log_file_.close();
    }
}

bool LatPlusController::SetCurrentParam(
        const std::string param_config_name,
        LatBaseLqrPlusControllerConf &current_lat_based_lqr_plus_controller_conf) {
    bool set_success = false;
    for (const auto &param : params_list_) {
        if (param.first == param_config_name) {
            current_lat_based_lqr_plus_controller_conf.CopyFrom(param.second);
            set_success = true;
            break;
        }
    }
    if (!set_success) {
        AERROR << "Fail to find lat current param[" << param_config_name << "]";
    }
    return set_success;
}

bool LatPlusController::ChooseCurrentParam(
        LatBaseLqrPlusControllerConf &current_lat_based_lqr_plus_controller_conf,
        const planning::ADCTrajectory *planning_published_trajectory) {
    std::string param_config_name = "public_road_forward_param";

    if (!IsForwardModel()) {
        param_config_name = "public_road_backward_param";
    }

    if (IsLargeCurvature(
                std::fabs(ref_curvature_last_),
                current_lat_based_lqr_plus_controller_conf_.min_large_ref_cur(),
                &is_in_large_curvature_)
        && IsForwardModel()) {
        param_config_name = "public_road_forward_s_turn_param";
    }

    if (planning_published_trajectory->trajectory_type() == ADCTrajectory::OPEN_SPACE) {
        if (IsForwardModel()) {
            param_config_name = "open_space_forward_param";
        } else {
            param_config_name = "open_space_backward_param";
        }
    }

    if (IsEdgeFollow()) {
        if (!IsLargeCurvature(
                    std::fabs(ref_curvature_last_),
                    current_lat_based_lqr_plus_controller_conf_.min_large_ref_cur(),
                    &is_in_large_curvature_)) {
            param_config_name = "edge_follow_public_road_forward_param";
        } else {
            if (IsLeftCurvature(ref_curvature_last_)) {
                param_config_name = "edge_follow_public_road_forward_sl_turn_param";
            } else {
                param_config_name = "edge_follow_public_road_forward_sr_turn_param";
            }
        }
    }

    if (planning_published_trajectory->debug().planning_data().scenario().scenario_plugin_type() == "ZONE_COVER") {
        if (!IsForwardModel()) {
            param_config_name = "zone_cover_backward_param";
        } else {
            param_config_name = "zone_cover_forward_param";
        }
    }

    if (!SetCurrentParam(param_config_name, current_lat_based_lqr_plus_controller_conf)) {
        AERROR << "Failed to set current lat param config: " << param_config_name;
        return false;
    } else {
        if (param_config_name != current_param_config_name_) {
            AINFO << "Update the current lat param config name is " << param_config_name;
            current_param_config_name_ = param_config_name;
        }
        return true;
    }
}

void LatPlusController::Stop() {
    CloseLogFile();
}

std::string LatPlusController::Name() const {
    return name_;
}

Status LatPlusController::ComputeControlCommand(
        const localization::LocalizationEstimate *localization,
        const canbus::Chassis *chassis,
        const planning::ADCTrajectory *planning_published_trajectory,
        ControlCommand *cmd) {
    chassis_ = chassis;

    if (!VehicleStatusIdentificationUpdate(localization, chassis, planning_published_trajectory)) {
        AERROR << "Fail to update the vehicle status identification!";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Fail to update the vehicle status identification!");
    }

    if (!ChooseCurrentParam(current_lat_based_lqr_plus_controller_conf_, planning_published_trajectory)) {
        AERROR << "Fail to load the current lon control param!";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Fail to load the current lon control param!");
    }

    if (!UpdateControlConf(current_lat_based_lqr_plus_controller_conf_)) {
        AERROR << "failed to load control conf";
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "failed to load control_conf");
    }

    LogInitParameters();

    leadlag_controller_.SetLeadlagConf(current_lat_based_lqr_plus_controller_conf_.reverse_leadlag_conf());

    auto vehicle_state = injector_->vehicle_state();
    auto previous_lon_debug = injector_->Get_previous_lon_debug_info();
    auto target_tracking_trajectory = *planning_published_trajectory;

    if (FLAGS_use_navigation_mode
        && current_lat_based_lqr_plus_controller_conf_.enable_navigation_mode_position_update()) {
        auto time_stamp_diff = planning_published_trajectory->header().timestamp_sec() - current_trajectory_timestamp_;

        auto curr_vehicle_x = localization->pose().position().x();
        auto curr_vehicle_y = localization->pose().position().y();

        double curr_vehicle_heading = 0.0;
        const auto &orientation = localization->pose().orientation();
        if (localization->pose().has_heading()) {
            curr_vehicle_heading = localization->pose().heading();
        } else {
            curr_vehicle_heading = common::math::QuaternionToHeading(
                    orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
        }

        // new planning trajectory
        if (time_stamp_diff > 1.0e-6) {
            init_vehicle_x_ = curr_vehicle_x;
            init_vehicle_y_ = curr_vehicle_y;
            init_vehicle_heading_ = curr_vehicle_heading;

            current_trajectory_timestamp_ = planning_published_trajectory->header().timestamp_sec();
        } else {
            auto x_diff_map = curr_vehicle_x - init_vehicle_x_;
            auto y_diff_map = curr_vehicle_y - init_vehicle_y_;
            auto theta_diff = curr_vehicle_heading - init_vehicle_heading_;

            auto cos_map_veh = std::cos(init_vehicle_heading_);
            auto sin_map_veh = std::sin(init_vehicle_heading_);

            auto x_diff_veh = cos_map_veh * x_diff_map + sin_map_veh * y_diff_map;
            auto y_diff_veh = -sin_map_veh * x_diff_map + cos_map_veh * y_diff_map;

            auto cos_theta_diff = std::cos(-theta_diff);
            auto sin_theta_diff = std::sin(-theta_diff);

            auto tx = -(cos_theta_diff * x_diff_veh - sin_theta_diff * y_diff_veh);
            auto ty = -(sin_theta_diff * x_diff_veh + cos_theta_diff * y_diff_veh);

            auto ptr_trajectory_points = target_tracking_trajectory.mutable_trajectory_point();
            std::for_each(
                    ptr_trajectory_points->begin(),
                    ptr_trajectory_points->end(),
                    [&cos_theta_diff, &sin_theta_diff, &tx, &ty, &theta_diff](common::TrajectoryPoint &p) {
                        auto x = p.path_point().x();
                        auto y = p.path_point().y();
                        auto theta = p.path_point().theta();

                        auto x_new = cos_theta_diff * x - sin_theta_diff * y + tx;
                        auto y_new = sin_theta_diff * x + cos_theta_diff * y + ty;
                        auto theta_new = common::math::NormalizeAngle(theta - theta_diff);

                        p.mutable_path_point()->set_x(x_new);
                        p.mutable_path_point()->set_y(y_new);
                        p.mutable_path_point()->set_theta(theta_new);
                    });
        }
    }

    trajectory_analyzer_ = std::move(TrajectoryAnalyzerExtend(&target_tracking_trajectory));

    // Transform the coordinate of the planning trajectory from the center of the
    // rear-axis to the center of mass, if conditions matched
    if (((current_lat_based_lqr_plus_controller_conf_.trajectory_transform_to_com_reverse() && (!IsForwardModel()))
         || (current_lat_based_lqr_plus_controller_conf_.trajectory_transform_to_com_drive() && IsForwardModel()))
        && enable_look_ahead_back_control_) {
        trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
    }

    // Re-build the vehicle dynamic models at reverse driving (in particular,
    // replace the lateral translational motion dynamics with the corresponding
    // kinematic models)
    if (!IsForwardModel()) {
        /*
        A matrix (Gear Reverse)
        [0.0, 0.0, 1.0 * v 0.0;
         0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
         (l_r * c_r - l_f * c_f) / m / v;
         0.0, 0.0, 0.0, 1.0;
         0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
         (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
        */
        cf_ = -current_lat_based_lqr_plus_controller_conf_.cf();
        cr_ = -current_lat_based_lqr_plus_controller_conf_.cr();
        matrix_a_(0, 1) = 0.0;
        matrix_a_coeff_(0, 2) = 1.0;
    } else {
        /*
        A matrix (Gear Drive)
        [0.0, 1.0, 0.0, 0.0;
         0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
         (l_r * c_r - l_f * c_f) / m / v;
         0.0, 0.0, 0.0, 1.0;
         0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
         (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
        */
        cf_ = current_lat_based_lqr_plus_controller_conf_.cf();
        cr_ = current_lat_based_lqr_plus_controller_conf_.cr();
        matrix_a_(0, 1) = 1.0;
        matrix_a_coeff_(0, 2) = 0.0;
    }
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    UpdateDrivingOrientation();

    SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
    debug->Clear();
    auto lat_debug_info = injector_->mutable_control_debug_info()->mutable_simple_lat_debug();
    lat_debug_info->Clear();

    // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
    // Error Rate, preview lateral error1 , preview lateral error2, ...]
    UpdateState(debug, lat_debug_info, chassis);

    UpdateMatrix();

    // Compound discrete matrix with road preview model
    UpdateMatrixCompound();

    // Adjust matrix_q_updated when in reverse gear
    int q_param_size = current_lat_based_lqr_plus_controller_conf_.matrix_q_size();
    for (int i = 0; i < q_param_size; ++i) {
        matrix_q_(i, i) = current_lat_based_lqr_plus_controller_conf_.matrix_q(i);
    }

    if (enable_enhance_qparam_ && (!previous_lon_debug->is_full_stop()) && (!previous_lon_debug->is_full_stop_soft())) {
        EnhanceQParamsAlternate(
                lat_debug_info->lateral_error_match_local(),
                lat_debug_info->heading_error_match_local(),
                planning_published_trajectory,
                previous_lon_debug,
                debug,
                lat_debug_info);
    }

    uint num_iteration;
    double result_diff;
    // Add gain scheduler for higher speed steering
    if (FLAGS_enable_gain_scheduler) {
        double curvature = std::fabs(debug->curvature());
        double speed = std::fabs(vehicle_state->linear_velocity());

        matrix_q_updated_(0, 0) = ed_date_ * matrix_q_(0, 0)
                * InterpolationPlus1D::interpolation_1d(
                                          speed,
                                          current_lat_based_lqr_plus_controller_conf_.lat_speed_input(),
                                          current_lat_based_lqr_plus_controller_conf_.lat_err_speed_gain_output())
                * InterpolationPlus1D::interpolation_1d(
                                          curvature,
                                          current_lat_based_lqr_plus_controller_conf_.lat_curvature_input(),
                                          current_lat_based_lqr_plus_controller_conf_.lat_err_curvature_gain_output());

        matrix_q_updated_(1, 1)
                = matrix_q_(1, 1)
                * InterpolationPlus1D::interpolation_1d(
                          speed,
                          current_lat_based_lqr_plus_controller_conf_.lat_speed_input(),
                          current_lat_based_lqr_plus_controller_conf_.lat_err_delta_speed_gain_output())
                * InterpolationPlus1D::interpolation_1d(
                          curvature,
                          current_lat_based_lqr_plus_controller_conf_.lat_curvature_input(),
                          current_lat_based_lqr_plus_controller_conf_.lat_err_delta_curvature_gain_output());

        matrix_q_updated_(2, 2)
                = efai_rate_ * matrix_q_(2, 2)
                * InterpolationPlus1D::interpolation_1d(
                          speed,
                          current_lat_based_lqr_plus_controller_conf_.lat_speed_input(),
                          current_lat_based_lqr_plus_controller_conf_.heading_err_speed_gain_output())
                * InterpolationPlus1D::interpolation_1d(
                          curvature,
                          current_lat_based_lqr_plus_controller_conf_.lat_curvature_input(),
                          current_lat_based_lqr_plus_controller_conf_.heading_err_curvature_gain_output());

        matrix_q_updated_(3, 3)
                = matrix_q_(3, 3)
                * InterpolationPlus1D::interpolation_1d(
                          speed,
                          current_lat_based_lqr_plus_controller_conf_.lat_speed_input(),
                          current_lat_based_lqr_plus_controller_conf_.heading_err_delta_speed_gain_output())
                * InterpolationPlus1D::interpolation_1d(
                          curvature,
                          current_lat_based_lqr_plus_controller_conf_.lat_curvature_input(),
                          current_lat_based_lqr_plus_controller_conf_.heading_err_delta_curvature_gain_output());
        ADEBUG << "current vehicle is " << speed << ", curvature is " << curvature;
        ADEBUG << "matrix_q_updated_(0, 0) is " << matrix_q_updated_(0, 0);
        ADEBUG << "matrix_q_updated_(1, 1) is " << matrix_q_updated_(1, 1);
        ADEBUG << "matrix_q_updated_(2, 2) is " << matrix_q_updated_(2, 2);
        ADEBUG << "matrix_q_updated_(3, 3) is " << matrix_q_updated_(3, 3);

        common::math::SolveLQRProblem(
                matrix_adc_,
                matrix_bdc_,
                matrix_q_updated_,
                matrix_r_,
                lqr_eps_,
                lqr_max_iteration_,
                &matrix_k_,
                &num_iteration,
                &result_diff);
    } else {
        common::math::SolveLQRProblem(
                matrix_adc_,
                matrix_bdc_,
                matrix_q_,
                matrix_r_,
                lqr_eps_,
                lqr_max_iteration_,
                &matrix_k_,
                &num_iteration,
                &result_diff);
    }

    ADEBUG << "LQR num_iteration is " << num_iteration << ", max iteration threshold is " << lqr_max_iteration_
           << "; result_diff is " << result_diff;

    lat_debug_info->set_lqr_iteration_num(num_iteration);
    lat_debug_info->set_lqr_result_diff(result_diff);

    // feedback = - K * state
    // Convert vehicle steer angle from rad to degree and then to steer degree
    // then to 100% ratio
    const double steer_angle_feedback
            = -(matrix_k_ * matrix_state_)(0, 0) * 180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;

    double curvature = 0.0;
    if (enable_look_ahead_back_control_) {
        if (use_new_look_ahead_back_) {
            if (!IsForwardModel()) {
                curvature = lat_debug_info->lookback_curvature_new();
            } else {
                curvature = lat_debug_info->lookahead_curvature_new();
            }
        } else {
            curvature = lat_debug_info->lookahead_curvature();
        }
    } else {
        curvature = debug->curvature();
    }

    const double steer_angle_feedforward = ComputeFeedForward(curvature);

    double steer_angle = 0.0;
    double steer_angle_feedback_augment = 0.0;
    // Augment the feedback control on lateral error at the desired speed domain
    if (enable_leadlag_) {
        if (current_lat_based_lqr_plus_controller_conf_.enable_feedback_augment_on_high_speed()
            || std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_) {
            steer_angle_feedback_augment = leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI
                    * steer_ratio_ / steer_single_direction_max_degree_ * 100;
            if (std::fabs(vehicle_state->linear_velocity()) > low_speed_bound_ - low_speed_window_) {
                // Within the low-high speed transition window, linerly interplolate the
                // augment control gain for "soft" control switch
                steer_angle_feedback_augment = common::math::lerp(
                        steer_angle_feedback_augment,
                        low_speed_bound_ - low_speed_window_,
                        0.0,
                        low_speed_bound_,
                        std::fabs(vehicle_state->linear_velocity()));
            }
        }
    }
    steer_angle = steer_angle_feedback + steer_angle_feedforward + steer_angle_feedback_augment;

    // Compute the steering command limit with the given maximum lateral
    // acceleration
    const double steer_limit = FLAGS_set_steer_limit
            ? std::atan(
                      max_lat_acc_ * wheelbase_ / (vehicle_state->linear_velocity() * vehicle_state->linear_velocity()))
                    * steer_ratio_ * 180 / M_PI / steer_single_direction_max_degree_ * 100
            : 100.0;

    const double steer_diff_with_max_rate
            = current_lat_based_lqr_plus_controller_conf_.enable_maximum_steer_rate_limit()
            ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI / steer_single_direction_max_degree_ * 100
            : 100.0;

    const double steering_position = chassis->steering_percentage();

    // Re-compute the steering command if the MRAC control is enabled, with steer
    // angle limitation and steer rate limitation
    if (enable_mrac_) {
        const int mrac_model_order = current_lat_based_lqr_plus_controller_conf_.steer_mrac_conf().mrac_model_order();
        Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
        steer_state(0, 0) = chassis->steering_percentage();
        if (mrac_model_order > 1) {
            steer_state(1, 0) = (steering_position - pre_steering_position_) / ts_;
        }
        if (std::fabs(vehicle_state->linear_velocity()) > FLAGS_minimum_speed_resolution) {
            mrac_controller_.SetStateAdaptionRate(1.0);
            mrac_controller_.SetInputAdaptionRate(1.0);
        } else {
            mrac_controller_.SetStateAdaptionRate(0.0);
            mrac_controller_.SetInputAdaptionRate(0.0);
        }
        steer_angle = mrac_controller_.Control(steer_angle, steer_state, steer_limit, steer_diff_with_max_rate / ts_);
        // Set the steer mrac debug message
        MracDebug *mracdebug = debug->mutable_steer_mrac_debug();
        Matrix steer_reference = mrac_controller_.CurrentReferenceState();
        mracdebug->set_mrac_model_order(mrac_model_order);
        for (int i = 0; i < mrac_model_order; ++i) {
            mracdebug->add_mrac_reference_state(steer_reference(i, 0));
            mracdebug->add_mrac_state_error(steer_state(i, 0) - steer_reference(i, 0));
            mracdebug->mutable_mrac_adaptive_gain()->add_state_adaptive_gain(
                    mrac_controller_.CurrentStateAdaptionGain()(i, 0));
        }
        mracdebug->mutable_mrac_adaptive_gain()->add_input_adaptive_gain(
                mrac_controller_.CurrentInputAdaptionGain()(0, 0));
        mracdebug->set_mrac_reference_saturation_status(mrac_controller_.ReferenceSaturationStatus());
        mracdebug->set_mrac_control_saturation_status(mrac_controller_.ControlSaturationStatus());
    }
    pre_steering_position_ = steering_position;
    debug->set_steer_mrac_enable_status(enable_mrac_);

    // Clamp the steer angle with steer limitations at current speed
    double steer_angle_limited = common::math::Clamp(steer_angle, -steer_limit, steer_limit);
    steer_angle = steer_angle_limited;
    debug->set_steer_angle_limited(steer_angle_limited);
    // Limit the steering command with the designed digital filter
    steer_angle = digital_filter_.Filter(steer_angle);
    steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

    // Check if the steer is locked and hence the previous steer angle should be
    // executed
    if (injector_->vehicle_state()->gear() != canbus::Chassis::GEAR_REVERSE) {
        if ((std::abs(vehicle_state->linear_velocity()) < current_lat_based_lqr_plus_controller_conf_.lock_steer_speed()
             || previous_lon_debug->path_remain() <= 0)
            && vehicle_state->gear() == canbus::Chassis::GEAR_DRIVE
            && chassis->driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE) {
            ADEBUG << "Into lock steer, path_remain is " << previous_lon_debug->path_remain() << "linear_velocity is "
                   << vehicle_state->linear_velocity();
            lat_debug_info->set_is_lock_steer(true);
            steer_angle = pre_steer_angle_;
        } else {
            lat_debug_info->set_is_lock_steer(false);
        }
    }

    // Set the steer commands
    cmd->set_steering_target(common::math::Clamp(
            steer_angle, pre_steer_angle_ - steer_diff_with_max_rate, pre_steer_angle_ + steer_diff_with_max_rate));
    cmd->set_steering_rate(FLAGS_steer_angle_rate);

    pre_steer_angle_ = cmd->steering_target();

    // compute extra information for logging and debugging
    const double steer_angle_lateral_contribution = -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_ratio_
            / steer_single_direction_max_degree_ * 100;

    const double steer_angle_lateral_rate_contribution = -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI
            * steer_ratio_ / steer_single_direction_max_degree_ * 100;

    const double steer_angle_heading_contribution = -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_ratio_
            / steer_single_direction_max_degree_ * 100;

    const double steer_angle_heading_rate_contribution = -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI
            * steer_ratio_ / steer_single_direction_max_degree_ * 100;

    debug->set_heading(driving_orientation_);
    debug->set_steer_angle(steer_angle);
    debug->set_steer_angle_feedforward(steer_angle_feedforward);
    debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
    debug->set_steer_angle_lateral_rate_contribution(steer_angle_lateral_rate_contribution);
    debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
    debug->set_steer_angle_heading_rate_contribution(steer_angle_heading_rate_contribution);
    debug->set_steer_angle_feedback(steer_angle_feedback);
    debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
    debug->set_steering_position(steering_position);
    debug->set_ref_speed(vehicle_state->linear_velocity());

    lat_debug_info->set_efai_rate(efai_rate_);
    lat_debug_info->set_ed_rate(ed_date_);
    lat_debug_info->set_enable_look_ahead_back_control(enable_look_ahead_back_control_);
    lat_debug_info->set_is_in_large_curvature(is_in_large_curvature_);

    ProcessLogs(debug, chassis);
    return Status::OK();
}

Status LatPlusController::Reset() {
    matrix_state_.setZero();
    if (enable_mrac_) {
        mrac_controller_.Reset();
    }
    ResetLateralSafetyCheck();
    com_change_add_ = 0.0;
    com_change_turbulence_add_ = 0.0;
    previous_lateral_error_lqr_input_ = 0.0;
    previous_heading_error_lqr_input_ = 0.0;
    ref_curvature_last_ = 0.0;
    is_lat_turbulence_ = false;
    is_lat_preview_in_straight_ = false;
    is_need_cover_turbulence_com_length_ = false;
    is_need_cover_turn_out_lane_com_length_ = false;
    return Status::OK();
}

void LatPlusController::UpdateState(
        SimpleLateralDebug *debug,
        SimpleLateralPlusDebug *debug_info,
        const canbus::Chassis *chassis) {
    auto vehicle_state = injector_->vehicle_state();
    if (FLAGS_use_navigation_mode) {
        ComputeLateralErrors(
                0.0,
                0.0,
                driving_orientation_,
                vehicle_state->linear_velocity(),
                vehicle_state->angular_velocity(),
                vehicle_state->linear_acceleration(),
                trajectory_analyzer_,
                chassis,
                choose_ref_point_,
                debug,
                debug_info);
    } else {
        // Transform the coordinate of the vehicle states from the center of the
        // rear-axis to the center of mass, if conditions matched
        double com_length = 0.0;
        if (is_lat_turbulence_) {
            com_length = com_length_turbulence_;
            is_need_cover_turbulence_com_length_ = true;
        } else if (!is_lat_turbulence_ && is_need_cover_turbulence_com_length_) {
            com_length = com_length_turbulence_ + com_change_turbulence_add_;
            ResetComLengthFromTurbulence();
            if (com_length >= lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add()) {
                is_need_cover_turbulence_com_length_ = false;
                com_change_turbulence_add_ = 0.0;
            }
        } else if (current_lat_based_lqr_plus_controller_conf_.is_need_turn_out_lane()) {
            com_length = current_lat_based_lqr_plus_controller_conf_.turn_out_lane_com_length();
            is_need_cover_turn_out_lane_com_length_ = true;
        } else if (
                !current_lat_based_lqr_plus_controller_conf_.is_need_turn_out_lane()
                && is_need_cover_turn_out_lane_com_length_) {
            com_length = current_lat_based_lqr_plus_controller_conf_.turn_out_lane_com_length()
                    + com_change_turbulence_add_;
            ResetComLengthFromTurbulence();
            if (com_length >= lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add()) {
                is_need_cover_turn_out_lane_com_length_ = false;
                com_change_turbulence_add_ = 0.0;
            }
        } else {
            com_length = lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add();
            if (current_lat_based_lqr_plus_controller_conf_.use_continous_errors_check()) {
                com_length = lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add() + com_change_add_;
            } else {
                com_length = lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add() + com_change_add_;
                ResetComLength();
            }
        }

        com_length = common::math::Clamp(com_length, -2 * lr_, 2 * lr_);
        debug_info->set_com_length(com_length);
        const auto &com = vehicle_state->ComputeCOMPosition(com_length);
        debug_info->mutable_lat_com_position()->set_x(com.x());
        debug_info->mutable_lat_com_position()->set_y(com.y());
        ComputeLateralErrors(
                com.x(),
                com.y(),
                driving_orientation_,
                vehicle_state->linear_velocity(),
                vehicle_state->angular_velocity(),
                vehicle_state->linear_acceleration(),
                trajectory_analyzer_,
                chassis,
                choose_ref_point_,
                debug,
                debug_info);
    }
    // State matrix update;
    // First four elements are fixed;
    if (enable_look_ahead_back_control_) {
        if (use_new_look_ahead_back_) {
            matrix_state_(0, 0) = debug_info->lateral_error_feedback_new();
            matrix_state_(2, 0) = debug_info->heading_error_feedback_new();
        } else {
            matrix_state_(0, 0) = debug->lateral_error_feedback();
            matrix_state_(2, 0) = debug->heading_error_feedback();
        }
    } else {
        matrix_state_(0, 0) = debug_info->lateral_error_lqr_input();
        matrix_state_(2, 0) = debug_info->heading_error_lqr_input();
    }
    matrix_state_(1, 0) = debug->lateral_error_rate();
    matrix_state_(3, 0) = debug->heading_error_rate();

    // Next elements are depending on preview window size;
    for (int i = 0; i < preview_window_; ++i) {
        const double preview_time = ts_ * (i + 1);
        const auto preview_point = trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);

        const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
                preview_point.path_point().x(), preview_point.path_point().y());

        const double dx = preview_point.path_point().x() - matched_point.path_point().x();
        const double dy = preview_point.path_point().y() - matched_point.path_point().y();

        const double cos_matched_theta = std::cos(matched_point.path_point().theta());
        const double sin_matched_theta = std::sin(matched_point.path_point().theta());
        const double preview_d_error = cos_matched_theta * dy - sin_matched_theta * dx;

        matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
    }

    const auto &track_com
            = vehicle_state->ComputeCOMPosition(current_lat_based_lqr_plus_controller_conf_.track_com_length());
    debug_info->mutable_lat_track_com_position()->set_x(track_com.x());
    debug_info->mutable_lat_track_com_position()->set_y(track_com.y());

    if (is_lat_turbulence_) {
        ComputeTrackLateralErrors(
                track_com.x(),
                track_com.y(),
                driving_orientation_,
                vehicle_state->linear_velocity(),
                trajectory_analyzer_,
                control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_LOCAL,
                debug_info);
    } else {
        ComputeTrackLateralErrors(
                track_com.x(),
                track_com.y(),
                driving_orientation_,
                vehicle_state->linear_velocity(),
                trajectory_analyzer_,
                current_lat_based_lqr_plus_controller_conf_.choose_track_point(),
                debug_info);
    }

    if (current_lat_based_lqr_plus_controller_conf_.use_continous_errors_check()
        && CheckContinousErrors(
                check_continous_errors_window_,
                debug_info->lateral_error_track_com(),
                current_lat_based_lqr_plus_controller_conf_.lat_continous_err_threshold(),
                &lateral_lr_errors_,
                &current_sign_)) {
        if (current_sign_ > 0) {
            ADEBUG << "Current vehicle is left of trajectory";
            // idenfy turn left
            if (chassis->steering_percentage()
                >= current_lat_based_lqr_plus_controller_conf_.straight_steering_percentage()) {
                // reduce the com if turn left
                com_change_ = -vehicle_state->linear_velocity() * ts_
                        * current_lat_based_lqr_plus_controller_conf_.com_change_rate();
                ADEBUG << "reduce the com: " << com_change_ << " in turn left";
            } else if (
                    chassis->steering_percentage()
                    <= -current_lat_based_lqr_plus_controller_conf_.straight_steering_percentage()) {
                // idenfy turn right
                // increase the com if turn right
                com_change_ = vehicle_state->linear_velocity() * ts_
                        * current_lat_based_lqr_plus_controller_conf_.com_change_rate();
                ADEBUG << "increase the com: " << com_change_ << " in turn right";
            } else {
                ResetComLength();
            }
            ADEBUG << "Current com_length is " << debug_info->com_length();
            choose_ref_point_ = control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_COM;
        } else if (current_sign_ < 0) {
            ADEBUG << "Current vehicle is right of trajectory";
            // idenfy turn left
            if (chassis->steering_percentage()
                >= current_lat_based_lqr_plus_controller_conf_.straight_steering_percentage()) {
                // increase the com if turn left
                com_change_ = vehicle_state->linear_velocity() * ts_
                        * current_lat_based_lqr_plus_controller_conf_.com_change_rate();
                ADEBUG << "increase the com: " << com_change_ << " in turn left";
            } else if (
                    chassis->steering_percentage()
                    <= -current_lat_based_lqr_plus_controller_conf_.straight_steering_percentage()) {
                // idenfy turn right
                // reduce the com if turn right
                com_change_ = -vehicle_state->linear_velocity() * ts_
                        * current_lat_based_lqr_plus_controller_conf_.com_change_rate();
                ADEBUG << "reduce the com: " << com_change_ << " in turn right";
            } else {
                ResetComLength();
            }
            ADEBUG << "Current com_length is " << debug_info->com_length();
            choose_ref_point_ = control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_COM;
        } else {
            com_change_ = 0;
        }
        com_change_add_ = com_change_add_ + com_change_;
        com_change_add_ = common::math::Clamp(
                com_change_add_,
                -(lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add()),
                (lr_ - current_lat_based_lqr_plus_controller_conf_.lat_err_position_add()));
        ADEBUG << "com_change_add_ is " << com_change_add_;
        ADEBUG << "com_change_ is " << com_change_;
        ADEBUG << "current_sign_ is " << current_sign_;
    } else {
        choose_ref_point_ = current_lat_based_lqr_plus_controller_conf_.choose_ref_point();
    }

    // check cuvature, lat_error, heading_error changing turbulence
    if (current_lat_based_lqr_plus_controller_conf_.use_turbulence_check()
        && (std::fabs(debug_info->lateral_error_lqr_input_change())
                    > current_lat_based_lqr_plus_controller_conf_.lateral_error_turbulence_threshold()
            && std::fabs(debug_info->heading_error_lqr_input_change())
                    > current_lat_based_lqr_plus_controller_conf_.heading_error_turbulence_threshold())) {
        AINFO << "Into turbulence condition";
        is_lat_turbulence_ = true;
        choose_ref_point_ = control::ChooseRefTrajectoryPoint::USE_LAT_ABSOLUTE_LOCAL;
        com_length_turbulence_ = 0;
    } else {
        ADEBUG << "Quit turbulence condition";
        is_lat_turbulence_ = false;
    }

    // check quit turbulence and into long straight condition

    for (size_t i = 0; i < lateral_lr_errors_.size(); i++) {
        debug_info->add_continous_lateral_errors(lateral_lr_errors_[i]);
    }
    debug_info->set_current_sign(current_sign_);
    debug_info->set_com_change(com_change_);
    debug_info->set_current_choose_point(choose_ref_point_);
}

void LatPlusController::ResetComLength() {
    auto vehicle_state = injector_->vehicle_state();
    if (com_change_add_ < -1e-3) {
        com_change_reset_ = vehicle_state->linear_velocity() * ts_
                * current_lat_based_lqr_plus_controller_conf_.com_change_rate();
    } else if (com_change_add_ > 1e-3) {
        com_change_reset_ = -std::fabs(vehicle_state->linear_velocity()) * ts_
                * current_lat_based_lqr_plus_controller_conf_.com_change_rate();
    } else {
        com_change_reset_ = 0;
    }
    com_change_add_ = com_change_add_ + com_change_reset_;
    ADEBUG << "com_change_add_ is " << com_change_add_;
    ADEBUG << "com_change_reset_ is " << com_change_reset_;
}

void LatPlusController::ResetComLengthFromTurbulence() {
    ADEBUG << "Into ResetComLengthFromTurbulence";
    auto vehicle_state = injector_->vehicle_state();
    com_turbulence_change_ = vehicle_state->linear_velocity() * ts_
            * current_lat_based_lqr_plus_controller_conf_.com_quit_turbulence_change_rate();
    com_change_turbulence_add_ = com_change_turbulence_add_ + com_turbulence_change_;
    com_change_turbulence_add_ = common::math::Clamp(
            com_change_turbulence_add_, 0.0, lr_ + current_lat_based_lqr_plus_controller_conf_.lat_err_position_add());
}

bool LatPlusController::CheckContinousErrors(
        const uint8_t window_size,
        const double error,
        const double threshold,
        std::vector<double> *arr,
        int32_t *current_sign) {
    arr->push_back(error);

    if (arr->size() < window_size) {
        AERROR << "Not enough elements in the array.";
        return false;
    }

    if (arr->size() > window_size) {
        arr->erase(arr->begin(), arr->begin() + (arr->size() - window_size));
    }

    int8_t currentIndex = arr->size() - 1;
    int32_t current_direction = 0;
    for (int i = currentIndex; i > currentIndex - window_size; --i) {
        current_direction = ((*arr)[i] >= threshold) ? 1 : ((*arr)[i] <= -threshold) ? -1 : 0;
        ADEBUG << "current_direction is " << current_direction << " with: " << (*arr)[i] << " " << i;
        double current_error_abs = std::fabs((*arr)[i]);
        // &&(prev_abs_ >= current_error_abs && current_lat_based_lqr_plus_controller_conf.use_monotonicity_check())
        if (prev_sign_ != 0 && current_direction != 0 && prev_sign_ != current_direction) {
            ADEBUG << "Elements at positions " << i << " and " << i - 1 << " do not satisfy the condition.";
            current_direction = 0;
            *current_sign = current_direction;
            ADEBUG << "current_sign is " << *current_sign;
            return false;
        }
        prev_sign_ = current_direction;
        prev_abs_ = current_error_abs;
    }
    *current_sign = current_direction;
    ADEBUG << "current_sign is " << *current_sign;
    return true;
}

void LatPlusController::UpdateMatrix() {
    double v = 0.01;
    // At reverse driving, replace the lateral translational motion dynamics with
    // the corresponding kinematic models
    if (!IsForwardModel() && !current_lat_based_lqr_plus_controller_conf_.reverse_use_dynamic_model()) {
        v = std::min(injector_->vehicle_state()->linear_velocity(), -minimum_speed_protection_);
        matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
    } else {
        v = std::max(injector_->vehicle_state()->linear_velocity(), minimum_speed_protection_);
        matrix_a_(0, 2) = 0.0;
    }
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
    Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
    matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() * (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LatPlusController::UpdateMatrixCompound() {
    // Initialize preview matrix
    matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
    matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
    if (preview_window_ > 0) {
        matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
        // Update A matrix;
        for (int i = 0; i < preview_window_ - 1; ++i) {
            matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
        }
    }
}

double LatPlusController::ComputeFeedForward(double ref_curvature) {
    const double kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

    // Calculate the feedforward term of the lateral controller; then change it
    // from rad to %
    const double v = injector_->vehicle_state()->linear_velocity();
    double steer_angle_feedforwardterm = 0.0;
    if (!IsForwardModel() && !current_lat_based_lqr_plus_controller_conf_.reverse_use_dynamic_model()) {
        steer_angle_feedforwardterm = current_lat_based_lqr_plus_controller_conf_.reverse_feedforward_ratio()
                * wheelbase_ * ref_curvature * 180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
    } else {
        steer_angle_feedforwardterm
                = (wheelbase_ * ref_curvature + kv * v * v * ref_curvature
                   - matrix_k_(0, 2)
                           * (lr_ * ref_curvature - lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_))
                * 180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
    }

    return steer_angle_feedforwardterm;
}

void LatPlusController::ComputeTrackLateralErrors(
        const double x,
        const double y,
        const double theta,
        const double linear_v,
        const TrajectoryAnalyzerExtend &trajectory_analyzer,
        const enum ChooseRefTrajectoryPoint &choose_point,
        SimpleLateralPlusDebug *lat_debug) {
    double lateral_error = 0.0;
    double heading_error = 0.0;
    double curvature = 0.0;
    PathPoint matched_track_point_com;
    switch (choose_point) {
    case control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_LR: {
        matched_track_point_com.set_x(lat_debug->target_point_lr().path_point().x());
        matched_track_point_com.set_y(lat_debug->target_point_lr().path_point().y());
        lateral_error = lat_debug->lateral_error_target_lr();
        heading_error = lat_debug->heading_error_target_lr();
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_LOCAL: {
        matched_track_point_com.set_x(lat_debug->matched_point_local().path_point().x());
        matched_track_point_com.set_y(lat_debug->matched_point_local().path_point().y());
        lateral_error = lat_debug->lateral_error_match_local();
        heading_error = lat_debug->heading_error_match_local();
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_ABSOLUTE_LOCAL: {
        matched_track_point_com.set_x(lat_debug->absolute_point_local().path_point().x());
        matched_track_point_com.set_y(lat_debug->absolute_point_local().path_point().y());
        lateral_error = lat_debug->lateral_error_absolute_local();
        heading_error = lat_debug->heading_error_absolute_local();
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_TRACK_COM: {
        // match_track_com
        matched_track_point_com = trajectory_analyzer.QueryMatchedPathPoint(x, y);
        double s_track_com, s_dot_track_com, d_track_com, d_dot_track_com;
        trajectory_analyzer.ToTrajectoryFrame(
                x,
                y,
                theta,
                linear_v,
                matched_track_point_com,
                &s_track_com,
                &s_dot_track_com,
                &d_track_com,
                &d_dot_track_com);

        lateral_error = d_track_com;
        heading_error = common::math::NormalizeAngle(theta - matched_track_point_com.theta());
        break;
    }
    default: {
        // match_track_com
        matched_track_point_com = trajectory_analyzer.QueryMatchedPathPoint(x, y);
        double s_track_com, s_dot_track_com, d_track_com, d_dot_track_com;
        trajectory_analyzer.ToTrajectoryFrame(
                x,
                y,
                theta,
                linear_v,
                matched_track_point_com,
                &s_track_com,
                &s_dot_track_com,
                &d_track_com,
                &d_dot_track_com);

        lateral_error = d_track_com;
        heading_error = common::math::NormalizeAngle(theta - matched_track_point_com.theta());
        break;
    }
    }
    lat_debug->mutable_track_point_com()->mutable_path_point()->set_x(matched_track_point_com.x());
    lat_debug->mutable_track_point_com()->mutable_path_point()->set_y(matched_track_point_com.y());
    lat_debug->set_lateral_error_track_com(lateral_error);
    lat_debug->set_heading_error_track_com(heading_error);
}

void LatPlusController::ComputeLateralErrors(
        const double x,
        const double y,
        const double theta,
        const double linear_v,
        const double angular_v,
        const double linear_a,
        const TrajectoryAnalyzerExtend &trajectory_analyzer,
        const canbus::Chassis *chassis,
        const enum ChooseRefTrajectoryPoint &choose_point,
        SimpleLateralDebug *debug,
        SimpleLateralPlusDebug *lat_debug) {
    TrajectoryPoint target_point;

    if (current_lat_based_lqr_plus_controller_conf_.query_time_nearest_point_only()) {
        target_point
                = trajectory_analyzer.QueryNearestPointByAbsoluteTime(Clock::NowInSeconds() + query_relative_time_);
    } else {
        if (FLAGS_use_navigation_mode
            && !current_lat_based_lqr_plus_controller_conf_.enable_navigation_mode_position_update()) {
            target_point
                    = trajectory_analyzer.QueryNearestPointByAbsoluteTime(Clock::NowInSeconds() + query_relative_time_);
        } else {
            target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
        }
    }
    ADEBUG << "x point: " << x << " y point: " << y;
    ADEBUG << "match trajectory point information : " << target_point.ShortDebugString();

    auto vehicle_state = injector_->vehicle_state();

    // target_lr
    const auto &com_lr = vehicle_state->ComputeCOMPosition(lr_);
    lat_debug->set_lr(lr_);
    lat_debug->mutable_lat_lr_position()->set_x(com_lr.x());
    lat_debug->mutable_lat_lr_position()->set_y(com_lr.y());
    auto target_point_lr = trajectory_analyzer.QueryNearestPointByPosition(com_lr.x(), com_lr.y());
    lat_debug->mutable_target_point_lr()->mutable_path_point()->set_x(target_point_lr.path_point().x());
    lat_debug->mutable_target_point_lr()->mutable_path_point()->set_y(target_point_lr.path_point().y());
    double s_target_lr, s_dot_target_lr, d_target_lr, d_dot_target_lr;
    trajectory_analyzer.ToTrajectoryFrame(
            com_lr.x(),
            com_lr.y(),
            vehicle_state->heading(),
            linear_v,
            target_point.path_point(),
            &s_target_lr,
            &s_dot_target_lr,
            &d_target_lr,
            &d_dot_target_lr);
    debug->set_ref_heading(target_point_lr.path_point().theta());
    lat_debug->set_curvature_target_lr(target_point_lr.path_point().kappa());
    lat_debug->set_lateral_error_target_lr(d_target_lr);
    lat_debug->set_heading_error_target_lr(common::math::NormalizeAngle(theta - target_point_lr.path_point().theta()));

    // target_com
    target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
    debug->mutable_current_target_point()->mutable_path_point()->set_x(target_point.path_point().x());
    debug->mutable_current_target_point()->mutable_path_point()->set_y(target_point.path_point().y());
    double s_target_com, s_dot_target_com, d_target_com, d_dot_target_com;
    trajectory_analyzer.ToTrajectoryFrame(
            x,
            y,
            vehicle_state->heading(),
            linear_v,
            target_point.path_point(),
            &s_target_com,
            &s_dot_target_com,
            &d_target_com,
            &d_dot_target_com);
    debug->set_ref_heading(target_point.path_point().theta());
    lat_debug->set_curvature_target_com(target_point.path_point().kappa());
    lat_debug->set_lateral_error_target_com(d_target_com);
    lat_debug->set_heading_error_target_com(common::math::NormalizeAngle(theta - target_point.path_point().theta()));

    // match_com
    auto matched_point_com = trajectory_analyzer.QueryMatchedPathPoint(x, y);
    lat_debug->mutable_matched_point_com()->mutable_path_point()->set_x(matched_point_com.x());
    lat_debug->mutable_matched_point_com()->mutable_path_point()->set_y(matched_point_com.y());
    double s_match_com, s_dot_match_com, d_match_com, d_dot_match_com;
    trajectory_analyzer.ToTrajectoryFrame(
            x, y, theta, linear_v, matched_point_com, &s_match_com, &s_dot_match_com, &d_match_com, &d_dot_match_com);
    debug->set_ref_heading(matched_point_com.theta());
    lat_debug->set_curvature_match_com(matched_point_com.kappa());
    lat_debug->set_lateral_error_match_com(d_match_com);
    lat_debug->set_heading_error_match_com(common::math::NormalizeAngle(theta - matched_point_com.theta()));

    // match_local
    auto matched_point_local = trajectory_analyzer.QueryMatchedPathPoint(vehicle_state->x(), vehicle_state->y());
    lat_debug->mutable_matched_point_local()->mutable_path_point()->set_x(matched_point_local.x());
    lat_debug->mutable_matched_point_local()->mutable_path_point()->set_y(matched_point_local.y());
    double s_match_local, s_dot_match_local, d_match_local, d_dot_match_local;
    trajectory_analyzer.ToTrajectoryFrame(
            vehicle_state->x(),
            vehicle_state->y(),
            theta,
            linear_v,
            matched_point_local,
            &s_match_local,
            &s_dot_match_local,
            &d_match_local,
            &d_dot_match_local);
    debug->set_ref_heading(matched_point_local.theta());
    lat_debug->set_curvature_match_local(matched_point_local.kappa());
    lat_debug->set_lateral_error_match_local(d_match_local);
    lat_debug->set_heading_error_match_local(common::math::NormalizeAngle(theta - matched_point_local.theta()));

    // absolute_local
    double current_absolute_time = FLAGS_sim_by_record
            ? (chassis_->header().timestamp_sec() + query_relative_time_)
            : (::apollo::cyber::Clock::NowInSeconds() + query_relative_time_);
    auto absolute_point_local = trajectory_analyzer.QueryNearestPointByAbsoluteTime(current_absolute_time);
    lat_debug->mutable_absolute_point_local()->mutable_path_point()->set_x(absolute_point_local.path_point().x());
    lat_debug->mutable_absolute_point_local()->mutable_path_point()->set_y(absolute_point_local.path_point().y());
    double s_absolute_local, s_dot_absolute_local, d_absolute_local, d_dot_absolute_local;
    trajectory_analyzer.ToTrajectoryFrame(
            vehicle_state->x(),
            vehicle_state->y(),
            theta,
            linear_v,
            absolute_point_local.path_point(),
            &s_absolute_local,
            &s_dot_absolute_local,
            &d_absolute_local,
            &d_dot_absolute_local);
    debug->set_ref_heading(absolute_point_local.path_point().theta());
    lat_debug->set_curvature_absolute_local(absolute_point_local.path_point().kappa());
    lat_debug->set_lateral_error_absolute_local(d_absolute_local);
    lat_debug->set_heading_error_absolute_local(
            common::math::NormalizeAngle(theta - absolute_point_local.path_point().theta()));

    double lateral_error = 0.0;
    double heading_error = 0.0;
    double curvature = 0.0;

    switch (choose_point) {
    case control::ChooseRefTrajectoryPoint::USE_LAT_TARGET_COM: {
        lateral_error = lat_debug->lateral_error_target_com();
        heading_error = lat_debug->heading_error_target_com();
        curvature = lat_debug->curvature_target_com();
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_COM: {
        lateral_error = lat_debug->lateral_error_match_com();
        heading_error = lat_debug->heading_error_match_com();
        curvature = lat_debug->curvature_match_com();
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_LOCAL: {
        lateral_error = lat_debug->lateral_error_match_local();
        heading_error = lat_debug->heading_error_match_local();
        curvature = lat_debug->curvature_match_local();
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_ABSOLUTE_LOCAL: {
        lateral_error = lat_debug->lateral_error_absolute_local();
        heading_error = lat_debug->heading_error_absolute_local();
        curvature = lat_debug->curvature_absolute_local();
        break;
    }
    default: {
        const double dx = com_lr.x() - target_point.path_point().x();
        const double dy = com_lr.y() - target_point.path_point().y();
        const double cos_target_heading = std::cos(target_point.path_point().theta());
        const double sin_target_heading = std::sin(target_point.path_point().theta());

        double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
        if (current_lat_based_lqr_plus_controller_conf_.enable_navigation_mode_error_filter()) {
            lateral_error = lateral_error_filter_.Update(lateral_error);
        }
        debug->set_ref_heading(target_point.path_point().theta());
        heading_error = common::math::NormalizeAngle(theta - debug->ref_heading());
        curvature = target_point.path_point().kappa();
        break;
    }
    }

    // Set curvature
    debug->set_curvature(curvature);
    lat_debug->set_curvature_change(debug->curvature() - ref_curvature_last_);
    ref_curvature_last_ = debug->curvature();

    // Set lateral error and heading error origin
    if (std::fabs(lat_debug->lateral_error_match_com()) <= std::fabs(lat_debug->lateral_error_match_local())) {
        debug->set_lateral_error(lat_debug->lateral_error_match_com());
        lat_debug->set_current_min_error_point(control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_COM);
    } else {
        debug->set_lateral_error(lat_debug->lateral_error_match_local());
        lat_debug->set_current_min_error_point(control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_LOCAL);
    }
    if (std::fabs(lat_debug->heading_error_match_com()) <= std::fabs(lat_debug->heading_error_match_local())) {
        debug->set_heading_error(lat_debug->heading_error_match_com());
    } else {
        debug->set_heading_error(lat_debug->heading_error_match_local());
    }

    // Set the lateral error lqr input
    lat_debug->set_lateral_error_lqr_input(lateral_error);
    lat_debug->set_lateral_error_lqr_input_change(
            lat_debug->lateral_error_lqr_input() - previous_lateral_error_lqr_input_);
    previous_lateral_error_lqr_input_ = lat_debug->lateral_error_lqr_input();

    // set heading error
    lat_debug->set_heading_error_by_ref(heading_error);
    if (current_lat_based_lqr_plus_controller_conf_.enable_navigation_mode_error_filter()) {
        heading_error = heading_error_filter_.Update(heading_error);
    }
    // the car may have inherent steer error, the method below can handle this
    // error
    if (current_lat_based_lqr_plus_controller_conf_.use_heading_auto_compensation()) {
        double compensation_output_value = CalculateTempAHCValue(linear_v);
        heading_error -= compensation_output_value;
        lat_debug->set_heading_error_by_auto_compensation(heading_error);
        injector_->mutable_control_debug_info()
                ->mutable_simple_lat_debug()
                ->mutable_ahc_value()
                ->set_compensation_output_value(compensation_output_value);
        if (IsDrivingStraight(chassis, target_point_lr.path_point().kappa())) {
            UpdateAHCParam(heading_error, injector_->vehicle_state()->linear_velocity());
        } else {
            AHCClear();
        }
    }
    // Set the heading error lqr input
    lat_debug->set_heading_error_lqr_input(heading_error);
    lat_debug->set_heading_error_lqr_input_change(
            lat_debug->heading_error_lqr_input() - previous_heading_error_lqr_input_);
    previous_heading_error_lqr_input_ = lat_debug->heading_error_lqr_input();

    // Within the low-high speed transition window, linerly interplolate the
    // lookahead/lookback station for "soft" prediction window switch
    double lookahead_station = 0.0;
    double lookback_station = 0.0;
    if (std::fabs(linear_v) >= low_speed_bound_) {
        lookahead_station = lookahead_station_high_speed_;
        lookback_station = lookback_station_high_speed_;
    } else if (std::fabs(linear_v) < low_speed_bound_ - low_speed_window_) {
        lookahead_station = lookahead_station_low_speed_;
        lookback_station = lookback_station_low_speed_;
    } else {
        lookahead_station = common::math::lerp(
                lookahead_station_low_speed_,
                low_speed_bound_ - low_speed_window_,
                lookahead_station_high_speed_,
                low_speed_bound_,
                std::fabs(linear_v));
        lookback_station = common::math::lerp(
                lookback_station_low_speed_,
                low_speed_bound_ - low_speed_window_,
                lookback_station_high_speed_,
                low_speed_bound_,
                std::fabs(linear_v));
    }
    lat_debug->set_lookahead_station(lookahead_station);
    lat_debug->set_lookback_station(lookback_station);

    // update lookforward point error
    switch (choose_point) {
    case control::ChooseRefTrajectoryPoint::USE_LAT_TARGET_COM: {
        UpdateLookForwardPointError(
                lookahead_station,
                lookback_station,
                x,
                y,
                theta,
                linear_v,
                target_point,
                trajectory_analyzer,
                debug,
                lat_debug);
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_COM: {
        UpdateLookForwardPointError(
                lookahead_station,
                lookback_station,
                x,
                y,
                theta,
                linear_v,
                target_point,
                trajectory_analyzer,
                debug,
                lat_debug);
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_MATCH_LOCAL: {
        UpdateLookForwardPointError(
                lookahead_station,
                lookback_station,
                vehicle_state->x(),
                vehicle_state->y(),
                theta,
                linear_v,
                absolute_point_local,
                trajectory_analyzer,
                debug,
                lat_debug);
        break;
    }
    case control::ChooseRefTrajectoryPoint::USE_LAT_ABSOLUTE_LOCAL: {
        UpdateLookForwardPointError(
                lookahead_station,
                lookback_station,
                vehicle_state->x(),
                vehicle_state->y(),
                theta,
                linear_v,
                absolute_point_local,
                trajectory_analyzer,
                debug,
                lat_debug);
        break;
    }
    default: {
        UpdateLookForwardPointError(
                lookahead_station,
                lookback_station,
                com_lr.x(),
                com_lr.y(),
                theta,
                linear_v,
                target_point_lr,
                trajectory_analyzer,
                debug,
                lat_debug);
        break;
    }
    }

    LateralSafetyCheck(debug, lat_debug);

    auto lateral_error_dot = linear_v * std::sin(heading_error);
    auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
    if (FLAGS_reverse_heading_control) {
        if (!IsForwardModel()) {
            lateral_error_dot = -lateral_error_dot;
            lateral_error_dot_dot = -lateral_error_dot_dot;
        }
    }
    auto centripetal_acceleration = linear_v * linear_v / wheelbase_
            * std::tan(chassis->steering_percentage() / 100 * vehicle_param_.max_steer_angle() / steer_ratio_);
    debug->set_lateral_error_rate(lateral_error_dot);
    debug->set_lateral_acceleration(lateral_error_dot_dot);
    debug->set_lateral_centripetal_acceleration(centripetal_acceleration);
    debug->set_lateral_jerk((debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
    previous_lateral_acceleration_ = debug->lateral_acceleration();

    if (!IsForwardModel()) {
        debug->set_heading_rate(-angular_v);
    } else {
        debug->set_heading_rate(angular_v);
    }
    debug->set_ref_heading_rate(target_point.path_point().kappa() * target_point.v());
    debug->set_heading_error_rate(debug->heading_rate() - debug->ref_heading_rate());

    debug->set_heading_acceleration((debug->heading_rate() - previous_heading_rate_) / ts_);
    debug->set_ref_heading_acceleration((debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
    debug->set_heading_error_acceleration(debug->heading_acceleration() - debug->ref_heading_acceleration());
    previous_heading_rate_ = debug->heading_rate();
    previous_ref_heading_rate_ = debug->ref_heading_rate();

    debug->set_heading_jerk((debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
    debug->set_ref_heading_jerk((debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) / ts_);
    debug->set_heading_error_jerk(debug->heading_jerk() - debug->ref_heading_jerk());
    previous_heading_acceleration_ = debug->heading_acceleration();
    previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();
}

void LatPlusController::UpdateLookForwardPointError(
        const double lookahead_station,
        const double lookback_station,
        const double pose_x,
        const double pose_y,
        const double theta,
        const double linear_v,
        const common::TrajectoryPoint &ref_point,
        const TrajectoryAnalyzerExtend &trajectory_analyzer,
        SimpleLateralDebug *debug,
        SimpleLateralPlusDebug *debug_info) {
    auto lookahead_point_new = trajectory_analyzer.QueryNearestPointByRelativeStation(ref_point, lookahead_station);
    debug_info->mutable_lookahead_point_new()->CopyFrom(lookahead_point_new);

    auto lookback_point_new = trajectory_analyzer.QueryNearestPointByRelativeStation(ref_point, -lookback_station);
    debug_info->mutable_lookback_point_new()->CopyFrom(lookback_point_new);

    auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
            ref_point.relative_time()
            + lookahead_station / (std::max(std::fabs(linear_v), 0.1) * std::cos(debug->heading())));

    debug_info->set_lookahead_time(ref_point.relative_time());
    debug_info->set_lookahead_station_s_diff(lookahead_station);

    // Estimate the heading error with look-ahead/look-back windows as feedback
    // signal for special driving scenarios
    // Calculate heading error feedback
    double heading_error_feedback = 0.0;
    double heading_error_feedback_new = 0.0;
    double lookahead_time_new = 0.0;
    double lookahead_station_s_diff_new = 0.0;
    double lookback_time_new = 0.0;
    double lookback_station_s_diff_new = 0.0;

    if (!IsForwardModel()) {
        heading_error_feedback = debug_info->heading_error_lqr_input();
        heading_error_feedback_new = common::math::NormalizeAngle(theta - lookback_point_new.path_point().theta());
        lookback_time_new = lookahead_point_new.relative_time() - ref_point.relative_time();
        lookback_station_s_diff_new = lookahead_point_new.path_point().s() - ref_point.path_point().s();
    } else {
        heading_error_feedback = common::math::NormalizeAngle(
                debug_info->heading_error_lqr_input() + ref_point.path_point().theta()
                - lookahead_point.path_point().theta());
        heading_error_feedback_new = common::math::NormalizeAngle(theta - lookahead_point_new.path_point().theta());
        lookahead_time_new = lookahead_point_new.relative_time() - ref_point.relative_time();
        lookahead_station_s_diff_new = lookahead_point_new.path_point().s() - ref_point.path_point().s();
    }
    debug_info->set_lookahead_time_new(lookahead_time_new);
    debug_info->set_lookahead_station_s_diff_new(lookahead_station_s_diff_new);
    debug_info->set_lookback_time_new(lookback_time_new);
    debug_info->set_lookback_station_s_diff_new(lookback_station_s_diff_new);

    debug_info->set_lookahead_curvature(lookahead_point.path_point().kappa());
    debug_info->set_lookahead_curvature_new(lookahead_point_new.path_point().kappa());
    debug_info->set_lookback_curvature_new(lookback_point_new.path_point().kappa());

    debug->set_heading_error_feedback(heading_error_feedback);
    debug_info->set_heading_error_feedback_new(heading_error_feedback_new);

    // Estimate the lateral error with look-ahead/look-back windows as feedback
    // signal for special driving scenarios
    // Calculate lateral error feedback
    double lateral_error_feedback = 0.0;
    double lateral_error_feedback_new = 0.0;
    if (!IsForwardModel()) {
        lateral_error_feedback = debug_info->lateral_error_lqr_input() - lookback_station * std::sin(debug->heading());
        const double dx = pose_x - lookback_point_new.path_point().x();
        const double dy = pose_y - lookback_point_new.path_point().y();
        const double cos_target_heading = std::cos(lookback_point_new.path_point().theta());
        const double sin_target_heading = std::sin(lookback_point_new.path_point().theta());
        lateral_error_feedback_new = cos_target_heading * dy - sin_target_heading * dx;
    } else {
        lateral_error_feedback = debug_info->lateral_error_lqr_input() + lookahead_station * std::sin(debug->heading());
        const double dx = pose_x - lookahead_point_new.path_point().x();
        const double dy = pose_y - lookahead_point_new.path_point().y();
        const double cos_target_heading = std::cos(lookahead_point_new.path_point().theta());
        const double sin_target_heading = std::sin(lookahead_point_new.path_point().theta());
        lateral_error_feedback_new = cos_target_heading * dy - sin_target_heading * dx;
    }
    debug->set_lateral_error_feedback(lateral_error_feedback);
    debug_info->set_lateral_error_feedback_new(lateral_error_feedback_new);

    ADEBUG << "lateral_error_feedback: " << lateral_error_feedback
           << ", lateral_error_feedback_new: " << lateral_error_feedback_new;
}

void LatPlusController::UpdateDrivingOrientation() {
    auto vehicle_state = injector_->vehicle_state();
    driving_orientation_ = vehicle_state->heading();
    matrix_bd_ = matrix_b_ * ts_;
    // Reverse the driving direction if the vehicle is in reverse mode
    if (FLAGS_reverse_heading_control) {
        if (!IsForwardModel()) {
            driving_orientation_ = common::math::NormalizeAngle(driving_orientation_ + M_PI);
            // Update Matrix_b for reverse mode
            matrix_bd_ = -matrix_b_ * ts_;
            ADEBUG << "Matrix_b changed due to gear direction";
        }
    }
}

bool LatPlusController::IsDrivingStraight(const canbus::Chassis *chassis, const double ref_curvature) {
    // steer < 1.0, v > 0.5, curvature < 0.01
    if (fabs(chassis->steering_percentage())
                < current_lat_based_lqr_plus_controller_conf_.straight_steering_percentage()
        && fabs(injector_->vehicle_state()->linear_velocity())
                > current_lat_based_lqr_plus_controller_conf_.straight_min_velocity()
        && fabs(ref_curvature) < current_lat_based_lqr_plus_controller_conf_.straight_max_ref_cur()) {
        return true;
    } else {
        return false;
    }
}

void LatPlusController::UpdateAHCParam(const double &heading_err, const double &speed) {
    if (AHC_is_compensating_) {
        AHC_container_.clear();
        AHC_sum_ = 0.0;
        AHC_compensated_value_ = AHC_prev_compensated_value_ + AutoCompensatedHeadingValue(speed);
        AHC_compensated_value_ = common::math::Clamp(
                AHC_compensated_value_,
                current_lat_based_lqr_plus_controller_conf_.ahc_mean_value_low_threshold(),
                current_lat_based_lqr_plus_controller_conf_.ahc_mean_value_high_threshold());
        return;
    }

    if (AHC_container_.size() < current_lat_based_lqr_plus_controller_conf_.ahc_container_size()) {
        AHC_container_.push_back(heading_err);
        AHC_sum_ += heading_err;
    }

    AHC_sum_ += heading_err - AHC_container_.front();
    AHC_mean_ = AHC_sum_ / current_lat_based_lqr_plus_controller_conf_.ahc_container_size();
    AHC_peak_ = *std::max_element(AHC_container_.begin(), AHC_container_.end())
            - *std::min_element(AHC_container_.begin(), AHC_container_.end());
    AHC_container_.pop_front();
    AHC_container_.push_back(heading_err);

    if (std::fabs(AHC_mean_) > std::fabs(current_lat_based_lqr_plus_controller_conf_.ahc_mean_value_low_threshold())
        && std::fabs(AHC_mean_) < std::fabs(current_lat_based_lqr_plus_controller_conf_.ahc_mean_value_high_threshold())
        && std::fabs(AHC_peak_) < std::fabs(current_lat_based_lqr_plus_controller_conf_.ahc_peak_value_threshold())) {
        AHC_is_compensating_ = true;
        AHC_compensating_value_ = 0.0;
    }

    // set debug info
    injector_->mutable_control_debug_info()->mutable_simple_lat_debug()->mutable_ahc_value()->set_ahc_sum(AHC_sum_);
    injector_->mutable_control_debug_info()->mutable_simple_lat_debug()->mutable_ahc_value()->set_ahc_mean(AHC_mean_);
    injector_->mutable_control_debug_info()->mutable_simple_lat_debug()->mutable_ahc_value()->set_ahc_peak(AHC_peak_);
    injector_->mutable_control_debug_info()
            ->mutable_simple_lat_debug()
            ->mutable_ahc_value()
            ->set_ahc_compensating_value(AHC_compensating_value_);
    injector_->mutable_control_debug_info()->mutable_simple_lat_debug()->mutable_ahc_value()->set_ahc_compensated_value(
            AHC_compensated_value_);
    injector_->mutable_control_debug_info()->mutable_simple_lat_debug()->mutable_ahc_value()->set_ahc_is_compensating(
            AHC_is_compensating_);
}

double LatPlusController::CalculateTempAHCValue(const double &speed) {
    double diff = AHC_compensated_value_ - AHC_prev_output_value_;
    double change_limit = current_lat_based_lqr_plus_controller_conf_.ts()
            * InterpolationPlus1D::interpolation_1d(
                                  speed,
                                  current_lat_based_lqr_plus_controller_conf_.speed_segment(),
                                  current_lat_based_lqr_plus_controller_conf_.changing_rate());
    diff = common::math::Clamp(diff, -change_limit, change_limit);
    double output_value = AHC_prev_output_value_ + diff;
    AHC_prev_output_value_ = output_value;
    return output_value;
}

double LatPlusController::AutoCompensatedHeadingValue(const double &speed) {
    if (AHC_mean_ > 0) {
        AHC_compensating_value_ += current_lat_based_lqr_plus_controller_conf_.ts()
                * InterpolationPlus1D::interpolation_1d(
                                           speed,
                                           current_lat_based_lqr_plus_controller_conf_.speed_segment(),
                                           current_lat_based_lqr_plus_controller_conf_.changing_rate());
    } else {
        AHC_compensating_value_ -= current_lat_based_lqr_plus_controller_conf_.ts()
                * InterpolationPlus1D::interpolation_1d(
                                           speed,
                                           current_lat_based_lqr_plus_controller_conf_.speed_segment(),
                                           current_lat_based_lqr_plus_controller_conf_.changing_rate());
    }

    // when finish compensating process, update compensated_value
    if (std::fabs(AHC_compensating_value_) > std::fabs(AHC_mean_)) {
        AHC_is_compensating_ = false;
        AHC_compensating_value_ = AHC_mean_;
        AHC_prev_compensated_value_ = AHC_compensated_value_;
    }

    return AHC_compensating_value_;
}

void LatPlusController::AHCClear() {
    AHC_container_.clear();
    AHC_sum_ = 0.0;
    AHC_peak_ = 0.0;
    AHC_mean_ = 0.0;
}

void LatPlusController::EnhanceQParamsAlternate(
        const double lateral_error,
        const double heading_error,
        const planning::ADCTrajectory *planning_published_trajectory,
        const SimpleLongitudinalDebug *previous_lon_debug,
        SimpleLateralDebug *debug,
        SimpleLateralPlusDebug *debug_info) {
    if (injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE) {
        ADEBUG << "[AL new reverse]" << "count_ = " << reverse_handle_count_;
        ADEBUG << "[AL new reverse]" << "heading err = " << debug_info->heading_error_lqr_input();
        ADEBUG << "[AL new reverse]" << "lat err = " << debug_info->lateral_error_lqr_input();
        if (now_reverse_stage_ == ReverseLogicStage::NO_REVERSE) {
            ADEBUG << "[AL new reverse]" << "first in reverse";
            now_reverse_stage_ = ReverseLogicStage::FIRST_HEADING_HANDLE_STAGE;
        } else if (now_reverse_stage_ == ReverseLogicStage::FIRST_HEADING_HANDLE_STAGE) {
            reverse_handle_count_++;
            ADEBUG << "[AL new reverse]" << "in 000000000000 stage damagely change heading ";
            // quit satuation
            if (reverse_handle_count_ >= current_lat_based_lqr_plus_controller_conf_.first_handle_heading_window()
                || debug_info->heading_error_lqr_input()
                        < current_lat_based_lqr_plus_controller_conf_.heading_err_threshold()) {
                ADEBUG << "[AL new reverse]" << "quit 00000000000 stage==============";
                now_reverse_stage_ = ReverseLogicStage::QUIT_FIRST_HEADING_HANDLE_STAGE;
                reverse_handle_count_ = 0;
                efai_rate_ = 1.0;
                ed_date_ = 1.0;
            } else {
                efai_rate_ = current_lat_based_lqr_plus_controller_conf_.first_handle_heading_stage_prarm();
                ed_date_ = 1.0;
            }

        } else if (now_reverse_stage_ == ReverseLogicStage::QUIT_FIRST_HEADING_HANDLE_STAGE) {
            ADEBUG << "[AL new reverse]" << "in 1111111111111 clear stage";
            reverse_handle_count_++;
            if (reverse_handle_count_
                >= current_lat_based_lqr_plus_controller_conf_.out_first_handle_heading_window()) {
                ADEBUG << "[AL new reverse]" << "quit 111111111 stage==============";
                now_reverse_stage_ = ReverseLogicStage::HANDLE_LAT_ERR_STAGE;
                reverse_handle_count_ = 0;
                efai_rate_ = 1.0;
                ed_date_ = 1.0;
            }

        } else if (now_reverse_stage_ == ReverseLogicStage::HANDLE_LAT_ERR_STAGE) {
            ADEBUG << "[AL new reverse]" << "in 222222222 lat err stage";
            reverse_handle_count_++;
            if ((reverse_handle_count_ >= current_lat_based_lqr_plus_controller_conf_.handle_lat_err_window())
                || (std::fabs(debug_info->lateral_error_lqr_input())
                    < current_lat_based_lqr_plus_controller_conf_.lat_err_threshold())
                || (std::fabs(debug->curvature())
                    < current_lat_based_lqr_plus_controller_conf_.openspace_enhance_curv())
                || (std::fabs(previous_lon_debug->path_remain())
                    < current_lat_based_lqr_plus_controller_conf_.reverse_final_heading_check_path())) {
                ADEBUG << "[AL new reverse]" << "quit 222222222 stage==============";
                now_reverse_stage_ = ReverseLogicStage::HANDLE_HEADING_ERR_STAGE;
                reverse_handle_count_ = 0;
                efai_rate_ = 1.0;
                ed_date_ = 1.0;
            } else if (
                    (pre_steer_angle_ <= (-1) * current_lat_based_lqr_plus_controller_conf_.reverse_dead_steer_left())
                    && (debug_info->heading_error_lqr_input()
                        > current_lat_based_lqr_plus_controller_conf_.reverse_too_big_heading_err())
                    && (debug_info->lateral_error_lqr_input()
                        > current_lat_based_lqr_plus_controller_conf_.reverse_too_big_lat_err())) {
                // right behind steer
                AINFO << "[STEERDEAD quit ,right behind ]" << "quit lat stage=======";
                now_reverse_stage_ = ReverseLogicStage::HANDLE_HEADING_ERR_STAGE;
                reverse_handle_count_ = 0;
                efai_rate_ = 1.0;
                ed_date_ = 1.0;
            } else if (
                    (pre_steer_angle_ >= current_lat_based_lqr_plus_controller_conf_.reverse_dead_steer_left())
                    && (debug_info->heading_error_lqr_input()
                        < (-1) * current_lat_based_lqr_plus_controller_conf_.reverse_too_big_heading_err())
                    && (debug_info->lateral_error_lqr_input()
                        < (-1) * current_lat_based_lqr_plus_controller_conf_.reverse_too_big_lat_err())) {
                // left behind steer
                AINFO << "[STEERDEAD quit ,left behind ]" << "quit lat stage=======";
                now_reverse_stage_ = ReverseLogicStage::HANDLE_HEADING_ERR_STAGE;
                reverse_handle_count_ = 0;
                efai_rate_ = 1.0;
                ed_date_ = 1.0;
            } else {
                efai_rate_ = current_lat_based_lqr_plus_controller_conf_.handle_lat_err_prarm_ignore();
                ed_date_ = current_lat_based_lqr_plus_controller_conf_.handle_lat_err_prarm_domain();
            }

        } else if (now_reverse_stage_ == ReverseLogicStage::HANDLE_HEADING_ERR_STAGE) {
            ADEBUG << "[AL new reverse]" << "in 33333333333 heading err stage";
            reverse_handle_count_++;
            if (reverse_handle_count_ >= current_lat_based_lqr_plus_controller_conf_.handle_heading_err_window()
                || std::fabs(debug_info->heading_error_lqr_input())
                        < current_lat_based_lqr_plus_controller_conf_.heading_err_threshold()
                || std::fabs(debug->curvature())
                        < current_lat_based_lqr_plus_controller_conf_.openspace_enhance_curv()) {
                ADEBUG << "[AL new reverse]" << "quit 333333333 stage==============";
                now_reverse_stage_ = ReverseLogicStage::HANDLE_LAT_ERR_STAGE;
                reverse_handle_count_ = 0;
                efai_rate_ = 1.0;
                ed_date_ = 1.0;
            } else {
                efai_rate_ = current_lat_based_lqr_plus_controller_conf_.handle_heading_err_parm_domain();
                ed_date_ = current_lat_based_lqr_plus_controller_conf_.handle_heading_err_parm_ignore();
            }
        } else {
            ADEBUG << "[AL new reverse]" << "wrong!!!!! don't change any param";
        }
    }
    if (injector_->vehicle_state()->gear() != canbus::Chassis::GEAR_REVERSE) {
        if (now_reverse_stage_ != ReverseLogicStage::NO_REVERSE) {
            ADEBUG << "[AL new reverse]" << "quit reverse ,clear all marks";
            now_reverse_stage_ = ReverseLogicStage::NO_REVERSE;
            reverse_handle_count_ = 0;
            efai_rate_ = 1.0;
            ed_date_ = 1.0;
        }
    }

    if ((injector_->vehicle_state()->gear() == canbus::Chassis::GEAR_REVERSE)
        && (std::fabs(previous_lon_debug->path_remain())
            < current_lat_based_lqr_plus_controller_conf_.reverse_final_heading_check_path())
        && (std::fabs(debug_info->heading_error_lqr_input())
            > current_lat_based_lqr_plus_controller_conf_.reverse_final_heading_check_heading())
        && (planning_published_trajectory->trajectory_type() == ADCTrajectory::OPEN_SPACE)) {
        ADEBUG << "[FINALCHECK]final check in reverse park";
        efai_rate_ = current_lat_based_lqr_plus_controller_conf_.handle_heading_err_parm_domain();
        ed_date_ = current_lat_based_lqr_plus_controller_conf_.handle_heading_err_parm_ignore();
    }

    debug_info->set_now_reverse_stage(now_reverse_stage_);
}

void LatPlusController::LateralSafetyCheck(SimpleLateralDebug *debug, SimpleLateralPlusDebug *debug_info) {
    u_int32_t max_uint32 = std::numeric_limits<uint32_t>::max();
    auto check_conf = current_lat_based_lqr_plus_controller_conf_.lat_safety_check_conf();
    auto lat_check_debug = debug_info->mutable_lat_control_check_debug();
    // lateral error check
    double lateral_error
            = enable_look_ahead_back_control_ ? debug->lateral_error_feedback() : debug_info->lateral_error_lqr_input();
    ControlCheckStatus lateral_error_failed = ControlChecker::check_lateral_error(check_conf, &lateral_error);
    if (lateral_error_failed == WARNING) {
        lateral_error_w_count_ = lateral_error_w_count_ % max_uint32;
        lateral_error_w_count_++;
        lat_check_debug->set_lateral_error_w(lateral_error);
    } else {
        lat_check_debug->set_lateral_error_w(0.0);
    }

    if (lateral_error_failed == ERROR) {
        lateral_error_e_count_ = lateral_error_e_count_ % max_uint32;
        lateral_error_e_count_++;
        lat_check_debug->set_lateral_error_e(lateral_error);
    } else {
        lat_check_debug->set_lateral_error_e(0.0);
    }
    lat_check_debug->set_lateral_error_check_result(lateral_error_failed);
    lat_check_debug->set_lateral_error_check_e_count(lateral_error_e_count_);
    lat_check_debug->set_lateral_error_check_w_count(lateral_error_w_count_);

    if (current_lat_based_lqr_plus_controller_conf_.use_safety_check_output()) {
        if (enable_look_ahead_back_control_) {
            debug->set_lateral_error_feedback(lateral_error);
        } else {
            debug_info->set_lateral_error_lqr_input(lateral_error);
        }
    }
}

void LatPlusController::ResetLateralSafetyCheck() {
    lateral_error_w_count_ = 0;
    lateral_error_e_count_ = 0;
}

}  // namespace control
}  // namespace apollo
