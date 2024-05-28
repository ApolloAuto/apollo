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

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/control_component/proto/preprocessor.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/control/control_component/controller_task_base/common/dependency_injector.h"
#include "modules/control/control_component/controller_task_base/control_task_agent.h"
#include "modules/control/control_component/submodules/preprocessor_submodule.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {
struct PIDParam {
    PIDParam(double kp,double ki, double kd, double inter, 
      double output, bool is_steer):kp_(kp), ki_(ki), kd_(kd), 
      integrator_limit_level_(inter), output_limit_level_(output), 
      is_steer_(is_steer){}
    double kp_;
    double ki_;
    double kd_;
    double integrator_limit_level_;
    double output_limit_level_;
    bool is_steer_;
};

class PidControl {
protected:
    bool first_hit_ = true;
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double integral_        = 0.0;
    double previous_error_  = 0.0;
    double previous_output_ = 0.0;
    double integrator_limit_level_= 0.0;
    double output_limit_level_ = 0.0;
    bool is_steer_ = true;  
    // std::chrono::high_resolution_clock::time_point
    //     last_time_point_;
public:
     ~PidControl() = default;
    void SetPIDParam(const PIDParam &pidparam);
    void Init(const PIDParam &pidparam);
    void Reset();
    double ComputePID(const double error, double dt, const double min);
    void SetP(double p) { kp_ += p;}
    void SetI(double i) {ki_ += i;}
    void SetD(double d) {kd_ += d;}
    double GetP() {return kp_;}
    double GetI() {return ki_;}
    double GetD() {return kd_;}
};



/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chassis, and
 * pad data to compute throttle, brake and steer values.
 */
class ControlComponent final : public apollo::cyber::TimerComponent {
  friend class ControlTestBase;

 public:
  ControlComponent();
  bool Init() override;

  bool Proc() override;

 private:
  // Upon receiving pad message
  void OnPad(const std::shared_ptr<PadMessage> &pad);

  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);

  void OnPlanning(
      const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory);

  void OnPlanningCommandStatus(
      const std::shared_ptr<external_command::CommandStatus>
          &planning_command_status);

  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);

  // Upon receiving monitor message
  void OnMonitor(
      const apollo::common::monitor::MonitorMessage &monitor_message);

  common::Status ProduceControlCommand(ControlCommand *control_command);
  common::Status CheckInput(LocalView *local_view);
  common::Status CheckTimestamp(const LocalView &local_view);
  common::Status CheckPad();
  void ResetAndProduceZeroControlCommand(ControlCommand *control_command);
  void GetVehiclePitchAngle(ControlCommand *control_command);



  void CheckJoy();
  void OnKeyBoard(double &cur_brake_v, double &cur_thro_v, 
  double &cur_steer_angle, ControlCommand &control_command);
  void set_terminal_echo(bool enabled);

 private:
  apollo::cyber::Time init_time_;

  localization::LocalizationEstimate latest_localization_;
  canbus::Chassis latest_chassis_;
  planning::ADCTrajectory latest_trajectory_;
  external_command::CommandStatus planning_command_status_;
  PadMessage pad_msg_;
  common::Header latest_replan_trajectory_header_;

  ControlTaskAgent control_task_agent_;

  bool estop_ = false;
  std::string estop_reason_;
  bool pad_received_ = false;

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  ControlPipeline control_pipeline_;

  std::mutex mutex_;

  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      trajectory_reader_;
  std::shared_ptr<cyber::Reader<apollo::external_command::CommandStatus>>
      planning_command_status_reader_;

  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;
  // when using control submodules
  std::shared_ptr<cyber::Writer<LocalView>> local_view_writer_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  LocalView local_view_;

  std::shared_ptr<DependencyInjector> injector_;


  PidControl steer_pidcontrol_;  //方向盘PID控制
  PidControl throttle_pidcontrol_; //油门PID控制
  PidControl brake_pidcontrol_; //刹车PID控制
  std::unique_ptr<std::thread> thread_;
  double steer_target_value_ = 0.0; //方向盘目标转角
  double brake_target_value_ = 2.5; //刹车目标电压值
  double thro_target_value_ = 1.5; //油门目标电压值
  bool is_steer_control_ = false; //是否收到转向命令
  double wheel_angle_ = 0.0; //车轮目标转角，与方向盘目标转角为1:18
  bool key_up_    = false;
  bool key_down_  = false;
  bool key_left_  = false;
  bool key_right_ = false;
  bool key_shift_ = false;
  bool key_p_     = false;
  bool is_first = true;
  bool is_switch = true;

  double scale = 14.95;
  //double scale = 16.68;
  //double scale = 17.25;
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo
