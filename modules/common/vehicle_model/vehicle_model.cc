/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/vehicle_model/vehicle_model.h"

#include "cyber/common/file.h"
#include "modules/common/configs/config_gflags.h"

namespace apollo {
namespace common {
/// @brief 基于后置中心运动学自行车模型预测车辆的未来状态
/// @param vehicle_model_config 车辆模型的配置参数
/// @param predicted_time_horizon 预测的时间范围，表示从当前时刻开始，预测的车辆状态持续的时间：规划周期 100ms
/// @param cur_vehicle_state 当前车辆的状态，包含位置、速度、加速度、航向角等信息
/// @param predicted_vehicle_state 指向预测车辆状态的指针，函数会修改该指针指向的状态
void VehicleModel::RearCenteredKinematicBicycleModel(
    const VehicleModelConfig& vehicle_model_config,
    const double predicted_time_horizon, const VehicleState& cur_vehicle_state,
    VehicleState* predicted_vehicle_state) {
  // Kinematic bicycle model centered at rear axis center by Euler forward
  // discretization
  // Assume constant control command and constant z axis position
  // 模型假设控制命令是常量的，并且车辆沿z轴的运动是恒定的
  // 预测的时间范围必须是正值
  CHECK_GT(predicted_time_horizon, 0.0);
  // 配置中获取时间步长（dt），用于更新车辆状态  0.06
  double dt = vehicle_model_config.rc_kinematic_bicycle_model().dt();
  double cur_x = cur_vehicle_state.x();
  double cur_y = cur_vehicle_state.y();
  double cur_z = cur_vehicle_state.z();
  // 车辆的航向角（方向角）
  double cur_phi = cur_vehicle_state.heading();
  // 车辆的线速度
  double cur_v = cur_vehicle_state.linear_velocity();
  // 车辆的线性加速度
  double cur_a = cur_vehicle_state.linear_acceleration();
  double next_x = cur_x;
  double next_y = cur_y;
  double next_phi = cur_phi;
  double next_v = cur_v;
  // 确保在预测时间内不会超出
  if (dt >= predicted_time_horizon) {
    dt = predicted_time_horizon;
  }

  // 初始化倒计时（countdown_time）为预测时间范围 100ms
  double countdown_time = predicted_time_horizon;
  // 标记是否完成预测
  bool finish_flag = false;
  // 用于在倒计时接近结束时，避免出现数值误差问题
  static constexpr double kepsilon = 1e-8;
  while (countdown_time > kepsilon && !finish_flag) {
    countdown_time -= dt;
    if (countdown_time < kepsilon) {
      dt = countdown_time + dt;
      finish_flag = true;
    }
    // 采用了运动学单轨模型的欧拉前向法来离散化车辆运动
    // 估算车辆在当前时间步长内的中间航向角（phi）
    double intermidiate_phi =
        cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa();
    next_phi =
        cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_vehicle_state.kappa();
    next_x =
        cur_x + dt * (cur_v + 0.5 * dt * cur_a) * std::cos(intermidiate_phi);
    next_y =
        cur_y + dt * (cur_v + 0.5 * dt * cur_a) * std::sin(intermidiate_phi);

    next_v = cur_v + dt * cur_a;
    //更新当前状态为下一时刻的状态，准备进行下一次的迭代
    cur_x = next_x;
    cur_y = next_y;
    cur_phi = next_phi;
    cur_v = next_v;
  }

  predicted_vehicle_state->set_x(next_x);
  predicted_vehicle_state->set_y(next_y);
  predicted_vehicle_state->set_z(cur_z);
  predicted_vehicle_state->set_heading(next_phi);
  predicted_vehicle_state->set_kappa(cur_vehicle_state.kappa());
  predicted_vehicle_state->set_linear_velocity(next_v);
  predicted_vehicle_state->set_linear_acceleration(
      cur_vehicle_state.linear_acceleration());
}
/// @brief 根据当前车辆的状态（cur_vehicle_state）和预测的时间范围（predicted_time_horizon），使用指定的车辆模型来预测车辆在未来一段时间内的状态，并返回预测结果
/// @param predicted_time_horizon 预测的时间范围，表示从当前时刻起，预测未来的这段时间内车辆的状态  规划周期
/// @param cur_vehicle_state 当前车辆的状态，包含了如位置、速度、加速度、方向等信息
/// @return 
VehicleState VehicleModel::Predict(const double predicted_time_horizon,
                                   const VehicleState& cur_vehicle_state) {
// 保存车辆模型的配置
  VehicleModelConfig vehicle_model_config;
// apollo/modules/common/configs/config_gflags.cc
// 配置文件是一个protobuf文件，GetProtoFromFile函数会将文件内容加载到vehicle_model_config对象中。如果加载失败，会输出错误信息并终止程序
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_vehicle_model_config_filename,
                                         &vehicle_model_config))
      << "Failed to load vehicle model config file "
      << FLAGS_vehicle_model_config_filename;

  // Some models not supported for now
  // 检查车辆模型的类型，确保当前使用的车辆模型类型不是不支持的模型类型
  ACHECK(vehicle_model_config.model_type() !=
         VehicleModelConfig::COM_CENTERED_DYNAMIC_BICYCLE_MODEL);
  ACHECK(vehicle_model_config.model_type() != VehicleModelConfig::MLP_MODEL);

// 保存预测得到的车辆状态
  VehicleState predicted_vehicle_state;
  // 一种基于后置中心运动学单轨模型的车辆运动模型： 利用自行车模型预测未来的位置
  if (vehicle_model_config.model_type() ==
      VehicleModelConfig::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL) {
  // 车辆状态预测
    RearCenteredKinematicBicycleModel(vehicle_model_config,
                                      predicted_time_horizon, cur_vehicle_state,
                                      &predicted_vehicle_state);
  }

  return predicted_vehicle_state;
}

}  // namespace common
}  // namespace apollo
