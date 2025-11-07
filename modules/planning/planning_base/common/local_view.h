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

// C++ 标准库，提供了 智能指针（如 std::shared_ptr） 的支持，能自动管理内存，防止 内存泄漏
#include <memory>

// 以下都是proto生成的
// 车辆底盘状态（如速度、档位等）
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
// 车辆的定位信息（如经纬度、方向等）
#include "modules/common_msgs/localization_msgs/localization.pb.h"
// 交通信号灯检测结果
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
// 导航相关信息
#include "modules/common_msgs/planning_msgs/navigation.pb.h"
// 用户的驾驶模式选择（如手动/自动驾驶）
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
// 	规划模块下达的指令
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
// 预测的障碍物信息（如行人、车辆等动态目标）
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
// 路径规划的路由信息
#include "modules/common_msgs/routing_msgs/routing.pb.h"
// 叙事模块的内容
#include "modules/common_msgs/storytelling_msgs/story.pb.h"

namespace apollo {
namespace planning {

/**
 * @struct local_view
 // 存储规划所需的 所有输入数据
 * @brief LocalView contains all necessary data as planning input
 */

// LocalView 存储路径规划所需的传感器数据、地图信息和控制指令
struct LocalView {
  // 预测模块输出的 障碍物运动轨迹
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  // 车辆底盘信息，如 速度、加速度、档位 等
  std::shared_ptr<canbus::Chassis> chassis;
  // 车辆的 位置信息（经纬度、方向等）
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  // 交通灯识别结果
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  // 相对地图信息
  std::shared_ptr<relative_map::MapMsg> relative_map;
  // 用户的驾驶模式选择（如 自动驾驶/手动驾驶）
  std::shared_ptr<PadMessage> pad_msg;
  // 叙事模块的状态
  std::shared_ptr<storytelling::Stories> stories;
  // 规划模块下达的 指令
  std::shared_ptr<PlanningCommand> planning_command;
  // 终点车道的 路径信息
  std::shared_ptr<routing::LaneWaypoint> end_lane_way_point;
};

}  // namespace planning
}  // namespace apollo
