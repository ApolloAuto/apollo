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

/**
 * @file
 **/

#pragma once

#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common_msgs/localization_msgs/pose.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/indexed_queue.h"
#include "modules/planning/planning_base/common/local_view.h"
#include "modules/planning/planning_base/common/obstacle.h"
#include "modules/planning/planning_base/common/open_space_info.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.  Frame类储存了一个规划周期的所有数据
 */

class Frame {
 public:
  /// @brief 
  /// @param sequence_num 序列号
  explicit Frame(uint32_t sequence_num);
  /// @brief 
  /// @param sequence_num 
  /// @param local_view LocalView类包含了规划模块输入所需的所有数据
  /// @param planning_start_point 规划轨迹起始点
  /// @param vehicle_state 车辆状态
  /// @param reference_line_provider 参考线提供器
  Frame(uint32_t sequence_num, const LocalView &local_view,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state,
        ReferenceLineProvider *reference_line_provider);

  Frame(uint32_t sequence_num, const LocalView &local_view,
        const common::TrajectoryPoint &planning_start_point,
        const common::VehicleState &vehicle_state);
  //析构函数
  virtual ~Frame() = default;
  //获取Frame类的数据成员规划起始点planning_start_point_
  const common::TrajectoryPoint &PlanningStartPoint() const;
  /// @brief 用输入参数去初始化Frame类的数据成员
  /// @param vehicle_state_provider 车辆状态提供器类指针对象
  /// @param reference_lines 参考线类对象列表
  /// @param segments 导航路径段类对象列表
  /// @param future_route_waypoints 将来的导航的车道航点序列
  /// @param ego_info 自车信息类指针对象
  /// @return 
  common::Status Init(
      const common::VehicleStateProvider *vehicle_state_provider,
      const std::list<ReferenceLine> &reference_lines,
      const std::list<hdmap::RouteSegments> &segments,
      const std::vector<routing::LaneWaypoint> &future_route_waypoints,
      const EgoInfo *ego_info);

  common::Status InitForOpenSpace(
      const common::VehicleStateProvider *vehicle_state_provider,
      const EgoInfo *ego_info);

  uint32_t SequenceNum() const;

  std::string DebugString() const;
  // 返回待发布的规划轨迹
  const PublishableTrajectory &ComputedTrajectory() const;
  
  //其实这个函数就记录planning的所有输入数据用于debug
  //这些输入数据几乎都是从Frame类成员local_view_里拷贝的
  //planning_internal::Debug 是modules\planning\proto\planning_internal.proto
  //里定义的message类，Debug下面包含planning_data
  //planning_data里又包含规划的相关数据，如自车位置，底盘反馈，routing响应，
  //起始点，路径，速度规划，st图，sl坐标，障碍物，信号灯，前方畅通距离，场景信息等等，几
  //乎所有的planning输入debug数据
  void RecordInputDebug(planning_internal::Debug *debug);
  //获取Frame类成员参考线信息类对象reference_line_info_，它是一个参考线信息的列表list
  const std::list<ReferenceLineInfo> &reference_line_info() const;
  //返回Frame类成员参考线信息类对象reference_line_info_的地址
  std::list<ReferenceLineInfo> *mutable_reference_line_info();
  //在障碍物列表中寻找输入参数id对应的障碍物对象
  Obstacle *Find(const std::string &id);
  //找到Frame类数据成员reference_line_info_道路参考线列表中cost最小且可行驶的道路参考线对象
  const ReferenceLineInfo *FindDriveReferenceLineInfo();
  //找目标道路参考线，返回的是道路参考线信息ReferenceLineInfo类对象
  const ReferenceLineInfo *FindTargetReferenceLineInfo();
  //查找失败的参考线信息，返回道路参考线信息类ReferenceLineInfo类对象
  //遍历道路参考线列表，找到不可驾驶的变道参考线信息类对象
  const ReferenceLineInfo *FindFailedReferenceLineInfo();
  //返回Frame类的数据成员驾驶参考线信息类对象
  const ReferenceLineInfo *DriveReferenceLineInfo() const;
  //返回Frame类的数据成员障碍物列表obstacles_
  const std::vector<const Obstacle *> obstacles() const;
  /// @brief 对虚拟静态障碍物的位置建立虚拟障碍物对象，如红绿灯停止线等
  /// @param reference_line_info 参考线信息类对象reference_line_info，这里不是列表了，只是一条参考线信息
  /// @param obstacle_id 障碍物obstacle_id,引用变量&应该是用来存放构建的虚拟障碍物的Id
  /// @param obstacle_s 障碍物对应的frenet系的s坐标
  /// @return 
  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);
  /// @brief 
  /// @param obstacle_id 
  /// @param lane_id 
  /// @param lane_s 车道对应的终点的frenet系s坐标lane_s
  /// @return 
  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

  bool Rerouting(PlanningContext *planning_context);

  const common::VehicleState &vehicle_state() const;
  /// @brief 实现预测时间对齐planning起始时间
  /// @param planning_start_time 
  /// @param prediction_obstacles 
  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);
  //设置当前frame帧的规划轨迹，用ADCTrajectory类轨迹对象
  void set_current_frame_planned_trajectory(
      ADCTrajectory current_frame_planned_trajectory) {
    current_frame_planned_trajectory_ =
        std::move(current_frame_planned_trajectory);
  }

  const ADCTrajectory &current_frame_planned_trajectory() const {
    return current_frame_planned_trajectory_;
  }
  //设置当前frame的离散路径 轨迹trajectory=路径path + 速度规划
  void set_current_frame_planned_path(
      DiscretizedPath current_frame_planned_path) {
    current_frame_planned_path_ = std::move(current_frame_planned_path);
  }

  const DiscretizedPath &current_frame_planned_path() const {
    return current_frame_planned_path_;
  }
  //离目标点是否足够近
  const bool is_near_destination() const { return is_near_destination_; }

  /**
   * @brief Adjust reference line priority according to actual road conditions
   * @id_to_priority lane id and reference line priority mapping relationship
   */
  void UpdateReferenceLinePriority(
      const std::map<std::string, uint32_t> &id_to_priority);
  //返回frame类的数据成员local_view_对象，其包含了所有的规划输入数据，是一个struct
  //障碍物/底盘/交通灯/定位等信息
  const LocalView &local_view() const { return local_view_; }
  //获取Frame类数据成员障碍物列表
  ThreadSafeIndexedObstacles *GetObstacleList() { return &obstacles_; }
  //返回开放空间信息类对象
  const OpenSpaceInfo &open_space_info() const { return open_space_info_; }

  OpenSpaceInfo *mutable_open_space_info() { return &open_space_info_; }
  //根据id获取交通灯对象，输入的是交通灯的id
  perception::TrafficLight GetSignal(const std::string &traffic_light_id) const;
  //获取frame类的数据成员存放人机交互动作对象
  const PadMessage::DrivingAction &GetPadMsgDrivingAction() const {
    return pad_msg_driving_action_;
  }

 private:
  /// @brief 
  /// @param vehicle_state_provider 车辆状态提供器类指针对象
  /// @param ego_info 自车信息类指针对象
  /// @return 
  common::Status InitFrameData(
      const common::VehicleStateProvider *vehicle_state_provider,
      const EgoInfo *ego_info);
  /// @brief 
  /// @param reference_lines 参考线类对象列表reference_lines
  /// @param segments 导航路径段类对象列表segments  导航route指全局路径规划
  /// @return 
  //其实这个函数就是将参数里的参考线列表reference_lines，导航路径段RouteSegments列表设
//置到Frame类数据成员reference_line_info_参考线信息类对象列表里
  bool CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines,
                               const std::list<hdmap::RouteSegments> &segments);

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle(const EgoInfo *ego_info) const;

  /**
   * @brief create a static virtual obstacle
   */
     //根据障碍物id和障碍物对应的边界盒对象作为参数去调用构建虚拟障碍物的函数，
  //返回一个虚拟障碍物对象类
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);
    //增加一个障碍物对象到类数据成员障碍物列表里
  void AddObstacle(const Obstacle &obstacle);
  //读交通灯函数，从数据成员local_view_读到Frame类数据成员traffic_lights_交通灯列表中
//local_view_包含规划的所有输入数据
  void ReadTrafficLights();
 //读取local_view_中的驾驶员交互操作行为
  void ReadPadMsgDrivingAction();
  //复位人机交互动作对象
  void ResetPadMsgDrivingAction();

 private:
  //人机交互动作，自驾模式选择等...
  static PadMessage::DrivingAction pad_msg_driving_action_;
  // 序列号
  uint32_t sequence_num_ = 0;
  //LocalView类对象，包含规划所有输入数据
  LocalView local_view_;
  //高精度地图对象，初始化为空指针
  const hdmap::HDMap *hdmap_ = nullptr;
  //规划起始点
  common::TrajectoryPoint planning_start_point_;
  //车辆状态
  common::VehicleState vehicle_state_;
  //参考线列表
  std::list<ReferenceLineInfo> reference_line_info_;
  //是否接近目标点？
  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  // 车辆最终选择驾驶的道路参考线
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;
  //障碍物列表
  ThreadSafeIndexedObstacles obstacles_;
  //无序map，存放id,和交通灯对象的映射关系
  std::unordered_map<std::string, const perception::TrafficLight *>
      traffic_lights_;

  // current frame published trajectory
  // 当前帧的规划发布轨迹，也就是输出结果
  ADCTrajectory current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
  //当前帧的路径点，为了将来可能的speed fallback速度规划失败？
  DiscretizedPath current_frame_planned_path_;
  //参考线提供器类对象
  const ReferenceLineProvider *reference_line_provider_ = nullptr;
  //开放空间信息类对象？非结构化道路？
  OpenSpaceInfo open_space_info_;
//将来的路由点vector，用于rerouting时将自车在轨迹上最近点与之拼接
//用于将之前设定的routing请求改为从自车位置出发
  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 public:
  FrameHistory();
};

}  // namespace planning
}  // namespace apollo
