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

#include "modules/planning/planning_component/on_lane_planning.h"

#include <algorithm>  // 如 std::sort, std::find 等，用于轨迹点排序、参考线选择等
#include <limits>     // 获取数值类型极值（如 std::numeric_limits<double>::max()），常用于初始化最小/最大距离
#include <list>       // 可能用于存储历史轨迹或障碍物列表（虽然 Apollo 更常用 vector）
#include <utility>    // 提供 std::pair, std::move 等，用于高效数据传递（如移动语义）
// 作用：允许 Google Test 框架访问 OnLanePlanning 的 私有成员函数
// 机制：通过 FRIEND_TEST 宏，测试代码可调用内部逻辑进行白盒测试
// 注意：此头文件 不影响运行时行为，仅在编译测试时生效
#include "gtest/gtest_prod.h"
// 作用：使用 absl::StrCat() 高效拼接字符串（比 std::string + 更快、更安全）
#include "absl/strings/str_cat.h"
// 包含调试用的内部数据结构（如 ST 图、SL 图、参考线信息）
#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"
// 接收来自 Routing 模块的导航路径（RoutingResponse），是规划的起点
#include "modules/common_msgs/routing_msgs/routing.pb.h"
// 定义语义地图相关配置（如路口放大区域、虚拟车道等）
#include "modules/planning/planning_base/proto/planning_semantic_map_config.pb.h"
// Cyber RT 基础设施（Apollo 的中间件
// 文件操作（如读取配置文件、保存调试数据）
#include "cyber/common/file.h"
// 定义 AINFO, AWARN, AERROR, ADEBUG 等日志宏
#include "cyber/common/log.h"
// 高精度时间获取（Clock::Now()），用于时间戳对齐、性能分析
#include "cyber/time/clock.h"
// 提供四元数运算，用于处理车辆姿态（从定位消息中提取 heading）
#include "modules/common/math/quaternion.h"
// 封装车辆状态（位置、速度、加速度、航向、角速度等）
#include "modules/common/vehicle_state/vehicle_state_provider.h"
// 提供高精地图查询工具
// 根据 GPS 坐标查找最近车道
// 获取车道边界、限速、交通灯关联关系
// 判断是否在交叉口、匝道等
#include "modules/map/hdmap/hdmap_util.h"
// 规划核心公共组件
// 封装自车（Ego Vehicle）相关信息，如前方净空距离、是否在路口等
#include "modules/planning/planning_base/common/ego_info.h"
// 管理历史帧缓存（FrameHistory），用于轨迹平滑、预测等
#include "modules/planning/planning_base/common/history.h"
// 全局规划上下文，存储跨帧状态（如变道意图、红灯等待状态）
#include "modules/planning/planning_base/common/planning_context.h"
// 实现轨迹“缝合”逻辑，保证新旧轨迹平滑衔接（避免跳变）
#include "modules/planning/planning_base/common/trajectory_stitcher.h"
// 通用工具函数，如坐标转换、插值、角度归一化等
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"
// 动态生成参考线（Reference Line），是 Frenet 坐标系规划的基础
// 支持多参考线（如直行、变道）并行评估
#include "modules/planning/planning_base/reference_line/reference_line_provider.h"
// 定义规划器抽象接口（Planner），具体实现如 PublicRoadPlanner, LatticePlanner 等
// 采用 策略模式（Strategy Pattern），支持插件化替换
#include "modules/planning/planning_interface_base/planner_base/planner.h"
// 应用交通规则（Traffic Rules）对每条参考线进行可行性判断
// 包括：红绿灯、让行、限速、行人避让等子规则
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_decider.h"

namespace apollo {
namespace planning {
using apollo::canbus::Chassis;
using apollo::common::EngageAdvice;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::dreamview::Chart;
using apollo::hdmap::HDMapUtil;
using apollo::planning_internal::SLFrameDebug;
using apollo::planning_internal::SpeedPlan;
using apollo::planning_internal::STGraphDebug;
/// @brief 自动设置 Dreamview 图表的 X/Y 轴范围和标签
/// @param chart 图表指针
/// @param label_name_x X轴标签名
/// @param label_name_y Y轴标签名
void SetChartminmax(apollo::dreamview::Chart* chart, std::string label_name_x,
                    std::string label_name_y) {
  auto* options = chart->mutable_options();
  // 初始化极值变量，用于遍历所有数据点后确定坐标轴范围
  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());
  // 遍历图表中所有折线（line）的所有点（point），更新 X/Y 的最小/最大值
  for (int i = 0; i < chart->line_size(); i++) {
    auto* line = chart->mutable_line(i);
    for (auto& pt : line->point()) {
      xmin = std::min(xmin, pt.x());
      ymin = std::min(ymin, pt.y());
      xmax = std::max(xmax, pt.x());
      ymax = std::max(ymax, pt.y());
    }
    // 设置折线样式：线宽 2px、无点标记、直线连接（无线条张力）、不填充区域、显示线条
    // 这些是 Chart.js（Dreamview 前端图表库）的配置项
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
  }
  // 将计算出的范围和标签应用到图表的 X/Y 轴
  options->mutable_x()->set_min(xmin);
  options->mutable_x()->set_max(xmax);
  options->mutable_x()->set_label_string(label_name_x);
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string(label_name_y);
  // Set chartJS's dataset properties
}
/// @brief 释放资源，停止后台线程，清空状态
OnLanePlanning::~OnLanePlanning() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  // 清空历史缓存、规划上下文、自车信息等，防止内存泄漏或状态残留
  injector_->frame_history()->Clear();
  injector_->history()->Clear();
  injector_->planning_context()->mutable_planning_status()->Clear();
  last_command_.Clear();
  injector_->ego_info()->Clear();
}

std::string OnLanePlanning::Name() const { return "on_lane_planning"; }
/// @brief 分配具体的Planner，启动参考线提供器(reference_line_provider_)
/// @param config 
/// @return 
/*
1.检查 PlanningConfig，确保配置正确。
2.清理历史数据，避免干扰。
3.加载高精地图，为规划提供地图支持。
4.初始化 ReferenceLineProvider，计算参考线。
5.创建 planner_（路径规划器），加载不同的规划算法。
6.启用强化学习（如果配置允许），用于训练自动驾驶模型。
7.初始化 traffic_decider_，处理交通规则。
8.记录启动时间，初始化 planner_
*/
// planning_config.pb.txt
Status OnLanePlanning::Init(const PlanningConfig& config) {
  if (!CheckPlanningConfig(config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config_.DebugString());
  }
  // 调用父类的Init()函数
  // 调用基类初始化，并清空前序状态
  PlanningBase::Init(config_);  // 会将config_赋值给PlanningBase中的成员变量

  // clear planning history
  // 清除先前的规划历史数据
  // OnLanePlanning公有继承PlanningBase， 且injecteor_是PlanningBase的protocted成员变量
  injector_->history()->Clear();
//
  // clear planning status
  // 清除当前的规划状态
  injector_->planning_context()->mutable_planning_status()->Clear();

  // load map
  // 使用 HDMapUtil::BaseMapPtr() 获取地图指针，并检查地图是否成功加载。如果地图加载失败，会触发断言并输出错误信息
  // 加载高精地图，失败则终止（ACHECK 是 fatal check）
  hdmap_ = HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map";

  // instantiate reference line provider
  // 创建 ReferenceLineProvider 实例并启动它，提供参考路线。如果配置中包含参考路线配置，将其传递给提供者
  // 创建并启动参考线提供器（内部可能有独立线程预计算参考线）
  const ReferenceLineConfig* reference_line_config = nullptr;
  if (config_.has_reference_line_config()) {
    reference_line_config = &config_.reference_line_config();
  }
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(
      injector_->vehicle_state(), reference_line_config);
  // 调用 Start() 开始提供参考线数据
  reference_line_provider_->Start();
  
  // 加载路径规划器并检查其是否成功初始化。如果没有成功初始化，返回错误状态
  // dispatch planner
  // 加载规划器
  // 根据配置加载具体 Planner（如 PublicRoadPlanner）
  LoadPlanner();  // 在父类中定义，选择EM 或者 Lattice
  if (!planner_) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }
  // 如果启用了学习模式，加载并初始化图像特征渲染器配置。如果加载失败，输出错误信息
  // 若启用学习模式，则初始化图像特征渲染器
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    PlanningSemanticMapConfig renderer_config;
    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planning_birdview_img_feature_renderer_config_file,
        &renderer_config))
        << "Failed to load renderer config"
        << FLAGS_planning_birdview_img_feature_renderer_config_file;

    BirdviewImgFeatureRenderer::Instance()->Init(renderer_config);
  }
  // 初始化交通规则决策器，并调用 Planner 的 Init 完成最终初始化
  // 初始化交通决策模块
  // traffic_decider_ 负责处理交通信号灯、行人、让行规则等
  traffic_decider_.Init(injector_);
  // 获取当前时间并保存为 start_time_，以记录路径规划的开始时间
  start_time_ = Clock::NowInSeconds();
  // 初始化路径规划器，并使用配置路径 FLAGS_planner_config_path 来进行初始化
  // 从LoadPlanner中得到的
  return planner_->Init(injector_, FLAGS_planner_config_path);  // public_road_planner_config.pb.txt
}
/// @brief 
/// @param sequence_num 标识这一帧的序列号
/// @param planning_start_point 表示路径规划开始的点
/// @param vehicle_state 当前车辆的状态，包括位置、速度、加速度等信息
/// @return 
Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const VehicleState& vehicle_state) {
  // planning_base.h: std::unique_ptr<Frame> frame_;    ??????    
  // reset 方法确保 frame_ 的指针是有效的   
  // 创建新的 Frame 对象，封装当前帧所有上下文（车辆状态、参考线、障碍物等）                       
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state, reference_line_provider_.get()));
  
  if (frame_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }

  // reference_lines 用来存储参考线
  std::list<ReferenceLine> reference_lines;
  // segments 用来存储路段信息
  std::list<hdmap::RouteSegments> segments;
  // 由routing给出的route_segments信息生成reference_lines
  //  会填充这两个列表，获取当前的参考线和路段
  reference_line_provider_->GetReferenceLines(&reference_lines, &segments);
  // 验证参考线和路段的数量一致
  DCHECK_EQ(reference_lines.size(), segments.size());
  
  // 静态方法，用于计算车辆基于当前速度的前向可视距离
  auto forward_limit = planning::PncMapBase::LookForwardDistance(
      vehicle_state.linear_velocity());
  
  // 遍历所有参考线，调用 ref_line.Segment 方法进行收缩。收缩操作是根据车辆当前位置和前向/后向限制来进行的
  // planning::FLAGS_look_backward_distance：后向可视距离
  // forward_limit：前向可视距离
  for (auto& ref_line : reference_lines) {
    if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()),
                          planning::FLAGS_look_backward_distance,  // 50
                          forward_limit)) {
      const std::string msg = "Fail to shrink reference line.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    PrintCurves ref_print_curve;
    for (const auto& p : ref_line.reference_points()) {
      ref_print_curve.AddPoint("ref_line", p.x(), p.y());
    }
    ref_print_curve.PrintToLog();
  }
  // 遍历所有路段并对每个路段调用 seg.Shrink 方法进行收缩，确保它们在车辆当前状态下是有效的。如果收缩失败，则返回错误
  for (auto& seg : segments) {
    if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
                    planning::FLAGS_look_backward_distance, forward_limit)) {
      const std::string msg = "Fail to shrink routing segments.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    PrintCurves segment_print_curve;
    for (auto& lane_segment : seg) {
      for (const auto& pt : lane_segment.lane->points()) {
        segment_print_curve.AddPoint("segment_line", pt.x(), pt.y());
      }
    }
    segment_print_curve.PrintToLog();
  }

 // 初始化帧
 // injector_->vehicle_state()：注入的车辆状态
 // reference_lines：参考线列表
 // segments：路段列表
 // reference_line_provider_->FutureRouteWaypoints()：未来的路线点
 // injector_->ego_info()：车辆的自我信息
  auto status = frame_->Init(
      injector_->vehicle_state(), reference_lines, segments,
      reference_line_provider_->FutureRouteWaypoints(), injector_->ego_info());
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

// TODO(all): fix this! this will cause unexpected behavior from controller
void OnLanePlanning::GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb) {
  // 确保在生成新轨迹点之前，轨迹数组为空，以避免附加到现有的轨迹点数据上
  ptr_trajectory_pb->clear_trajectory_point();

  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  // apollo/modules/planning/planning_base/gflags/planning_gflags.cc
  // 轨迹时间范围（如 3 秒），表示这个轨迹要持续多久
  const double max_t = FLAGS_fallback_total_time;  // 3
  // 轨迹时间步长（如 0.1 秒），表示每个轨迹点的间隔
  const double unit_t = FLAGS_fallback_time_unit;  // 0.1

  TrajectoryPoint tp;
  auto* path_point = tp.mutable_path_point(); // 返回的是一个指向 path_point 的指针，允许我们修改轨迹点的位置信息
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_theta(vehicle_state.heading());
  path_point->set_s(0.0); // 轨迹点的初始位置
  tp.set_v(0.0);
  tp.set_a(0.0);
  // 未来3s内，每个0.1s取一个轨迹点，这些点的信息和tp一样
  for (double t = 0.0; t < max_t; t += unit_t) {
    tp.set_relative_time(t);
    auto next_point = ptr_trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
}
/*
private：只有父类自己可以访问，子类不能访问。
protected：父类自己可以访问，子类也可以访问，但外部无法访问。
public：所有地方都可以访问
*/
// 这部分主要做以下工作：
/*
1.更新传感器数据
2.检查车辆状态
3.更新参考线
4.轨迹拼接
5.生成规划轨迹
6.轨迹平滑 & 输出
*/
void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb) {
    // 1.0                           
  // when rerouting, reference line might not be updated. In this case, planning
  // module maintains not-ready until be restarted.
  local_view_ = local_view;
   // 获取路径规划起始时间戳 s
   // 受 ROS 2 use_sim_time 影响，如果 use_sim_time:=true，那么 Clock::NowInSeconds() 取的是 仿真时间 而不是系统时间
  const double start_timestamp = Clock::NowInSeconds(); // start_timestamp 用来计算轨迹的时间戳，确保和 ROS 2 其他模块同步
  // 路径规划算法耗时起始时间 s
  const double start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count(); // start_system_timestamp 用来计算 规划耗时（单位：秒），避免受 ROS 2 use_sim_time 影响

// 2.0
  // localization
  ADEBUG << "Get localization:"
         << local_view_.localization_estimate->DebugString();

  // chassis
  ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();
 // 更新车辆状态---> 进入Update函数查看
 // apollo/modules/common/vehicle_state/vehicle_state_provoder.cc
 // 融合定位（位置/姿态）和底盘（速度/档位）数据，生成统一的 VehicleState
  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);
 // 获取车辆状态
 // 获取并检查车辆状态的时间戳是否有效，确保 start_timestamp 大于等于车辆状态时间戳
  VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
  const double vehicle_state_timestamp = vehicle_state.timestamp();
  // 确保 start_timestamp 不会早于 vehicle_state_timestamp，并在不符合要求时打印出详细的错误信息
  // 检查时间合理性：规划开始时间不应早于车辆状态时间（否则说明时钟异常）
  DCHECK_GE(start_timestamp, vehicle_state_timestamp)
      << "start_timestamp is behind vehicle_state_timestamp by "
      << start_timestamp - vehicle_state_timestamp << " secs";
// 如果车辆状态无效，生成刹车轨迹
// status.ok()： Status status
  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    // 车辆状态无效 → 输出 NotReady + 停车轨迹
    const std::string msg =
        "Update VehicleStateProvider failed "
        "or the vehicle state is out dated.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    return;
  }
  // 检查系统时间是否滞后于GPS时间，并输出警告
  if (start_timestamp + 1e-6 < vehicle_state_timestamp) {
    common::monitor::MonitorLogBuffer monitor_logger_buffer(
        common::monitor::MonitorMessageItem::PLANNING);
    monitor_logger_buffer.ERROR("ego system time is behind GPS time");
  }
// 对齐时间戳 vehicle_state_timestamp为车辆状态时间戳
// 如果时间差小于某个阈值，对齐车辆状态的时间戳
// 大于它就完犊子了
// 时间对齐：若延迟小（如 < 100ms），用运动学模型预测车辆在 start_timestamp 时刻的状态，提升规划精度
  if (start_timestamp - vehicle_state_timestamp <
      FLAGS_message_latency_threshold) {  // 0.02s 消息延时阈值
    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
  }

// 参考线更新车辆状态
  // Update reference line provider and reset scenario if new routing
  reference_line_provider_->UpdateVehicleState(vehicle_state); // 车辆状态单纯的赋值
  // // 如果是两个不同的routing,则重新初始化
  // 当收到新的导航指令（如重新规划路线）时
  if (local_view_.planning_command->is_motion_command() &&
      util::IsDifferentRouting(last_command_, *local_view_.planning_command)) {
    // 如果发现新的规划命令，更新 last_command_ 为当前的规划命令
    last_command_ = *local_view_.planning_command;
    AINFO << "new_command:" << last_command_.DebugString();
    // 重置参考路线提供者，即清除当前的参考路线信息
    // 重置参考线提供器（ReferenceLineProvider）
    reference_line_provider_->Reset();
    // 通过 injector_ 调用 history() 获取历史数据的对象，并清除历史记录（Clear()）。这意味着在接收到新命令后，之前的历史数据（如轨迹、状态等）可能已经不再适用
    // 清空历史缓存（FrameHistory）和规划上下文（PlanningContext）
    injector_->history()->Clear();
// 通过 injector_ 获取规划上下文（planning_context()）并清除其中的规划状态（Clear()）。这表明与当前规划相关的状态信息也需要重置，可能是为了清除过时的规划状态
    injector_->planning_context()->mutable_planning_status()->Clear();
    reference_line_provider_->UpdatePlanningCommand(
        *(local_view_.planning_command));
// 重置规划器，并使用 frame_ 中的相关信息来重新初始化或更新规划器的状态
// 通知 Planner 重置状态（如取消正在进行的变道）
    planner_->Reset(frame_.get());
  }
  // Get end lane way point.
  // 获取当前车道的结束点
  // 更新终点车道信息（用于泊车或终点减速）
  reference_line_provider_->GetEndLaneWayPoint(local_view_.end_lane_way_point);

  // planning is triggered by prediction data, but we can still use an estimated
  // cycle time for stitching
  // 3.0
  // 进行轨迹拼接
// 在apollo规划算法中，在每一帧规划开始时会调用一个轨迹拼接函数，返回一段拼接轨迹点集
// 并告诉我们是否重新规划以及重规划的原因
// 将上一帧发布的轨迹末尾与当前车辆状态拼接，保证轨迹连续性
  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);  // 10 hz

// 使用轨迹拼接器计算拼接后的轨迹
// 轨迹拼接保留长度
  std::string replan_reason;
  std::vector<TrajectoryPoint> stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          *(local_view_.chassis), vehicle_state, start_timestamp,
          planning_cycle_time, FLAGS_trajectory_stitching_preserved_length,
          true, last_publishable_trajectory_.get(), &replan_reason);

// 4.0
// 更新ego信息，进入EgoInfo类查看私有成员变量释义，进入Update查看
// 规划起点设置、车辆状态、包围盒
// 更新自车信息（如前方净空距离），供后续决策使用
  injector_->ego_info()->Update(stitching_trajectory.back(), vehicle_state);
  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  AINFO << "Planning start frame sequence id = [" << frame_num << "]";


  // 初始化frame,重要信息，将拼接轨迹最后一个点作为路径规划起点
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);// frame类存储了一次规划循环中所需的所有数据   180
  // 计算前方障碍物距离、当前路段信息（如是否在交叉口）
  if (status.ok()) {
    injector_->ego_info()->CalculateFrontObstacleClearDistance(
        frame_->obstacles());
    // injector_->ego_info()->CalculateCurrentRouteInfo(
    //     reference_line_provider_.get());
  }

// 如果启用了调试记录，则记录调试信息
  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }
  // 记录 Frame 初始化耗时
  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  if (!status.ok()) {
    AERROR << status.ToString();
    // ESTOP：最高优先级，控制模块会立即刹车
    if (FLAGS_publish_estop) {
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      ADCTrajectory estop_trajectory;
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      ptr_trajectory_pb->CopyFrom(estop_trajectory);
    } else {
      // NotReady + Stop Trajectory：温和降级，缓慢停车
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      GenerateStopTrajectory(ptr_trajectory_pb);
    }
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    // 结束当前帧的规划并将帧添加到历史中
    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));
    return;
  }
  // 交通规则决策
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    auto traffic_status =
        traffic_decider_.Execute(frame_.get(), &ref_line_info);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
    }
  }
 //  开始正在的规划 planner 开始规划
  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);

  // print trajxy
  // 打印轨迹 XY 曲线
  PrintCurves trajectory_print_curve;
  for (const auto& p : ptr_trajectory_pb->trajectory_point()) {
    trajectory_print_curve.AddPoint("trajxy", p.path_point().x(),
                                    p.path_point().y());
  }
  trajectory_print_curve.PrintToLog();

  // print obstacle polygon
  // 打印障碍物多边形
  for (const auto& obstacle : frame_->obstacles()) {
    obstacle->PrintPolygonCurve();
  }
  // print ego box
  // 打印自车包围盒
  PrintBox print_box("ego_box");
  print_box.AddAdcBox(vehicle_state.x(), vehicle_state.y(),
                      vehicle_state.heading(), true);
  print_box.PrintToLog();

 // 计算总的规划时间并输出
 // 记录总耗时（通常要求 < 100ms）
  const auto end_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto time_diff_ms =
      (end_system_timestamp - start_system_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << ptr_trajectory_pb->latency_stats().DebugString();
  
  // 如果规划失败，输出错误信息并根据设置发布 estop 信号
  if (!status.ok()) {
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = ptr_trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }

// 若轨迹缝合失败（只保留一个点），说明是重新规划，记录原因
  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  if (ptr_trajectory_pb->is_replan()) {
    ptr_trajectory_pb->set_replan_reason(replan_reason);
  }

// 根据是否在开放空间轨迹上，填充并保存规划结果
  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    // 泊车模式：直接输出
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  } else {
    // 处理不同场景的规划结果
    // 主干道模式：可能进行轨迹平滑
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");
    // 填充header信息
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
    // 将当前帧加入历史缓存，供下一帧缝合使用
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    if (FLAGS_enable_planning_smoother) {
      planning_smoother_.Smooth(injector_->frame_history(), frame_.get(),
                                ptr_trajectory_pb);
    }
  }

  const auto end_planning_perf_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto plnning_perf_ms =
      (end_planning_perf_timestamp - start_system_timestamp) * 1000;
  AINFO << "Planning Perf: planning name [" << Name() << "], "
        << plnning_perf_ms << " ms.";
  AINFO << "Planning end frame sequence id = [" << frame_num << "]";

  injector_->frame_history()->Add(frame_num, std::move(frame_));
}

void OnLanePlanning::ExportReferenceLineDebug(planning_internal::Debug* debug) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  for (auto& reference_line_info : *frame_->mutable_reference_line_info()) {
    auto rl_debug = debug->mutable_planning_data()->add_reference_line();
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);

    // store kappa and dkappa for performance evaluation
    const auto& reference_points =
        reference_line_info.reference_line().reference_points();
    double kappa_rms = 0.0;
    double dkappa_rms = 0.0;
    double kappa_max_abs = std::numeric_limits<double>::lowest();
    double dkappa_max_abs = std::numeric_limits<double>::lowest();
    for (const auto& reference_point : reference_points) {
      double kappa_sq = reference_point.kappa() * reference_point.kappa();
      double dkappa_sq = reference_point.dkappa() * reference_point.dkappa();
      kappa_rms += kappa_sq;
      dkappa_rms += dkappa_sq;
      kappa_max_abs = kappa_max_abs < kappa_sq ? kappa_sq : kappa_max_abs;
      dkappa_max_abs = dkappa_max_abs < dkappa_sq ? dkappa_sq : dkappa_max_abs;
    }
    double reference_points_size = static_cast<double>(reference_points.size());
    kappa_rms /= reference_points_size;
    dkappa_rms /= reference_points_size;
    kappa_rms = std::sqrt(kappa_rms);
    dkappa_rms = std::sqrt(dkappa_rms);
    rl_debug->set_kappa_rms(kappa_rms);
    rl_debug->set_dkappa_rms(dkappa_rms);
    rl_debug->set_kappa_max_abs(kappa_max_abs);
    rl_debug->set_dkappa_max_abs(dkappa_max_abs);

    bool is_off_road = false;
    double minimum_boundary = std::numeric_limits<double>::infinity();

    const double adc_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    const auto& reference_line_path =
        reference_line_info.reference_line().GetMapPath();
    const auto sample_s = 0.1;
    const auto reference_line_length =
        reference_line_info.reference_line().Length();
    double average_offset = 0.0;
    double sample_count = 0.0;
    for (double s = 0.0; s < reference_line_length; s += sample_s) {
      double left_width = reference_line_path.GetLaneLeftWidth(s);
      double right_width = reference_line_path.GetLaneRightWidth(s);
      average_offset += 0.5 * std::abs(left_width - right_width);
      if (left_width < adc_half_width || right_width < adc_half_width) {
        is_off_road = true;
      }
      if (left_width < minimum_boundary) {
        minimum_boundary = left_width;
      }
      if (right_width < minimum_boundary) {
        minimum_boundary = right_width;
      }
      ++sample_count;
    }
    rl_debug->set_is_offroad(is_off_road);
    rl_debug->set_minimum_boundary(minimum_boundary);
    rl_debug->set_average_offset(average_offset / sample_count);
  }
}

Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  // 记录调试起点（Init Point）
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }

  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                               ptr_trajectory_pb);

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      injector_->ego_info()->front_clear_distance());

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    frame_->mutable_open_space_info()->sync_debug_instance();
    const auto& publishable_trajectory =
        frame_->open_space_info().publishable_trajectory_data().first;
    const auto& publishable_trajectory_gear =
        frame_->open_space_info().publishable_trajectory_data().second;
    // 直接使用 OpenSpace 生成的轨迹
    publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
    ptr_trajectory_pb->set_gear(publishable_trajectory_gear);
    ptr_trajectory_pb->set_trajectory_type(ADCTrajectory::OPEN_SPACE);
    // TODO(QiL): refine engage advice in open space trajectory optimizer.
    // 设置 Engage Advice（接管建议）
    auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

    // enable start auto from open_space planner.
    if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      engage_advice->set_reason(
          "Ready to engage when staring with OPEN_SPACE_PLANNER");
    } else {
      engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      engage_advice->set_reason("Keep engage while in parking");
    }
    // TODO(QiL): refine the export decision in open space info
    // 设置决策状态
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_parking()
        ->set_status(MainParking::IN_PARKING);
    // 记录 Debug & 导出图表
    if (FLAGS_enable_record_debug) {
      // ptr_debug->MergeFrom(frame_->open_space_info().debug_instance());
      frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
      ADEBUG << "Open space debug information added!";
      // call open space info load debug
      // TODO(Runxin): create a new flag to enable openspace chart
      ExportOpenSpaceChart(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
                           ptr_debug);
    }
  } else {
    // On-Lane 规划（正常行驶）
    const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
    const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();
    if (!best_ref_info) {
      const std::string msg = "planner failed to make a driving plan";
      AERROR << msg;
      // 参考线空,清空last_publishable_trajectory_
      if (last_publishable_trajectory_) {
        last_publishable_trajectory_->Clear();
      }
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    // Store current frame stitched path for possible speed fallback in next
    // frames
    //  构建 current_frame_planned_path（用于速度回退）
    DiscretizedPath current_frame_planned_path;
    // 先加入 stitching_trajectory 的所有 path_point
    for (const auto& trajectory_point : stitching_trajectory) {
      current_frame_planned_path.push_back(trajectory_point.path_point());
    }
    const auto& best_ref_path = best_ref_info->path_data().discretized_path();
    // 再追加 best_ref_info 规划的新路径（跳过第一个点，避免重复）
    std::copy(best_ref_path.begin() + 1, best_ref_path.end(),
              std::back_inserter(current_frame_planned_path));
    // Speed Fallback 机制: 如果下一帧速度规划失败，可沿用本帧的路径，只重新规划速度
    frame_->set_current_frame_planned_path(current_frame_planned_path);

    ptr_debug->MergeFrom(best_ref_info->debug());
    if (FLAGS_export_chart) {
      ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
    } else {
      ExportReferenceLineDebug(ptr_debug);
      // Export additional ST-chart for failed lane-change speed planning
      const auto* failed_ref_info = frame_->FindFailedReferenceLineInfo();
      if (failed_ref_info) {
        ExportFailedLaneChangeSTChart(failed_ref_info->debug(), ptr_debug);
      }
    }
    // 各子模块耗时（ST图生成、DP、QP等）
    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
        best_ref_info->latency_stats());
    // set right of way status
    // 是否拥有路权（如通过无保护左转）
    ptr_trajectory_pb->set_right_of_way_status(
        best_ref_info->GetRightOfWayStatus());
    // 当前车道 & 目标车道 ID
    for (const auto& id : best_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
    }

    for (const auto& id : target_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_target_lane_id()->CopyFrom(id);
    }

    ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());
    // Responsibility-Sensitive Safety 信息
    if (FLAGS_enable_rss_info) {
      *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
    }
    // 包含 yield、overtake、stop 等决策
    best_ref_info->ExportDecision(ptr_trajectory_pb->mutable_decision(),
                                  injector_->planning_context());

    // Add debug information.
    if (FLAGS_enable_record_debug) {
      auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
      reference_line->set_name("planning_reference_line");
      const auto& reference_points =
          best_ref_info->reference_line().reference_points();
      double s = 0.0;
      double prev_x = 0.0;
      double prev_y = 0.0;
      bool empty_path = true;
      for (const auto& reference_point : reference_points) {
        auto* path_point = reference_line->add_path_point();
        path_point->set_x(reference_point.x());
        path_point->set_y(reference_point.y());
        path_point->set_theta(reference_point.heading());
        path_point->set_kappa(reference_point.kappa());
        path_point->set_dkappa(reference_point.dkappa());
        if (empty_path) {
          path_point->set_s(0.0);
          empty_path = false;
        } else {
          double dx = reference_point.x() - prev_x;
          double dy = reference_point.y() - prev_y;
          s += std::hypot(dx, dy);
          path_point->set_s(s);
        }
        prev_x = reference_point.x();
        prev_y = reference_point.y();
      }
    }
    // (1) 创建新轨迹（仅含新规划部分）
    last_publishable_trajectory_.reset(new PublishableTrajectory(
        current_time_stamp, best_ref_info->trajectory()));
    PrintCurves debug_traj;
    for (size_t i = 0; i < last_publishable_trajectory_->size(); i++) {
      auto& traj_pt = last_publishable_trajectory_->at(i);
      debug_traj.AddPoint("traj_sv", traj_pt.path_point().s(), traj_pt.v());
      debug_traj.AddPoint("traj_sa", traj_pt.path_point().s(), traj_pt.a());
      debug_traj.AddPoint("traj_sk", traj_pt.path_point().s(),
                          traj_pt.path_point().kappa());
    }
    // debug_traj.PrintToLog();
    ADEBUG << "current_time_stamp: " << current_time_stamp;
    // (2) 前置拼接历史轨迹（stitching）
    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));
    // 最终发布的轨迹 = 历史缝合点 + 新规划轨迹，保证连续性
    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);
    // 设置 Engage Advice（接管建议）
    best_ref_info->ExportEngageAdvice(
        ptr_trajectory_pb->mutable_engage_advice(),
        injector_->planning_context());
  }

  return status;
}

bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  // TODO(All): check other config params
  return true;
}

void PopulateChartOptions(double x_min, double x_max, std::string x_label,
                          double y_min, double y_max, std::string y_label,
                          bool display, Chart* chart) {
  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

void AddSTGraph(const STGraphDebug& st_graph, Chart* chart) {
  if (st_graph.name() == "DP_ST_SPEED_OPTIMIZER") {
    chart->set_title("Speed Heuristic");
  } else {
    chart->set_title("Planning S-T Graph");
  }
  PopulateChartOptions(-2.0, 10.0, "t (second)", -10.0, 220.0, "s (meter)",
                       false, chart);

  for (const auto& boundary : st_graph.boundary()) {
    // from 'ST_BOUNDARY_TYPE_' to the end
    std::string type =
        StGraphBoundaryDebug_StBoundaryType_Name(boundary.type()).substr(17);

    auto* boundary_chart = chart->add_polygon();
    auto* properties = boundary_chart->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["cubicInterpolationMode"] = "monotone";
    (*properties)["showLine"] = "true";
    (*properties)["showText"] = "true";
    (*properties)["fill"] = "false";

    if (type == "DRIVABLE_REGION") {
      (*properties)["color"] = "\"rgba(0, 255, 0, 0.5)\"";
      // (*properties)["color"] = "\"rgba(255, 145, 0, 0.5)\"";
    } else {
      (*properties)["color"] = "\"rgba(255, 0, 0, 0.8)\"";
    }

    boundary_chart->set_label(boundary.name() + "_" + type);
    for (const auto& point : boundary.point()) {
      auto* point_debug = boundary_chart->add_point();
      point_debug->set_x(point.t());
      point_debug->set_y(point.s());
    }
  }

  auto* speed_profile = chart->add_line();
  auto* properties = speed_profile->mutable_properties();
  (*properties)["color"] = "\"rgba(255, 255, 255, 0.5)\"";
  for (const auto& point : st_graph.speed_profile()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.t());
    point_debug->set_y(point.s());
  }
}

void AddSLFrame(const SLFrameDebug& sl_frame, Chart* chart) {
  chart->set_title(sl_frame.name());
  PopulateChartOptions(0.0, 80.0, "s (meter)", -8.0, 8.0, "l (meter)", false,
                       chart);
  auto* sl_line = chart->add_line();
  sl_line->set_label("SL Path");
  for (const auto& sl_point : sl_frame.sl_path()) {
    auto* point_debug = sl_line->add_point();
    point_debug->set_x(sl_point.s());
    point_debug->set_x(sl_point.l());
  }
}

void AddSpeedPlan(
    const ::google::protobuf::RepeatedPtrField<SpeedPlan>& speed_plans,
    Chart* chart) {
  chart->set_title("Speed Plan");
  PopulateChartOptions(0.0, 80.0, "s (meter)", 0.0, 50.0, "v (m/s)", false,
                       chart);

  for (const auto& speed_plan : speed_plans) {
    auto* line = chart->add_line();
    line->set_label(speed_plan.name());
    for (const auto& point : speed_plan.speed_point()) {
      auto* point_debug = line->add_point();
      point_debug->set_x(point.s());
      point_debug->set_y(point.v());
    }

    // Set chartJS's dataset properties
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
    if (speed_plan.name() == "DpStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(27, 249, 105, 0.5)\"";
    } else if (speed_plan.name() == "QpSplineStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(54, 162, 235, 1)\"";
    }
  }
}

void OnLanePlanning::ExportFailedLaneChangeSTChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
}

void OnLanePlanning::ExportOnLaneChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
  for (const auto& sl_frame : src_data.sl_frame()) {
    AddSLFrame(sl_frame, dst_data->add_chart());
  }
  AddSpeedPlan(src_data.speed_plan(), dst_data->add_chart());
}

void OnLanePlanning::ExportOpenSpaceChart(
    const planning_internal::Debug& debug_info,
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug_chart) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_record_debug) {
    AddOpenSpaceOptimizerResult(debug_info, debug_chart);
    AddPartitionedTrajectory(debug_info, debug_chart);
    AddStitchSpeedProfile(debug_chart);
    AddPublishedSpeed(trajectory_pb, debug_chart);
    AddPublishedAcceleration(trajectory_pb, debug_chart);
    // AddFallbackTrajectory(debug_info, debug_chart);
  }
}

void OnLanePlanning::AddOpenSpaceOptimizerResult(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  auto open_space_debug = debug_info.planning_data().open_space();

  chart->set_title("Open Space Trajectory Optimizer Visualization");
  PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                       open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                       open_space_debug.xy_boundary(2) - 1.0,
                       open_space_debug.xy_boundary(3) + 1.0, "y (meter)", true,
                       chart);

  chart->mutable_options()->set_sync_xy_window_size(true);
  chart->mutable_options()->set_aspect_ratio(0.9);
  int obstacle_index = 1;
  for (const auto& obstacle : open_space_debug.obstacles()) {
    auto* obstacle_outline = chart->add_line();
    obstacle_outline->set_label(absl::StrCat("Bdr", obstacle_index));
    obstacle_index += 1;
    for (int vertice_index = 0;
         vertice_index < obstacle.vertices_x_coords_size(); vertice_index++) {
      auto* point_debug = obstacle_outline->add_point();
      point_debug->set_x(obstacle.vertices_x_coords(vertice_index));
      point_debug->set_y(obstacle.vertices_y_coords(vertice_index));
    }
    // Set chartJS's dataset properties
    auto* obstacle_properties = obstacle_outline->mutable_properties();
    (*obstacle_properties)["borderWidth"] = "2";
    (*obstacle_properties)["pointRadius"] = "0";
    (*obstacle_properties)["lineTension"] = "0";
    (*obstacle_properties)["fill"] = "false";
    (*obstacle_properties)["showLine"] = "true";
  }

  auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* smoothed_line = chart->add_line();
  smoothed_line->set_label("Smooth");
  // size_t adc_label = 0;
  for (int i = 0; i < smoothed_trajectory.vehicle_motion_point_size() / 2;
       i++) {
    auto& point = smoothed_trajectory.vehicle_motion_point(i);
    const auto x = point.trajectory_point().path_point().x();
    const auto y = point.trajectory_point().path_point().y();
    // const auto heading = point.trajectory_point().path_point().theta();
    /*
        // Draw vehicle shape along the trajectory
        auto* adc_shape = chart->add_car();
        adc_shape->set_x(x);
        adc_shape->set_y(y);
        adc_shape->set_heading(heading);
        adc_shape->set_color("rgba(54, 162, 235, 1)");
        adc_shape->set_label(std::to_string(adc_label));
        adc_shape->set_hide_label_in_legend(true);
        ++adc_label;
    */
    // Draw vehicle trajectory points
    auto* point_debug = smoothed_line->add_point();
    point_debug->set_x(x);
    point_debug->set_y(y);
  }

  // Set chartJS's dataset properties
  auto* smoothed_properties = smoothed_line->mutable_properties();
  (*smoothed_properties)["borderWidth"] = "2";
  (*smoothed_properties)["pointRadius"] = "0";
  (*smoothed_properties)["lineTension"] = "0";
  (*smoothed_properties)["fill"] = "false";
  (*smoothed_properties)["showLine"] = "true";

  auto warm_start_trajectory = open_space_debug.warm_start_trajectory();
  auto* warm_start_line = chart->add_line();
  warm_start_line->set_label("WarmStart");
  for (int i = 0; i < warm_start_trajectory.vehicle_motion_point_size() / 2;
       i++) {
    auto* point_debug = warm_start_line->add_point();
    auto& point = warm_start_trajectory.vehicle_motion_point(i);
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }
  // Set chartJS's dataset properties
  auto* warm_start_properties = warm_start_line->mutable_properties();
  (*warm_start_properties)["borderWidth"] = "2";
  (*warm_start_properties)["pointRadius"] = "0";
  (*warm_start_properties)["lineTension"] = "0";
  (*warm_start_properties)["fill"] = "false";
  (*warm_start_properties)["showLine"] = "true";
}

void OnLanePlanning::AddPartitionedTrajectory(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.chosen_trajectory().trajectory();
  if (chosen_trajectories.empty() ||
      chosen_trajectories[0].trajectory_point().empty()) {
    return;
  }

  const auto& vehicle_state = frame_->vehicle_state();
  auto chart = debug_chart->mutable_planning_data()->add_chart();
  auto chart_kappa = debug_chart->mutable_planning_data()->add_chart();
  auto chart_theta = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Partitioned Trajectory");
  chart_kappa->set_title("total kappa");
  chart_theta->set_title("total theta");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(0.9);

  // Draw vehicle state
  auto* adc_shape = chart->add_car();
  adc_shape->set_x(vehicle_state.x());
  adc_shape->set_y(vehicle_state.y());
  adc_shape->set_heading(vehicle_state.heading());
  adc_shape->set_label("ADV");
  adc_shape->set_color("rgba(54, 162, 235, 1)");

  // Draw the chosen trajectories
  const auto& chosen_trajectory = chosen_trajectories[0];
  auto* chosen_line = chart->add_line();
  chosen_line->set_label("Chosen");
  for (const auto& point : chosen_trajectory.trajectory_point()) {
    auto* point_debug = chosen_line->add_point();
    point_debug->set_x(point.path_point().x());
    point_debug->set_y(point.path_point().y());
  }
  auto* chosen_properties = chosen_line->mutable_properties();
  (*chosen_properties)["borderWidth"] = "2";
  (*chosen_properties)["pointRadius"] = "0";
  (*chosen_properties)["lineTension"] = "0";
  (*chosen_properties)["fill"] = "false";
  (*chosen_properties)["showLine"] = "true";
  auto* theta_line = chart_theta->add_line();
  auto* kappa_line = chart_kappa->add_line();
  // Draw partitioned trajectories
  size_t partitioned_trajectory_label = 0;
  for (const auto& partitioned_trajectory :
       open_space_debug.partitioned_trajectories().trajectory()) {
    auto* partition_line = chart->add_line();
    partition_line->set_label(
        absl::StrCat("Partitioned ", partitioned_trajectory_label));
    ++partitioned_trajectory_label;
    for (const auto& point : partitioned_trajectory.trajectory_point()) {
      auto* point_debug = partition_line->add_point();
      auto* point_theta = theta_line->add_point();
      auto* point_kappa = kappa_line->add_point();
      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
      point_theta->set_x(point.relative_time());
      point_kappa->set_x(point.relative_time());
      point_theta->set_y(point.path_point().theta());
      point_kappa->set_y(point.path_point().kappa());
    }

    auto* partition_properties = partition_line->mutable_properties();
    (*partition_properties)["borderWidth"] = "2";
    (*partition_properties)["pointRadius"] = "0";
    (*partition_properties)["lineTension"] = "0";
    (*partition_properties)["fill"] = "false";
    (*partition_properties)["showLine"] = "true";
    SetChartminmax(chart_kappa, "time", "total kappa");
    SetChartminmax(chart_theta, "time", "total theta");
  }

  // Draw trajectory stitching point (line with only one point)
  // auto* stitching_line = chart->add_line();
  // stitching_line->set_label("TrajectoryStitchingPoint");
  // auto* trajectory_stitching_point = stitching_line->add_point();
  // trajectory_stitching_point->set_x(
  //     open_space_debug.trajectory_stitching_point().path_point().x());
  // trajectory_stitching_point->set_y(
  //     open_space_debug.trajectory_stitching_point().path_point().y());
  // // Set chartJS's dataset properties
  // auto* stitching_properties = stitching_line->mutable_properties();
  // (*stitching_properties)["borderWidth"] = "3";
  // (*stitching_properties)["pointRadius"] = "5";
  // (*stitching_properties)["lineTension"] = "0";
  // (*stitching_properties)["fill"] = "true";
  // (*stitching_properties)["showLine"] = "true";

  // Draw fallback trajectory compared with the partitioned and potential
  // collision_point (line with only one point)
  // if (open_space_debug.is_fallback_trajectory()) {
  //   auto* collision_line = chart->add_line();
  //   collision_line->set_label("FutureCollisionPoint");
  //   auto* future_collision_point = collision_line->add_point();
  //   future_collision_point->set_x(
  //       open_space_debug.future_collision_point().path_point().x());
  //   future_collision_point->set_y(
  //       open_space_debug.future_collision_point().path_point().y());
  //   // Set chartJS's dataset properties
  //   auto* collision_properties = collision_line->mutable_properties();
  //   (*collision_properties)["borderWidth"] = "3";
  //   (*collision_properties)["pointRadius"] = "8";
  //   (*collision_properties)["lineTension"] = "0";
  //   (*collision_properties)["fill"] = "true";
  // (*stitching_properties)["showLine"] = "true";
  // (*stitching_properties)["pointStyle"] = "cross";

  // const auto& fallback_trajectories =
  //     open_space_debug.fallback_trajectory().trajectory();
  // if (fallback_trajectories.empty() ||
  //     fallback_trajectories[0].trajectory_point().empty()) {
  //   return;
  // }
  // const auto& fallback_trajectory = fallback_trajectories[0];
  // // has to define chart boundary first
  // auto* fallback_line = chart->add_line();
  // fallback_line->set_label("Fallback");
  // for (const auto& point : fallback_trajectory.trajectory_point()) {
  //   auto* point_debug = fallback_line->add_point();
  //   point_debug->set_x(point.path_point().x());
  //   point_debug->set_y(point.path_point().y());
  // }
  // // Set chartJS's dataset properties
  // auto* fallback_properties = fallback_line->mutable_properties();
  // (*fallback_properties)["borderWidth"] = "3";
  // (*fallback_properties)["pointRadius"] = "2";
  // (*fallback_properties)["lineTension"] = "0";
  // (*fallback_properties)["fill"] = "false";
  // (*fallback_properties)["showLine"] = "true";
  // }
}

void OnLanePlanning::AddStitchSpeedProfile(
    planning_internal::Debug* debug_chart) {
  if (!injector_->frame_history()->Latest()) {
    AINFO << "Planning frame is empty!";
    return;
  }

  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Speed Plan Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());
  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  const auto& last_trajectory =
      injector_->frame_history()->Latest()->current_frame_planned_trajectory();
  for (const auto& point : last_trajectory.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       last_trajectory.header().timestamp_sec());
    point_debug->set_y(point.v());
    if (point_debug->x() > xmax) xmax = point_debug->x();
    if (point_debug->x() < xmin) xmin = point_debug->x();
    if (point_debug->y() > ymax) ymax = point_debug->y();
    if (point_debug->y() < ymin) ymin = point_debug->y();
  }
  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("speed (m/s)");
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";
}

void OnLanePlanning::AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                                       planning_internal::Debug* debug_chart) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }

  auto chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());
  // auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE) {
      point_debug->set_y(point.v());
    }
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE) {
      point_debug->set_y(-point.v());
    }
    if (point_debug->x() > xmax) xmax = point_debug->x();
    if (point_debug->x() < xmin) xmin = point_debug->x();
    if (point_debug->y() > ymax) ymax = point_debug->y();
    if (point_debug->y() < ymin) ymin = point_debug->y();
  }
  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("speed (m/s)");
  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

/// @brief 对齐车辆状态的时间戳，即根据时间差 (curr_timestamp - vehicle_state.timestamp()) 
// 预测 curr_timestamp 时刻的车辆位置 (x, y)，然后返回更新后的 VehicleState
/// @param vehicle_state 当前的车辆状态
/// @param curr_timestamp 目标时间戳，通常是规划器希望对齐的轨迹起点时间
/// @return 
VehicleState OnLanePlanning::AlignTimeStamp(const VehicleState& vehicle_state,
                                            const double curr_timestamp) const {
  // TODO(Jinyun): use the same method in trajectory stitching
  //               for forward prediction
  // 预测 time_delta 秒后 车辆的位置
  auto future_xy = injector_->vehicle_state()->EstimateFuturePosition(
      curr_timestamp - vehicle_state.timestamp());

  VehicleState aligned_vehicle_state = vehicle_state;
  aligned_vehicle_state.set_x(future_xy.x());
  aligned_vehicle_state.set_y(future_xy.y());
  aligned_vehicle_state.set_timestamp(curr_timestamp);
  return aligned_vehicle_state;
}

void OnLanePlanning::AddPublishedAcceleration(
    const ADCTrajectory& trajectory_pb, planning_internal::Debug* debug) {
  // if open space info provider success run
  if (!frame_->open_space_info().open_space_provider_success()) {
    return;
  }
  double xmin(std::numeric_limits<double>::max()),
      xmax(std::numeric_limits<double>::lowest()),
      ymin(std::numeric_limits<double>::max()),
      ymax(std::numeric_limits<double>::lowest());
  auto chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();
  // options->mutable_x()->set_mid_value(Clock::NowInSeconds());

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_DRIVE)
      point_debug->set_y(point.a());
    if (trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE)
      point_debug->set_y(-point.a());
    if (point_debug->x() > xmax) xmax = point_debug->x();
    if (point_debug->x() < xmin) xmin = point_debug->x();
    if (point_debug->y() > ymax) ymax = point_debug->y();
    if (point_debug->y() < ymin) ymin = point_debug->y();
  }
  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("acceleration (m/s)");
  // Set chartJS's dataset properties
  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");

  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);

  // Set chartJS's dataset properties
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

}  // namespace planning
}  // namespace apollo
