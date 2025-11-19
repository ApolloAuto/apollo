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
 * @file frame.cc
 **/
#include "modules/planning/planning_base/common/frame.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"

#include "modules/common_msgs/routing_msgs/routing.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/feature_output.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::cyber::Clock;
using apollo::prediction::PredictionObstacles;

PadMessage::DrivingAction Frame::pad_msg_driving_action_ = PadMessage::NONE;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_frame_history_num) {}

Frame::Frame(uint32_t sequence_num)
    : sequence_num_(sequence_num),
    //modules\common\monitor_log\proto\monitor_log.proto定义的message类MonitorMessageItem，监视的msg模块是planning
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {}

Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider)
    : sequence_num_(sequence_num),
      local_view_(local_view),
      planning_start_point_(planning_start_point),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider),
      monitor_logger_buffer_(common::monitor::MonitorMessageItem::PLANNING) {}

//Frame类带参构造函数，跟上面的构造函数区别就是没有最后一个参数参考线提供器
//但其实调用的还是上面的构造函数，只不过最后一个参数传个nullptr空指针
Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
    : Frame(sequence_num, local_view, planning_start_point, vehicle_state,
            nullptr) {}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const common::VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

//重新规划全局路径函数，输入参数是planning_context就是规划状态
//就是从自车位置出发再次routing,主要是更新routing_request的起始点为自车位置
//PlanningContext里包含routing_request
//这里找到自车最近的匹配车道上的最近点作为routing_request的起始点重新routing
//来设置PlanningContext里的rerouting对象
bool Frame::Rerouting(PlanningContext *planning_context) {
  //导航模式下不支持rerouting，默认不是导航模式
  if (FLAGS_use_navigation_mode) {
    AERROR << "Rerouting not supported in navigation mode";
    return false;
  }
  //如果local_view_里的routing结果里是空指针的话报错，说明没有先前的可用的routing
  if (local_view_.planning_command == nullptr) {
    AERROR << "No previous routing available";
    return false;
  }
  //如果hdmap_也就是空指针则报错返回
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return false;
  }
    //让指针rerouting指向规划状态planning_context里的
  //成员planning_status里的rerouting重新导航请求
  auto *rerouting =
      planning_context->mutable_planning_status()->mutable_rerouting();
    //设置rerouting重新导航请求里需要重新导航为true
  rerouting->set_need_rerouting(true);
  auto *lane_follow_command = rerouting->mutable_lane_follow_command();
  if (future_route_waypoints_.size() < 1) {
    AERROR << "Failed to find future waypoints";
    return false;
  }
  for (size_t i = 0; i < future_route_waypoints_.size() - 1; i++) {
    auto waypoint = lane_follow_command->add_way_point();
    waypoint->set_x(future_route_waypoints_[i].pose().x());
    waypoint->set_y(future_route_waypoints_[i].pose().y());
    waypoint->set_heading(future_route_waypoints_[i].heading());
  }
  auto *end_pose = lane_follow_command->mutable_end_pose();
  end_pose->set_x(future_route_waypoints_.back().pose().x());
  end_pose->set_y(future_route_waypoints_.back().pose().y());
  end_pose->set_heading(future_route_waypoints_.back().heading());

  monitor_logger_buffer_.INFO("Planning send Rerouting request");
  return true;
}

const std::list<ReferenceLineInfo> &Frame::reference_line_info() const {
  return reference_line_info_;
}

std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
  return &reference_line_info_;
}

void Frame::UpdateReferenceLinePriority(
    const std::map<std::string, uint32_t> &id_to_priority) {
  for (const auto &pair : id_to_priority) {
    const auto id = pair.first;
    const auto priority = pair.second;
    //找到这个id对应的参考线信息对象，遍历所有的参考线信息对象，find_if返回id相等的那
    //一个放入ref_line_info_itr，就是pair这对数据对应的参考线信息类对象
    auto ref_line_info_itr =
        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
                     [&id](const ReferenceLineInfo &ref_line_info) {
                       return ref_line_info.Lanes().Id() == id;
                     });
    if (ref_line_info_itr != reference_line_info_.end()) {
      ref_line_info_itr->SetPriority(priority);
    }
  }
}

bool Frame::CreateReferenceLineInfo(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments) {
  //首先清空数据成员参考线信息类列表reference_line_info_
  reference_line_info_.clear();
  if (reference_lines.empty()) {
    return true;
  }
  //获取参考线列表里的第一个对象ref_line_iter ，准备参与迭代
  auto ref_line_iter = reference_lines.begin();
  //获取导航路径段列表里的第一个对象segments_iter ，准备参与迭代
  auto segments_iter = segments.begin();
    //遍历每一条参考线列表里的每一条参考线
  //这个while循环的作用就是把参数里的参考线列表以及导航路径段列表用来构造参考线信息类对
  //象,挨个塞入Frame类数据成员reference_line_info_参考线信息类对象列表
  while (ref_line_iter != reference_lines.end()) {
        //若导航路径段因为到达终点附件而停止，
    //设置数据成员标志位is_near_destination_ 足够靠近终点为true
    if (segments_iter->StopForDestination()) {
      is_near_destination_ = true;
    }
        //将当前车辆状态，规划起始点，当前遍历的参考线对象，当前遍历的导航路径段对象一起塞
    //入类的数据成员reference_line_info_参考线信息类对象列表里
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    //迭代器的参考线对象指向列表里的下一个
    ++ref_line_iter;
    //迭代器导航路径段对象指向列表里的下一个
    ++segments_iter;
  }
  
  //若Frame类数据成员参考线信息类对象列表里元素个数为2
  if (reference_line_info_.size() == 2) {
    //首先将车辆当前位置点xy构建一个Vec2d二维向量就是点坐标
    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());
    //定义一个空的frenet系坐标点first_sl   (s,l)
    common::SLPoint first_sl;
    //首先获取参考线信息类对象列表里的第一条参考线，将车辆当前位置点坐标xy_point
    //投影到列表reference_line_info_里第一条参考线的frenet系下，投影的sl坐标
    //放入first_sl
    //自车XY坐标转列表第一条参考线下的frenet坐标，结果放入first_sl
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
                                                              &first_sl)) {
      return false;
    }
    common::SLPoint second_sl;
    //获取参考线信息类对象列表里的最后一条参考线，将车辆当前位置点坐标xy_point
    //投影到列表reference_line_info_里最后一条参考线的frenet系下，投影的sl坐标
    //放入second_sl
    //自车XY坐标转列表最后一条参考线下的frenet坐标，结果放入second_sl
    //这个是在参考线信息类对象列表里元素为2，所以参考线其实就2条？
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
                                                             &second_sl)) {
      return false;
    }
    //计算自车坐标投影到第一条参考线和投影最后一条参考线的frenet系下的l坐标的偏差
    //这个是在参考线信息类对象列表里元素为2，所以参考线其实就2条？
    const double offset = first_sl.l() - second_sl.l();
        //设置ReferenceLineInfo类对象列表reference_line_info_的数据成员
    //这个offset其实就是这两条参考线在横向上的相对偏移？
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }
  double target_speed = FLAGS_default_cruise_speed;
  if (local_view_.planning_command->has_target_speed()) {
    target_speed = local_view_.planning_command->target_speed();
  }
  //定义一个标志位 有有效的参考线？初始为false
  bool has_valid_reference_line = false;
   //遍历Frame类的数据成员reference_line_info_参考线信息类对象列表
  for (auto iter = reference_line_info_.begin();
       iter != reference_line_info_.end();) {
    //用障碍物列表去初始化当前遍历的参考线信息类对象
    if (!iter->Init(obstacles(), target_speed)) {
      reference_line_info_.erase(iter++);
    } else {
      //初始化成功就设置有有效的参考线标志位为true
      has_valid_reference_line = true;
      iter++;
    }
  }
  if (!has_valid_reference_line) {
    AINFO << "No valid reference line";
  }
  return true;
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  //如果参考线信息类对象列表为空，报错返回
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }
  //获取输入参数reference_line_info参考线信息里的参考线
  const auto &reference_line = reference_line_info->reference_line();
    //障碍物盒对应的中心点的frenet系s坐标=障碍物s坐标 + 0.1 / 2.0
  //virtual_stop_wall_length是gflags文件中定义的变量，为0.1
  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  //用参考线对象根据障碍物盒中心点s坐标获取其对应的参考点对象
  auto box_center = reference_line.GetReferencePoint(box_center_s);
  //获取这个障碍物盒中心点对应参考线上的参考点的Heading角
  double heading = reference_line.GetReferencePoint(obstacle_s).heading();
    //定义虚拟障碍物盒的宽度为4.0m，宽度对应着车道宽度方向
  //长度对应着纵向
  static constexpr double kStopWallWidth = 4.0;
    //构建虚拟障碍物停止墙边界盒，用障碍物盒的中心点对应的参考点对象，障碍物盒的中心点对
  //应的参考线上参考点对象heading角，边界盒长度0.1m，边界盒宽度4.0m
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                      kStopWallWidth};
    //然后通过构建的虚拟障碍物的边界盒对象stop_wall_box和障碍物id去创建虚拟障碍物
  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  if (!hdmap_) {
    AERROR << "Invalid HD Map.";
    return nullptr;
  }
  hdmap::LaneInfoConstPtr lane = nullptr;
  if (nullptr == reference_line_provider_) {
    lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  } else {
    lane = reference_line_provider_->GetLaneById(hdmap::MakeMapId(lane_id));
  }

  if (!lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }
   //终点的车道s坐标就是输入参数lane_s
  double dest_lane_s = std::max(0.0, lane_s);
  //获取终点的车道s坐标对应的ENU大地系坐标点
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
    //用参数lane_id对应的车道对象lane去获取终点dest_lane_s处的车道中心线左右宽度。
  //获取的结果放入引用变量lane_left_width，lane_right_width
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);
  //构建停止墙边界盒对象stop_wall_box
  //参数终点处x,y 终点处对应的车道heading,默认的虚拟墙长度0.1m，车道宽度(中心线做宽
  //度 + 中心线右宽度)
  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};
  //最后返回根据障碍物id和停止墙边界盒对象构建的虚拟障碍物对象
  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 */
const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  // start_xy
  // 将输入参数障碍物起始位置从SL坐标转化成XY坐标obstacle_start_xy
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  // 将输入参数障碍物终点位置从SL坐标转化成XY坐标obstacle_end_s
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  //获取障碍物起始点处对应的车道中心线左右宽度
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }
    //根据障碍物起始点，终点的xy坐标构成的二维线段类对象
  //以及车道在虚拟障碍物起始点处的总宽度去构建
  //构建障碍物的二维障碍物边界盒对象obstacle_box
  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};
    //根据障碍物id和障碍物对应的边界盒对象作为参数去调用构建虚拟障碍物的函数，
  //返回一个虚拟障碍物对象类
  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

//这个函数还是创建虚拟障碍物对象，返回的类型为Obstacle 类
//输入参数障碍物id，以及障碍物对应的二维边界盒box
const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  //在障碍物列表中寻找输入参数id对应的障碍物对象
  const auto *object = obstacles_.Find(id);
  //如果object不为空，说明找到了对应id的障碍物，障碍物还没创建就已经存在，报警告并返回
  if (object) {
    AWARN << "obstacle " << id << " already exist.";
    return object;
  }
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AERROR << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}
/// @brief 初始化一帧数据的函数，用来准备轨迹规划所需的信息
/// @param vehicle_state_provider 
/// @param reference_lines 
/// @param segments 
/// @param future_route_waypoints 
/// @param ego_info 
/// @return 
Status Frame::Init(
    const common::VehicleStateProvider *vehicle_state_provider,
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &segments,
    const std::vector<routing::LaneWaypoint> &future_route_waypoints,
    const EgoInfo *ego_info) {
  // TODO(QiL): refactor this to avoid redundant nullptr checks in scenarios.
  // 初始化基本数据，比如车辆状态、Ego 车辆信息等
  auto status = InitFrameData(vehicle_state_provider, ego_info);
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  // 将参考线和路径段匹配成参考线信息
  if (!CreateReferenceLineInfo(reference_lines, segments)) {
    const std::string msg = "Failed to init reference line info.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  future_route_waypoints_ = future_route_waypoints;
  return Status::OK();
}

Status Frame::InitForOpenSpace(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  return InitFrameData(vehicle_state_provider, ego_info);
}

Status Frame::InitFrameData(
    const common::VehicleStateProvider *vehicle_state_provider,
    const EgoInfo *ego_info) {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  vehicle_state_ = vehicle_state_provider->vehicle_state();
  if (!util::IsVehicleStateValid(vehicle_state_)) {
    AERROR << "Adc init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "Adc init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  if (FLAGS_align_prediction_time) {
    auto prediction = *(local_view_.prediction_obstacles);
    AlignPredictionTime(vehicle_state_.timestamp(), &prediction);
    local_view_.prediction_obstacles->CopyFrom(prediction);
  }
  for (auto &ptr :
       Obstacle::CreateObstacles(*local_view_.prediction_obstacles)) {
    AddObstacle(*ptr);
  }
  if (planning_start_point_.v() < 1e-3) {
    const auto *collision_obstacle = FindCollisionObstacle(ego_info);
    if (collision_obstacle != nullptr) {
      const std::string msg = absl::StrCat("Found collision with obstacle: ",
                                           collision_obstacle->Id());
      AERROR << msg;
      monitor_logger_buffer_.ERROR(msg);
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  ReadTrafficLights();

  ReadPadMsgDrivingAction();

  return Status::OK();
}

const Obstacle *Frame::FindCollisionObstacle(const EgoInfo *ego_info) const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }

  const auto &adc_polygon = Polygon2d(ego_info->ego_box());
  for (const auto &obstacle : obstacles_.Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    if (obstacle_polygon.HasOverlap(adc_polygon)) {
      return obstacle;
    }
  }
  return nullptr;
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

std::string Frame::DebugString() const {
  return absl::StrCat("Frame: ", sequence_num_);
}

void Frame::RecordInputDebug(planning_internal::Debug *debug) {
  if (!debug) {
    ADEBUG << "Skip record input into debug";
    return;
  }
  auto *planning_debug_data = debug->mutable_planning_data();
  auto *adc_position = planning_debug_data->mutable_adc_position();
  adc_position->CopyFrom(*local_view_.localization_estimate);

  auto debug_chassis = planning_debug_data->mutable_chassis();
  debug_chassis->CopyFrom(*local_view_.chassis);

  if (!FLAGS_use_navigation_mode) {
    auto debug_routing = planning_debug_data->mutable_routing();
    debug_routing->CopyFrom(
        local_view_.planning_command->lane_follow_command());
  }

  planning_debug_data->mutable_prediction_header()->CopyFrom(
      local_view_.prediction_obstacles->header());
  /*
  auto relative_map = AdapterManager::GetRelativeMap();
  if (!relative_map->Empty()) {
    planning_debug_data->mutable_relative_map()->mutable_header()->CopyFrom(
        relative_map->GetLatestObserved().header());
  }
  */
}

void Frame::AlignPredictionTime(const double planning_start_time,
                                PredictionObstacles *prediction_obstacles) {
  if (!prediction_obstacles || !prediction_obstacles->has_header() ||
      !prediction_obstacles->header().has_timestamp_sec()) {
    return;
  }
  double prediction_header_time =
      prediction_obstacles->header().timestamp_sec();
  for (auto &obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
    for (auto &trajectory : *obstacle.mutable_trajectory()) {
      for (auto &point : *trajectory.mutable_trajectory_point()) {
        point.set_relative_time(prediction_header_time + point.relative_time() -
                                planning_start_time);
      }
      if (!trajectory.trajectory_point().empty() &&
          trajectory.trajectory_point().begin()->relative_time() < 0) {
        auto it = trajectory.trajectory_point().begin();
        while (it != trajectory.trajectory_point().end() &&
               it->relative_time() < 0) {
          ++it;
        }
        trajectory.mutable_trajectory_point()->erase(
            trajectory.trajectory_point().begin(), it);
      }
    }
  }
}

Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

void Frame::ReadTrafficLights() {
  traffic_lights_.clear();

  const auto traffic_light_detection = local_view_.traffic_light;
  if (traffic_light_detection == nullptr) {
    return;
  }
  const double delay =
      traffic_light_detection->header().timestamp_sec() - Clock::NowInSeconds();
  if (delay > FLAGS_signal_expire_time_sec) {
    ADEBUG << "traffic signals msg is expired, delay = " << delay
           << " seconds.";
    return;
  }
  for (const auto &traffic_light : traffic_light_detection->traffic_light()) {
    traffic_lights_[traffic_light.id()] = &traffic_light;
  }
}

perception::TrafficLight Frame::GetSignal(
    const std::string &traffic_light_id) const {
  const auto *result =
      apollo::common::util::FindPtrOrNull(traffic_lights_, traffic_light_id);
  if (result == nullptr) {
    perception::TrafficLight traffic_light;
    traffic_light.set_id(traffic_light_id);
    traffic_light.set_color(perception::TrafficLight::UNKNOWN);
    traffic_light.set_confidence(0.0);
    traffic_light.set_tracking_time(0.0);
    return traffic_light;
  }
  return *result;
}

void Frame::ReadPadMsgDrivingAction() {
  if (local_view_.pad_msg) {
    if (local_view_.pad_msg->has_action()) {
      pad_msg_driving_action_ = local_view_.pad_msg->action();
    }
  }
}

void Frame::ResetPadMsgDrivingAction() {
  pad_msg_driving_action_ = PadMessage::NONE;
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::FindTargetReferenceLineInfo() {
  const ReferenceLineInfo *target_reference_line_info = nullptr;
  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
    target_reference_line_info = &reference_line_info;
  }
  return target_reference_line_info;
}

const ReferenceLineInfo *Frame::FindFailedReferenceLineInfo() {
  for (const auto &reference_line_info : reference_line_info_) {
    // Find the unsuccessful lane-change path
    if (!reference_line_info.IsDrivable() &&
        reference_line_info.IsChangeLanePath()) {
      return &reference_line_info;
    }
  }
  return nullptr;
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

}  // namespace planning
}  // namespace apollo
