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

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/planning_base/common/smoothers/smoother.h"
#include "modules/planning/planning_component/planning_base.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector) {}
// 虚析构函数，确保当通过基类指针删除派生类对象时，能够正确调用派生类的析构函数。
  virtual ~OnLanePlanning();

  /**
   * @brief Planning name.
   */
  // 返回规划模块的名称，覆盖基类中的同名函数，即"OnLanePlanning""
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  // 模块的初始化函数，接收配置并返回初始化状态
  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  // 主规划逻辑函数，周期性地被定时器触发，处理输入数据并生成路径
  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;

  // 执行路径规划的主要函数，接受时间戳和路径点进行规划
  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
 // 初始化规划帧，包括设置序列号、规划起点和车辆状态
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);
// 对齐车辆状态的时间戳，确保与当前时间一致
  common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                      const double curr_timestamp) const;
// 导出参考线的调试信息
  void ExportReferenceLineDebug(planning_internal::Debug* debug);
// 检查规划配置的有效性
  bool CheckPlanningConfig(const PlanningConfig& config);
// 生成停车轨迹
  void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb);
// 导出车道变换失败的调试图表
  void ExportFailedLaneChangeSTChart(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
// 导出在车道上行驶的调试图表
  void ExportOnLaneChart(const planning_internal::Debug& debug_info,
                         planning_internal::Debug* debug_chart);
// 导出开放空间的调试图表
  void ExportOpenSpaceChart(const planning_internal::Debug& debug_info,
                            const ADCTrajectory& trajectory_pb,
                            planning_internal::Debug* debug_chart);
// 向调试图表中添加开放空间优化结果
  void AddOpenSpaceOptimizerResult(const planning_internal::Debug& debug_info,
                                   planning_internal::Debug* debug_chart);
// 向调试图表中添加分区轨迹
  void AddPartitionedTrajectory(const planning_internal::Debug& debug_info,
                                planning_internal::Debug* debug_chart);
// 向调试图表中添加拼接速度曲线
  void AddStitchSpeedProfile(planning_internal::Debug* debug_chart);
// 向调试图表中添加发布的速度
  void AddPublishedSpeed(const ADCTrajectory& trajectory_pb,
                         planning_internal::Debug* debug_chart);
// 向调试图表中添加发布的加速度
  void AddPublishedAcceleration(const ADCTrajectory& trajectory_pb,
                                planning_internal::Debug* debug);
// 向调试图表中添加备用轨迹
  void AddFallbackTrajectory(const planning_internal::Debug& debug_info,
                             planning_internal::Debug* debug_chart);

 private:
  PlanningCommand last_command_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  Smoother planning_smoother_;
};

}  // namespace planning
}  // namespace apollo
