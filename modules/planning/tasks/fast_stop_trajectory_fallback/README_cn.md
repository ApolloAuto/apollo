planning-task-fast-stop-trajectory-fallback
==============

## 简介

planning-task-fast-stop-trajectory-fallback 插件中主要作为主要处理在stage场景下无fallback配置，作为默认的fallback_task任务进行执行；fallback_task主要是在stage场景下的task任务运行error的情况下，根据车辆的实时速度、加速度计算出车辆fallback场景轨迹线Speed_data信息，这样做保证了车辆行驶的安全性。

场景切入条件：
  1. 当前stage场景下的fallback_task为空的情况下，设置FastStopTrajectoryFallback为预设的fallback场景；
  2. 当前的task任务运行出现error；
  3. 参考线path信息中的speed_data为空

### GenerateFallbackSpeed

该函数主要作为fallback任务执行时的主函数来计算fallback场景下的速度信息，该函数的输入主要有两个`ego_info`和`stop_distance`，函数的输入中主要通过`injector_`中的`ego_info`来获取车辆上一帧轨迹线终点的信息（速度、加速度信息），`stop_distance`设置车辆停止距离；当车辆的加速度小于0时，且速度小于0时，设置位移、速度、加速度为0使得车辆立即停下；否则根据预设的停止距离采用分段的jerk优化策略规划车辆的Speed_data信息，当分段jerk优化求解失败时，采用固定的预设定的减速度进行Speed_data信息的规划；

### GenerateStopProfile

该函数主要在分段的jerk优化策略求解失败时、采用固定的预设减速度对Speed_data信息的规划；

## 文件组织结构及说明

```shell
modules/planning/tasks/fast_stop_trajectory_fallback/
├── BUILD                                                 // 规则构建文件
├── cyberfile.xml                                         // 插件包管理配置文件
├── fast_stop_trajectory_fallback.cc                      // 任务器实现文件
├── fast_stop_trajectory_fallback.h                       // 任务器实现文件
├── plugins.xml                                           // 插件配置文件
└── README_cn.md                                          // 说明文档
```


## 模块

### FastStopTrajectoryFallback插件

apollo::planning::FastStopTrajectoryFallback
  
#### 阶段

| 阶段名                                        | 类型                                                   | 描述                                                                            |
| -------------------------------------------- | ------------------------------------------------------| -------------------------------------------------------------------------------|
| `轨迹线终点的速度v和加速度a均小于0`                         | `apollo::common::SpeedPoint`             | 主车轨迹线终点的速度和加速度均小于0，车辆已经完全停止，返回speed_data信息均为0  |
| `轨迹线终点的速度或者加速度a大于0`                         | `apollo::common::SpeedPoint`             | 主车轨迹线终点速度或者加速度大于0，车辆未完全停靠，分段计算SpeedPoint  |

#### 使用方式

FastStopTrajectoryFallback 场景在代码中作为default的fallback_task,如果stage场景中未配置fallback_task，task运行失败时，默认进入FastStopTrajectoryFallback任务；
