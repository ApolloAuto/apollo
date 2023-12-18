planning-task-piecewise-jerk-speed-nonlinear
============

## 简介

`PiecewiseJerkSpeedNonlinearOptimizer`任务基于非线性规划在凸ST空间的速度规划算法
### 模块流程

1. 检查是否到达终点，如果到达终点不再进行速度规划
  ```c++
   if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }
  ```
  2. 构建优化变量边界
   ```c++
   const auto problem_setups_status =
      SetUpStatesAndBounds(path_data, *speed_data);
   ```
  3. 基于QP算法对动态规划的粗ST曲线进行平滑
   ```c++
     const auto qp_smooth_status =
      OptimizeByQP(speed_data, &distance, &velocity, &acceleration);
   ```
  4.  基于QP算法平滑路径曲率曲线
  ```c++
   const auto path_curvature_smooth_status = SmoothPathCurvature(path_data);
   ```
   5. 基于QP算法平滑限速曲线
   ```c++
   const auto speed_limit_smooth_status = SmoothSpeedLimit();
   ```
   6. 基于非线性规划优化ST曲线
    ```c++
      const auto nlp_smooth_status =
          OptimizeByNLP(&distance, &velocity, &acceleration);
    ```
  
  7. 填充速度点满足轨迹时长要求
  ```c++
  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  ```
  8. 记录结果用于PNC Monitor可视化
  ```c++
  RecordDebugInfo(*speed_data, st_graph_data->mutable_st_graph_debug());
  ```
  
## 目录结构

```shell

modules/planning/tasks/piecewise_jerk_speed_nonlinear/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── piecewise_jerk_speed_nonlinear_ipopt_interface.cc
├── piecewise_jerk_speed_nonlinear_ipopt_interface.h
├── piecewise_jerk_speed_nonlinear_optimizer.cc
├── piecewise_jerk_speed_nonlinear_optimizer.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── piecewise_jerk_speed_nonlinear.proto
└── README_cn.md

```

## 模块

### PiecewiseJerkSpeedNonlinearOptimizer插件

apollo::planning::PiecewiseJerkSpeedNonlinearOptimizer

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/piecewise_jerk_speed_nonlinear/conf/default_conf.pb.txt` | apollo::planning::PiecewiseJerkNonlinearSpeedOptimizerConfig | PiecewiseJerkNonlinearSpeedOptimizer的默认配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 模块参数说明
  
   算法参数配置定义于modules/planning/tasks/piecewise_jerk_speed_nonlinear/proto/piecewise_jerk_speed_nonlinear.proto
   
   | acc_weight             | 加速度权重   |
   | ---------------------- | ------------ |
   | jerk_weight            | 加加速度权重 |
   | kappa_penalty_weight   | 曲率惩罚权重 |
   | ref_s_weight           | 参考位置权重 |
   | ref_v_weight           | 参考速度权重 |
   | follow_distance_buffer | 跟车距离缓冲 |

#### 命令行参数/gflags
  `modules/planning/planning_base/common/planning_gflags.cc`中定义了用到的命令行参数，planning.conf中定义命令行参数值。

  | flag | 描述 |
  |  ---- | ---- |
  |  FLAGS_longitudinal_jerk_lower_bound | 速度规划加加速度下边界 |
  |  FLAGS_longitudinal_jerk_upper_bound | 速度规划加加速度上边界 |
  |  FLAGS_planning_upper_speed_limit | 最大速度 |
  |  FLAGS_follow_min_distance | 最小跟车距离 |
  |  FLAGS_follow_time_buffer | 最小跟车时距 |

#### 使用方式

##### 配置加载 PiecewiseJerkSpeedNonlinearOptimizer Task 插件

在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`PiecewiseJerkSpeedNonlinearOptimizer`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`PiecewiseJerkSpeedNonlinearOptimizer`。
```
task {
  name: "PIECEWISE_JERK_SPEED_NONLINEAR"
  type: "PiecewiseJerkSpeedNonlinearOptimizer"
}
  ```