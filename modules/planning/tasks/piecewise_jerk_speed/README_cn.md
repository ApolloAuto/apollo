planning-task-piecewise-jerk-speed
============

## 简介

`PiecewiseJerkSpeedOptimizer`任务基于二次规划在凸ST空间的速度规划算法,优化的速度规划结果保存在reference_line_info_的st_graph_data_中。

### 模块流程
1. 检查是否到达终点，如果到达终点不再进行速度规划
  ```c++
   if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }
  ```
  2. 构建速度规划任务
   ```c++
   PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,  init_s);
   ```
  3. 根据障碍物决策和ST占用空间构建s的约束
   ```c++
     for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;
    double s_lower_bound = 0.0;
    double s_upper_bound = total_length;
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      double s_lower = 0.0;
      double s_upper = 0.0;
      if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        continue;
      }
      switch (boundary->boundary_type()) {
        case STBoundary::BoundaryType::STOP:
        case STBoundary::BoundaryType::YIELD:
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;
        case STBoundary::BoundaryType::FOLLOW:
          // TODO(Hongyi): unify follow buffer on decision side
          s_upper_bound = std::fmin(s_upper_bound,
                                    s_upper - config_.follow_distance_buffer());
          break;
        case STBoundary::BoundaryType::OVERTAKE:
          s_lower_bound = std::fmax(s_lower_bound, s_lower);
          break;
        default:
          break;
      }
    }
    print_debug.AddPoint("st_bounds_lower", curr_t, s_lower_bound);
    print_debug.AddPoint("st_bounds_upper", curr_t, s_upper_bound);
    if (s_lower_bound > s_upper_bound) {
      const std::string msg =
          "s_lower_bound larger than s_upper_bound on STGraph";
      AERROR << msg;
      speed_data->clear();
      print_debug.PrintToLog();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }
   ```
  4.  优化获得最优解
   `piecewise_jerk_problem.Optimize()`
   5. 填充速度点满足轨迹时间长度
   `SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);`

## 目录结构

```shell

modules/planning/tasks/piecewise_jerk_speed/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── piecewise_jerk_speed_optimizer.cc
├── piecewise_jerk_speed_optimizer.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── piecewise_jerk_speed.proto
└── README_cn.md

```

## 模块

### PiecewiseJerkSpeedOptimizer插件

apollo::planning::PiecewiseJerkSpeedOptimizer

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/piecewise_jerk_speed/conf/default_conf.pb.txt` | apollo::planning::SpeedHeuristicOptimizerConfig | PathTimeHeuristicOptimizer的默认配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 模块参数说明
  
   算法参数配置定义于modules/planning/tasks/piecewise_jerk_speed/proto/piecewise_jerk_speed.proto
   
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

#### 使用方式

##### 配置加载 PiecewiseJerkSpeedOptimizer Task 插件

在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`PiecewiseJerkSpeedOptimizer`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`PiecewiseJerkSpeedOptimizer`。
```
task {
  name: "PIECEWISE_JERK_SPEED"
  type: "PiecewiseJerkSpeedOptimizer"
}
  ```