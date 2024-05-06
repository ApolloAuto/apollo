# planning-planner-navi

## 介绍

`planning-planner-navi` 是包含NaviPlanner的插件包，NaviPlanner是基于相对地图的规划器，主要用于高速公路场景。它主要考虑实际临时或者移动障碍物，并考虑速度和动力学约束的情况下，尽量按照规划路径进行轨迹规划。

## 目录结构

```shell

modules/planning/planner/navi
├── public_road
    ├── conf                        // 配置文件
    ├── decider                     // 决策器类
    ├── proto                       // 配置定位文件
    ├── navi_planner.h              // NaviPlanner头文件
    ├── navi_planner.cc             // NaviPlanner源文件
    ├── navi_planner_test.cc        // 单元测试文件
    ├── BUILD                       // 构建规则文件
    ├── cyberfile.xml               // 包管理配置文件
    ├── plugins.xml                 // 插件描述说明文件
    └── README_cn.md                // 说明文档
```
## 插件

### NaviPlanner

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/planners/navi/conf/planner_config.pb.txt`                 | `apollo::planning::PlannerNaviConfig`              | NaviPlanner的配置文件               |