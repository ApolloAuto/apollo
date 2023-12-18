# planning-planner-lattice

## 介绍

`planning-planner-lattice` 是包含LatticePlanner的插件包，LatticePlanner适用于在高速公路等简单场景下的规划。

## 目录结构

```shell

modules/planning/planner/lattice
├── lattice
    ├── behavior                // 行为决策库
    ├── trajectory_generation   // 轨迹生成库
    ├── lattice_planner.h       // LatticePlanner头文件
    ├── lattice_planner.cc      // LatticePlanner源文件
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── plugins.xml             // 插件描述说明文件
    └── README_cn.md            // 说明文档
```