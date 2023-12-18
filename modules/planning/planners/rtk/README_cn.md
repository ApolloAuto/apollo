# planning-planner-rtk

## 介绍

`planning-planner-rtk` 是包含RTKReplayPlanner的插件包，RTKReplayPlanner是基于录制的轨迹进行循迹的Planner，需要事先录制好设定的轨迹来规划行车路线。

## 目录结构

```shell

modules/planning/planner/rtk
├── rtk
    ├── testdata                    // 测试数据
    ├── rtk_replay_planner.h        // RTKReplayPlanner头文件
    ├── rtk_replay_planner.cc       // RTKReplayPlanner源文件
    ├── rtk_replay_planner_test.cc  // 单元测试文件
    ├── BUILD                       // 构建规则文件
    ├── cyberfile.xml               // 包管理配置文件
    ├── plugins.xml                 // 插件描述说明文件
    └── README_cn.md                // 说明文档
```