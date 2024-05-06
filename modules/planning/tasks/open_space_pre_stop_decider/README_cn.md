planning-task-open-space-pre-stop-decider
==============

## 简介
`OpenSpacePreStopDecider`用于开放空间中执行泊车与靠边停车任务时，生成在公共道路上的停车点，车辆停在停车点后，会转入开放空间算法。

### CheckPullOverPreStop
函数计算靠边停车目标点在道路上的纵向投影点。

### CheckParkingSpotPreStop
计算目标停车位中心点在道路上的纵向投影点

### SetParkingSpotStopFence
该函数根据停车位的纵向投影点，计算并生成停止墙。

### SetPullOverStopFence
该函数根据靠边停车点的纵向投影点，计算并生成停止墙。

## 目录结构 
```shell
modules/planning/tasks/open_space_pre_stop_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── open_space_pre_stop_decider.cc
├── open_space_pre_stop_decider.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── open_space_pre_stop_decider.proto
└── README_cn.md
```

## 模块输入输出与配置

### planning-task-open-space-pre-stop-decider插件

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/open_space_pre_stop_decider/conf/default_conf.pb.txt` | apollo::planning::OpenSpacePreStopDeciderConfig | OpenSpacePreStopDecider 的配置文件 |

#### 使用方式
##### 配置加载 OpenSpacePreStopDecider Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`OpenSpacePreStopDecider`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`OpenSpacePreStopDecider`。
```
task {
  name: "OPEN_SPACE_PRE_STOP_DECIDER"
  type: "OpenSpacePreStopDecider"
}
```
##### 配置 OpenSpacePreStopDecider 参数
在`modules/planning/tasks/open_space_pre_stop_decider/conf/default_conf.pb.txt`中，对`OpenSpacePreStopDecider`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`OpenSpacePreStopDecider`插件的gflag参数进行配置。
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```