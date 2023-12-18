planning-task-open-space-trajectory-partition
==============

## 简介
`OpenSpaceTrajectoryPartition`首先对开放空间生成轨迹进行线性插值,而后根据路径前进或后退，将轨迹分割为不同段。最后找到当前轨迹上与自车最接近的点，作为轨迹起点，规划相应轨迹。

### InterpolateTrajectory
对生成轨迹`stitched_trajectory_result`通过间隔时间进行线性插值，而后将插值后的轨迹存储在`interpolated_trajectory`中。

### PartitionTrajectory
该函数遍历所有轨迹点，根据当前轨迹点与下一轨迹点的朝向，判断车辆是否换挡，如果换挡则以当前轨迹点对轨迹进行分割，从而使每段轨迹的车辆朝向相同。

### AdjustRelativeTimeAndS
该函数以车辆当前跟踪轨迹点为起点，对所有轨迹点进行相对时间与纵向距离调整，使当前跟踪轨迹点处的相对时间与纵向距离为0。

### InsertGearShiftTrajectory
当车辆走到轨迹末端，要走下一段轨迹时，插入停车轨迹，使车辆能够原地调整车轮转角。


## 目录结构 
```shell
modules/planning/tasks/open_space_trajectory_partition/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── open_space_trajectory_partition.cc
├── open_space_trajectory_partition.h
├── open_space_trajectory_partition_test.cc
├── plugins.xml
├── proto
│   ├── BUILD
│   └── open_space_trajectory_partition.proto
└── README_cn.md
```

## 模块输入输出与配置

### planning-task-open-space-trajectory-partition插件

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/open_space_trajectory_partition/conf/default_conf.pb.txt` | apollo::planning::OpenSpaceTrajectoryPartitionConfig | OpenSpaceTrajectoryPartition 的配置文件 |

#### 使用方式
##### 配置加载 OpenSpaceTrajectoryPartition Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`OpenSpaceTrajectoryPartition`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`OpenSpaceTrajectoryPartition`。
```
task {
  name: "OPEN_SPACE_TRAJECTORY_PARTITION"
  type: "OpenSpaceTrajectoryPartition"
}
```
##### 配置 OpenSpaceTrajectoryPartition 参数
在`modules/planning/tasks/open_space_trajectory_partition/conf/default_conf.pb.txt`中，对`OpenSpaceTrajectoryPartition`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`OpenSpaceTrajectoryPartition`插件的gflag参数进行配置。
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```