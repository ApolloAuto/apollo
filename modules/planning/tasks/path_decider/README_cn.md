planning-task-path-decider
==============

## 简介
`PathDecider`决策生成借道路路径。

### MakeObjectDecision
调用MakeStaticObstacleDecision进行障碍物判决，之后判断是否需要忽略后向障碍物，如果配置文件中的后向障碍物忽略参数为是，则忽略后向障碍物。

### MakeStaticObstacleDecision
该方法用于静态障碍物判断
首先，计算出frenet坐标系下的path用于后续使用，并计算了车辆半宽half_width和横向半径

然后，该方法遍历了所有障碍物。
1)对于非静态和虚拟障碍物，该方法直接跳过；
2)对于经过纵向判决并且决策结果为ingnore，以及经过横向判决且横向决策结果为ignore的障碍物，该方法也直接跳过；
3)对于判决结果为stop的障碍物，该方法直接跳过；
4)对于id为 blocking_obstacle_id的障碍物，该方法为障碍物增加stop的判决;
5)对于boundary_type 为 KEEP_CLEAR 的障碍物，该方法直接跳过；
6)对于障碍物end_s 小于 frenet路径的front_s，或者 障碍物的start_s小于frenet路径的back_s，该方法增加not-in-s判决。（相当于ignore）；

其次，该方法判断静态障碍物可否nudge。
1）对于横向距离过远的障碍物，该方法ignore，加入not-in-l判决
2）对于横向距离过近，无法 nudge,根据情况加入stop判决
3）其他情况分别加入向左/向右判决

### GenerateObjectStopDecision
函数用于生成对障碍物的stop判决，并返回ObjectStop object_stop
首先使用MinRadiusStopDistance方法计算出对于障碍物的最短刹停距离stop_distance，然后为object_stop设置原因码和刹停距离（刹停距离设置为-1 * stop_distance）,其次计算刹停距离对应的参考station为stop_ref_s,最后根据stop_ref_s生成参考最短停止点。

### IgnoreBackwardObstacle
用于对obstacle生成忽略的结果。
首先生成参考线的sl边界的start_s,即sl边界对应的起始位置station；然后遍历所有障碍物，如果障碍物是静态障碍物或者虚拟障碍物，则跳过不做判断；其次判断障碍物在st图中的结束station，即end_s，如果end_s小于start_s，则认为该障碍物为backward-obstacle,并对障碍物做出纵向的ignore-backward-obstacle判决结果


## 目录结构 
```shell
modules/planning/tasks/path_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── path_decider.cc
├── path_decider.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── path_decider.proto
└── README_cn.md

```

## 模块

### PathDeciderh插件
apollo::planning::PathDecider

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/path_decider/conf/default_conf.pb.txt` | apollo::planning::PathDeciderConfig | PathDecider 的配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式
##### 配置加载 PathDecider Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`PathDecider`插件的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 PathDecider
```
task {
  name: "PATH_DECIDER"
  type: "PathDecider"
}
```
##### 配置 PathDecider 参数
在`modules/planning/tasks/path_decider/conf/default_conf.pb.txt`中，对`PathDecider`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`PathDecider`插件的gflag参数进行配置
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```