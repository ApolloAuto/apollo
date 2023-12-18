planning-task-st-bounds-decider
==============

## 简介
`ST BOUNDS DECIDER`st边界决策器。该模块共由4部分代码组成，分别为基础功能st_bounds_decider，和三个工具st_driving_limits、st_guide_line以及st_obstacle_processor
主要是对动态、最近的静态且阻塞当前引导路径的障碍物，进行st图构建。ignore对当前纵向规划没有影响的障碍物,并按照设定轨迹给出每一个障碍物boundary的最优决策，overtake超越，或者yield，在障碍物后跟随,最终决策出最优的Drivable_st_boundary;


1) st_bounds_decider
### Process
st bounds decider模块的主要逻辑函数。

1.1 初始化STBoundsDecider

1.2 调用 GenerateRegularSTBound方法，生成regular_st_bound和 regular_vt_bound

1.3 调用 SetSTDrivableBoundary方法，为st_drivable_boundary_设置t，s_lower,S_upper,并根据需要设置v_obs_lower和v_obs_upper

1.4 调用GetAllSTBoundaries方法，获取所有boundry信息，存入all_st_boundaries.

1.5 遍历所有st_boundary,记录debug信息，打印日志


### InitSTBoundsDecider
st_obstacles_processor_ 初始化 ，并调用MapObstaclesToSTBoundaries方法将障碍物绘入st图
设置部分速度限制，并初始化st_guide_line_和 st_driving_limits_,设定部分车辆动力学限制。

### GenerateFallbackSTBound
在所有时间为主车找到最安全（距离障碍物最大s）的决策方式

### GenerateRegularSTBound
首先生成st边界和vt边界，并为其填充极值，作为后续使用。

然后遍历所有st bound中的时刻，先根据车辆动力学限制设定一次 s_lower和s_upper,

依据障碍物st_boundary约束，调用GetBoundsFromDecisions更新可通行s_bounds以及对应的决策
对于同一障碍物的st_boundary可能存在多个决策，比如overtake或yield,

依据自车运动学约束，调用RemoveInvalidDecisions剔除不可达的障碍物

对avaliable_choices进行按规则迭代排序

找到优先级最高的可通行空间并赋值给obstacleDecision()

更新s_guide_line/st_driving_limits/v_limts

### RemoveInvalidDecisions
依据自车运动学约束，剔除不可达的障碍物decision

### RankDecisions
对avaliable_choices进行按规则进行排序。


### RecordSTGraphDebug
打印部分debug信息

2) st_obstacle_processor 工具类

### MapObstaclesToSTBoundaries
构建障碍物到路径上的st图
该函数包含如下部分

2.1 更新低路权路段。

通过循环的方式，将所有低路权路段连接为整段，更新在adc_low_road_right_segments_容器中。
例如，假设用l h代表低路权，高路权，多个path_pt_info中的数据为
1 l
2 l
3 l
4 l
5 h
6 h
7 h
8 l
9 l
10 l，
则更新之后adc_low_road_right_segments_中存储的内容为（1,4）（8,10）

设置非忽略障碍物的set non_ignore_obstacles

设置最近静态障碍物 closest_stop_obstacle，后续将会不断更新此最近静态障碍物

2.2 遍历所有障碍物，并进行如下操作：
2.2.1 
将障碍物绘入st图，
对于不在st图中的障碍物，直接忽略，其他障碍物，对上下边界点不做删减，保留原始数值
对于标记为is_caution_obstacle的障碍物，设置路权结束时间 road_right_ending_t，
对于不是低路权的障碍物删除上下边界
对于标记为keep clear的障碍物，加入candidate_clear_zones_列表,忽略后续操作。

2.2.2
对于静态障碍物，判断静态障碍物与上一个障碍物在st图上的边界，如果距离主车更近，则更新最近静态障碍物closest_stop_obstacle

对于动态障碍物，忽略后向动态障碍物，前向动态障碍物加入obs_id_to_st_boundary_列表中，并设置st_boundary,
并在non_ignore_obstacles加入障碍物id

2.3
存储最近的static障碍物boundary到obs_id_boundary_map容器中
如果static障碍物边界与keepclear区域发生重叠，则更新对应的id/boundary/obs_ptr

2.4 不在st图中的障碍物设置为ignore

2.5 每个障碍物存储对应的首尾/上下界s/id 信息

2.6 对edges进行排序

### GetLimitingSpeedInfo
首先检查 obs_id_to_decision_ 是否为空，如果为空，说明没有障碍物，因此返回 false。

初始化 s_min 和 s_max 为规划距离的起点和终点。

遍历 obs_id_to_decision_ 中的每一个障碍物。对于每一个障碍物：

根据时间 t 和障碍物边界信息，计算出障碍物在时间t的边界范围和边界斜率（上界和下界）。
如果障碍物的决策是让路或者停车，并且障碍物在时间t的最小距离小于等于s_max，则更新s_max 为障碍物的最小距离，
并将障碍物的下界斜率赋给 limiting_speed_info->second

如果障碍物的决策是超车，并且障碍物在时间 t 的最大距离大于等于 s_min，则更新 s_min 为障碍物的最大距离，并将障碍物的上界斜率赋给 limiting_speed_info->first。

最后检查 s_min 是否小于等于 s_max，如果是，则说明存在限制速度信息，函数返回 true，否则返回 false。

### GetSBoundsFromDecisions
先在st图中绘制t边界,然后删除对于在t之前消失的边界。

对于被超越的障碍物，如果它们的高路权结束时间小于或等于我们当前时间减去警告时间kOvertakenObsCautionTime，则从obs_id_to_decision_删除。

遍历obs_id_to_decision_向量，检查每个决策是否有超越，并且障碍物的最小t和最大t是否都小于或等于当前时间减去警告时间。
如果是，则将它们的ID添加到要删除的obs_id列表中，并在obs_id_to_decision_向量中删除它们。

根据现有的决策获取s范围。遍历obs_id_to_decision_向量，对于每个决策，获取其对应的ST边界，并获取该边界的s范围。
如果决策是让路或停车，则将s范围的最小值设置为s的最大值。如果决策是超越，则将s范围的最大值设置为s的最小值。

如果计算得到的s范围的最小值大于最大值，则返回false，表示无法得到有效的s范围。

对于新进入的ST边界，确定可能的新的ST边界。对于那些显然需要让路或停车的障碍物，直接为其做出决策。

对于决策模糊的情况，先去寻找对应的可通行上下边界,然后再进行决策，无法决策时会加入默认决策


### SetObstacleDecision 
(const std::string& obs_id, const ObjectDecisionType& obs_decision)
### SetObstacleDecision
(const std::vector<std::pair<std::string, ObjectDecisionType>>&
        obstacle_decisions)
上述两个方法最终实现都在第一个中，将obstacle信息设置入obs_id_to_decision_ 这个map中，并为每一个obsid对应的obstacle设置boundaryType,



### ComputeObstacleSTBoundary
计算障碍物的st_boundary

对于静态障碍物，即没有trajectory_point(),使用GetOverlappingS方法计算出障碍物在path上的重叠起始点和结束点，
并将这些信息更新在lower_points和upper_points中，is_caution_obstacle设置为true，caution结束时间就是planning_time_时间

对于动态障碍物，遍历每一个时间点，障碍物的OverlappingS，并更新结果到lowerpoints和upperpoints中，
然后根据先前配置的低路权路段更新iscautiontime和obs_caution_end_t

### GetOverlappingS
计算出障碍物在path上的重叠，先通过GetSBoundingPathPointIndex找到大致搜索范围，如果搜索范围过小，则返回false，表示障碍物与path没有重叠。
然后对于每个路径点，通过两次循环，一次从前向后，一次从后向前，分别确定障碍物在path上的重叠起始点和重叠结束点
起始点和距离存储在std::pair<double, double>* const overlapping_s中

### GetSBoundingPathPointIndex
这是一个递归调用的函数，通过反复调用IsPathPointAwayFromObstacle和GetSBoundingPathPointIndex，
二分查找到障碍物前后在path上的大致投影点位置，精度<=sthred

### IsPathPointAwayFromObstacle
通过向量计算，确定路径点是否远离障碍物所有角点,该函数中的corner_pt_s_dist是计算出的obs与主车行驶方向的法向的距离。
需要注意的是，该函数并非简单地比较这个距离的绝对值与sthred的大小来决定返回true或者false，
以is before为例，当corner_pt_s_dist为负时（即使绝对值很大），也会返回true，详情参考函数代码本身。

### IsADCOverlappingWithObstacle
检查主车路径点是否和障碍物边界框有重叠

### FindSGaps
寻找障碍物间的s间隙

### DetermineObstacleDecision
判断是否应该超车，否则返回默认值

### IsSWithinADCLowRoadRightSegment
判断一段s是否在低路权路段中。

3) st_driving_limits

### GetVehicleDynamicsLimits
计算从当前速度到t时刻，车辆使用最大加速度加速（max acc）能走过的最短距离，或者使用最大制动力制动（max dec），所走过的最长距离。
假设车辆加速度恒定，加减速过程均为理想状态

### UpdateBlockingInfo
根据动力学公式，调用GetVehicleDynamicsLimits计算出车辆在动力学下的上下限，然后根据传入的上下限更新车辆的上下限，主要包括v0 s0的最大最小值。

4) st_guide_line
### Init
根据输入的参考轨迹和期望速度生成guideline_speed_data_,该参考线速度数据储存了guideline上的速度信息

### GetGuideSFromT
根据guideline_speed_data_中的信息计算s，如果计算时刻小于TotalTime,则按照信息直接获取s，如果计算时刻大于totaltime，则按照v0推算s

### UpdateBlockingInfo
根据传入的t，更新s0 和t0


## 目录结构 
```shell
modules/planning/tasks/st_bounds_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── st_bounds_decider.cc
├── st_bounds_decider.h
├── st_driving_limits.cc
├── st_driving_limits.h
├── st_guide_line.cc
├── st_guide_line.h
├── st_obstacles_processor.cc
├── st_obstacles_processor.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── st_bounds_decider.proto
└── README_cn.md

```

## 模块

### StBoundsDecider插件
apollo::planning::StBoundsDecider

#### 配置文件&配置项
| 文件路径 | 类型/结构 | 说明 |
| ---- | ---- | ---- |
| `modules/planning/tasks/st_bounds_decider/conf/default_conf.pb.txt` | `apollo::planning::STBoundsDeciderConfig` | StBoundsDecider的默认配置文件 |
|`modules/common/data/vehicle_param.pb.txt`|`apollo::common::VehicleConfig`|车辆底盘配置文件|

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式
##### 配置加载 StBoundsDecider Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`StBoundsDecider`插件的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 StBoundsDecider
```
task {
  name: "ST_BOUNDS_DECIDER"
  type: "StBoundsDecider"
}
```
##### 配置 StBoundsDecider 参数
在`modules/planning/tasks/st_bounds_decider/conf/default_conf.pb.txt`中，对`StBoundsDecider`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`StBoundsDecider`插件的gflag参数进行配置
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```