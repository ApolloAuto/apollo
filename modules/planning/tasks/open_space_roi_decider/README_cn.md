planning-task-open-space-roi-decider
==============

## 简介
`OpenSpaceRoiDecider`用于在开放空间算法中生成可行驶边界，根据规划场景的不同，如：泊车、靠边停车、驶入主路，以当前车辆位置为坐标原点，生成不同的可行驶边界与目标点。

### GetParkingSpot
函数首先将停车位的polygon转换为顺时针，而后计算了目标车位的位姿，根据与道路的夹角，判断车位是平行车位还是垂直车位，并将车位信息存储在`parking_info`中。

### SetParkingSpotEndPose
将目标点位姿转为以目标车位左上角为原点的局部坐标系下，并将其存储在`parking_info`中。

### GetParkingBoundary
该函数为多态函数，`GetParkingBoundary(const ParkingInfo &parking_info, const hdmap::Path &nearby_path, Frame *const frame, std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary)`将目标车位转换为局部坐标系，而后通过`GetRoadBoundary`函数计算出道路边界，而后根据车位与道路的位置关系将车位边界与道路边界组合。`GetParkingBoundary(const hdmap::Path &nearby_path, Frame *const frame, std::vector<std::vector<common::math::Vec2d>> *const roi_parking_boundary)`同样将目标车位转换为局部坐标系，而后通过`GetRoadBoundary`函数计算出道路边界，不同的是该函数会通过`right_lane_boundary[i] *= (std::fabs(left_top_l))`对道路边界进行膨胀，使其能够覆盖到车位。

### GetPullOverSpot
函数计算了靠边停车目标点的位姿，生成了目标停车边界框。

### SetPullOverSpotEndPose
将目标点位姿转换为以目标车位左上角为原点的坐标系下，并将目标位姿存入`frame`中。

### GetPullOverBoundary
该函数根据目标点车位在道路上的位置，计算出目标车位道路的左右边界，而后生成开放空间算法边界。

### SetOriginFromADC
膨胀自车box，将膨胀后的box左上角点作为坐标系原点。

### GetParkAndGoBoundary
根据车辆所在道路位置与道路信息，计算出道路边界，并将此作为开放空间算法边界。

### SetParkAndGoEndPose
根据当前车辆位置在参考线上的投影点，计算目标终点位姿，并将目标点转换到以自车左上角位姿为原点的坐标系下。

### FormulateBoundaryConstraints
用于将处理得到的障碍物边界通过函数`LoadObstacleInVertices`转换为矢量形式，而后通过函数`LoadObstacleInHyperPlanes`将障碍物矢量转换为超平面，并将数据保存在`frame`中。

## 目录结构 
```shell
modules/planning/tasks/open_space_roi_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── open_space_roi_decider.cc
├── open_space_roi_decider.h
├── open_space_roi_decider_test.cc
├── plugins.xml
├── proto
│   ├── BUILD
│   └── open_space_roi_decider.proto
└── README_cn.md
```

## 模块输入输出与配置

### planning-task-open-space-pre-stop-decider插件

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/open_space_roi_decider/conf/default_conf.pb.txt` | apollo::planning::OpenSpaceRoiDeciderConfig | OpenSpaceRoiDecider 的配置文件 |
|`modules/common/data/vehicle_param.pb.txt`|`apollo::common::VehicleConfig`|车辆底盘配置文件|

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式
##### 配置加载 OpenSpaceRoiDecider Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`OpenSpaceRoiDecider`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`OpenSpaceRoiDecider`。
```
task {
  name: "OPEN_SPACE_ROI_DECIDER"
  type: "OpenSpaceRoiDecider"
}
```
##### 配置 OpenSpaceRoiDecider 参数
在`modules/planning/tasks/open_space_roi_decider/conf/default_conf.pb.txt`中，对`OpenSpaceRoiDecider`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`OpenSpaceRoiDecider`插件的gflag参数进行配置。
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```
