# 地图

## 介绍
地图模块是Apollo的重要组成部分，主要用于加载OpenStreetMap格式的地图数据，提供丰富的地图操作API，包括地图绘制、定位、搜索等功能。通过阿波罗地图模块，车辆可以获得实时的道路信息、交通状况、兴趣点信息等，从而进行智能路径规划、导航等操作。

## 目录结构

```shell
modules/map/
├── BUILD
├── cyberfile.xml
├── data                // 地图数据，包括矢量地图、卫星地图等
├── hdmap               // API implementations of high-precision maps.
├── pnc_map             // API实现高精度地图
├── README.md
├── relative_map        // 相对映射的API实现.
├── testdata
└── tools               // 用于处理地图数据和执行地图相关操作的工具
```

## proto说明

#### modules/common_msgs/map_msgs/map.proto
```
// This message defines how we project the ellipsoidal Earth surface to a plane.
// 坐标投影
message Projection { 
  // PROJ.4 setting:
  // "+proj=tmerc +lat_0={origin.lat} +lon_0={origin.lon} +k={scale_factor}
  // +ellps=WGS84 +no_defs"
  optional string proj = 1;
}

//地图文件头
message Header {
  optional bytes version = 1; //地图版本
  optional bytes date = 2; //地图日期
  optional Projection projection = 3; //地图使用的投影坐标系
  optional bytes district = 4; //地图园区唯一ID
  optional bytes generation = 5;
  optional bytes rev_major = 6;
  optional bytes rev_minor = 7;
  optional double left = 8;
  optional double top = 9;
  optional double right = 10;
  optional double bottom = 11;
  optional bytes vendor = 12; //供应商
}

message Map { //地图信息
  optional Header header = 1; //地图文件头描述,
  repeated Crosswalk crosswalk = 2; //人行横道元素
  repeated Junction junction = 3; //路口元素
  repeated Lane lane = 4; //车道元素
  repeated StopSign stop_sign = 5; //停车让行标识元素
  repeated Signal signal = 6; //红绿灯元素
  repeated YieldSign yield = 7; //减速让行元素
  repeated Overlap overlap = 8; //重叠关系元素
  repeated ClearArea clear_area = 9; //禁止停车区元素
  repeated SpeedBump speed_bump = 10; //减速带元素
  repeated Road road = 11; //道路元素
  repeated ParkingSpace parking_space = 12; //停车位元素
  repeated PNCJunction pnc_junction = 13; //PNCJunction元素
  repeated RSU rsu = 14; //V2X路边广播设备
}
```

#### modules/common_msgs/map_msgs/map_crosswalk.proto

Crosswalk主要是对人行横道的元素的表达。

```
// Crosswalk is a place designated for pedestrians to cross a road.
message Crosswalk {
  optional Id id = 1; //人行横道(Crosswalk)元素的全局唯一ID

  optional Polygon polygon = 2; //人行横道的外边框，地图没有表达人行横道的斑马线，而是把所有同一个方向的所有斑马线用一个外包围框包围起来

  repeated Id overlap_id = 3; //重叠id
}
```

#### modules/common_msgs/map_msgs/map_junction.proto

Junction主要是路口元素的表达。

```
// A junction is the junction at-grade of two or more roads crossing.
// 路口
message Junction {
  optional Id id = 1; //路口(Junction)元素的全局唯一ID

  optional Polygon polygon = 2; //路口区域的边界范围，包含了真实边界和虚拟边界的形状点，但无法区分哪些是真实边界，哪些是虚拟边界。

  repeated Id overlap_id = 3; //重叠id
  enum Type {
    UNKNOWN = 0;
    IN_ROAD = 1; //包含车道多变少的路口
    CROSS_ROAD = 2; //包含丁字路口，十字路口等
    FORK_ROAD = 3; //包含车道少变多的路口
    MAIN_SIDE = 4; //包含主辅路连接路的路口
    DEAD_END = 5; //包含断头路的路口
  };
  optional Type type = 4; //路口(Junction)的类型
}
```


#### modules/common_msgs/map_msgs/map_lane.proto

Lane主要描述了地图车道元素的信息

```
// 车道边界类型
message LaneBoundaryType {
  enum Type {
    UNKNOWN = 0;
    DOTTED_YELLOW = 1; //黄色虚线
    DOTTED_WHITE = 2; //白色虚线
    SOLID_YELLOW = 3; //黄色实线
    SOLID_WHITE = 4; //白色实线
    DOUBLE_YELLOW = 5; //双黄线
    CURB = 6; //路牙
  };
  // Offset relative to the starting point of boundary
  optional double s = 1; //车道边界类型相对于车道边界起始位置的偏移量。
  // support multiple types
  repeated Type types = 2; //车道边界类型
}

// 车道边界
message LaneBoundary {
  optional Curve curve = 1; //车道边界的形状点

  optional double length = 2; //车道边界的长度，即curve的长度
  // indicate whether the lane boundary exists in real world
  optional bool virtual = 3; //车道边界是否真实存在。在路口区域，真实世界中本没有车道标线，但是为了更好的支持下游使用，会人为的补充车道，这类车道边界都是虚拟车道边界。
  // in ascending order of s
  repeated LaneBoundaryType boundary_type = 4; //车道边界类型 该字段定义为数组结构是为了支持同一条车道边界上有不同边界类型的情况。
}

// Association between central point to closest boundary.
// 车道采样信息
message LaneSampleAssociation {
  optional double s = 1; //沿车道中心线几何的采样点，记录采样点位置与车道中心线起点的距离；
  optional double width = 2; //采样点位置，车道中心线与两侧车道边界几何的距离；
}

// A lane is part of a roadway, that is designated for use by a single line of
// vehicles.
// Most public roads (include highways) have more than two lanes.
// 车道类型
message Lane {
  optional Id id = 1; //目前具体由三部分构成：road id、section id、车道序号构成

  // Central lane as reference trajectory, not necessary to be the geometry
  // central.
  optional Curve central_curve = 2;  //车道中心线，作为车道的参考轨迹使用；

  // Lane boundary curve.
  optional LaneBoundary left_boundary = 3; //车道左侧边界
  optional LaneBoundary right_boundary = 4; //车道右侧边界

  // in meters.
  optional double length = 5; //车道的长度

  // Speed limit of the lane, in meters per second.
  optional double speed_limit = 6; //车道最高限速

  repeated Id overlap_id = 7; //车道与车道或其他要素的压盖关系

  // All lanes can be driving into (or from).
  repeated Id predecessor_id = 8; //当前车道的前驱车道ID
  repeated Id successor_id = 9; //当前车道的后继车道ID

  // Neighbor lanes on the same direction.
  repeated Id left_neighbor_forward_lane_id = 10; //当前车道的同向左邻车道ID
  repeated Id right_neighbor_forward_lane_id = 11; //当前车道的同向右邻车道ID

  enum LaneType { 
    NONE = 1;
    CITY_DRIVING = 2; //城市车道（机动车道）
    BIKING = 3; //自行车道（非机动车道）
    SIDEWALK = 4; //人行道
    PARKING = 5; //停车区
    SHOULDER = 6; //路肩车道
  };
  optional LaneType type = 12; //道路类型

  enum LaneTurn {
    NO_TURN = 1; //直行
    LEFT_TURN = 2; //左转
    RIGHT_TURN = 3; //右转
    U_TURN = 4; //调头
  };
  optional LaneTurn turn = 13; //转弯

  repeated Id left_neighbor_reverse_lane_id = 14; //当前车道的反向左邻车道ID
  repeated Id right_neighbor_reverse_lane_id = 15; //当前车道的反向右邻车道ID

  optional Id junction_id = 16; //当前车道所属的Junction的ID，如果当前车道不属于任何Junction，该字段为空；

  // Association between central point to closest boundary.
  repeated LaneSampleAssociation left_sample = 17; //车道中心线采样点到车道左侧边界的关联属性
  repeated LaneSampleAssociation right_sample = 18; //车道中心线采样点到车道右侧边界的关联属性

  enum LaneDirection {
    FORWARD = 1; //车道方向与车道形状点方向相同
    BACKWARD = 2; //车道方向与车道形状点方向相反
    BIDIRECTION = 3; //车道方向既可以与车道形状点方向相同，也可以与车道形状点方向相反。
  }
  optional LaneDirection direction = 19; //车道方向，以车道中心线的数字化方向为依据

  // Association between central point to closest road boundary.
  repeated LaneSampleAssociation left_road_sample = 20; //中心点与最近左路边界之间的关联
  repeated LaneSampleAssociation right_road_sample = 21; //中心点与最近右路边界之间的关联

  repeated Id self_reverse_lane_id = 22; //自相反的Lane的ID。对于双向通行的lane，地图会将其处理为两条空间重叠，但是方向相反的Lane，这两条Lane之间是自相反的关系。PNC利用这个关系实现在允许双向通行的车道上进行变道。
}
```

#### modules/common_msgs/map_msgs/map_stop_sign.proto

StopSign是对地图停车让行标识的描述。

```
// 停车让行标牌
message StopSign {
  optional Id id = 1; //停车让行标牌(StopSign)的全局唯一ID

  repeated Curve stop_line = 2; //停止线

  repeated Id overlap_id = 3;

  enum StopType {
    UNKNOWN = 0;
    ONE_WAY = 1;
    TWO_WAY = 2;
    THREE_WAY = 3;
    FOUR_WAY = 4;
    ALL_WAY = 5;
  };
  optional StopType type = 4; 停止标牌的类型，在美国比较常见
}
```

#### modules/common_msgs/map_msgs/map_signal.proto

Signal主要是对地图红绿灯元素的描述

```
// 红绿灯灯泡
message Subsignal {
  enum Type {
    UNKNOWN = 1;
    CIRCLE = 2; //圆灯
    ARROW_LEFT = 3; //左转箭头灯
    ARROW_FORWARD = 4; //直行箭头灯
    ARROW_RIGHT = 5; //右转箭头灯
    ARROW_LEFT_AND_FORWARD = 6; //左转直行箭头灯
    ARROW_RIGHT_AND_FORWARD = 7; //右转直行箭头灯
    ARROW_U_TURN = 8; //掉头箭头灯
  };

  optional Id id = 1; //红绿灯灯泡(Subsignal)的全局唯一ID
  optional Type type = 2; //灯泡的类型

  // Location of the center of the bulb. now no data support.
  optional apollo.common.PointENU location = 3; //灯泡中心位置坐标
}

// 红绿灯的标牌信息
message SignInfo {
  enum Type {
    None = 0;
    NO_RIGHT_TURN_ON_RED = 1; //红灯禁止右转
  };

  optional Type type = 1; //标牌的类型
}

// 红绿灯的信息
message Signal {
  enum Type {
    UNKNOWN = 1;
    MIX_2_HORIZONTAL = 2; //水平两灯泡
    MIX_2_VERTICAL = 3; //垂直两灯泡
    MIX_3_HORIZONTAL = 4; //水平三灯泡
    MIX_3_VERTICAL = 5; //垂直三灯泡
    SINGLE = 6; //单个灯泡
  };

  optional Id id = 1;
  optional Polygon boundary = 2; //红绿灯的边框坐标
  repeated Subsignal subsignal = 3; //对于红绿灯灯泡的描述
  // TODO: add orientation. now no data support.
  repeated Id overlap_id = 4;
  optional Type type = 5; //红绿灯的布局类型
  // stop line
  repeated Curve stop_line = 6; //红绿灯对应的停止线，一个红绿灯可能对应一个或者多个停止线

  repeated SignInfo sign_info = 7; //标牌信息
}

```

#### modules/common_msgs/map_msgs/map_yield_sign.proto

// YieldSign主要是对减速让行标牌的描述
```
message YieldSign {
  optional Id id = 1; //停车让行(yield)标牌的全局唯一ID

  repeated Curve stop_line = 2; //停车让行标识对应的停止线

  repeated Id overlap_id = 3;
}
```

#### modules/common_msgs/map_msgs/map_overlap.protp

// Lane的Overlap信息
```
message LaneOverlapInfo {
  optional double start_s = 1;  // 车道线交点在Lane中心线上的最小偏距
  optional double end_s = 2;    // 车道线交点部分在Lane中心线上的最大偏距
  optional bool is_merge = 3;  // 仅当Lane与Lane的Overlap时，才有该字段

  optional Id region_overlap_id = 4; //egion overlap的ID
}

message SignalOverlapInfo {}

message StopSignOverlapInfo {}

message CrosswalkOverlapInfo {
  optional Id region_overlap_id = 1;
}

message JunctionOverlapInfo {}

message YieldOverlapInfo {}

message ClearAreaOverlapInfo {}

message SpeedBumpOverlapInfo {}

message ParkingSpaceOverlapInfo {}

message PNCJunctionOverlapInfo {}

message RSUOverlapInfo {}

message RegionOverlapInfo {
  optional Id id = 1;
  repeated Polygon polygon = 2;
}

// Information about one object in the overlap.
message ObjectOverlapInfo {
  optional Id id = 1; //Overlap元素的ID，元素的类型可以根据第二个字段判断，比如如果overlap_info为lane_overlap_info，则id为车道(lane)的id

  oneof overlap_info {
    LaneOverlapInfo lane_overlap_info = 3;
    SignalOverlapInfo signal_overlap_info = 4;
    StopSignOverlapInfo stop_sign_overlap_info = 5;
    CrosswalkOverlapInfo crosswalk_overlap_info = 6;
    JunctionOverlapInfo junction_overlap_info = 7;
    YieldOverlapInfo yield_sign_overlap_info = 8;
    ClearAreaOverlapInfo clear_area_overlap_info = 9;
    SpeedBumpOverlapInfo speed_bump_overlap_info = 10;
    ParkingSpaceOverlapInfo parking_space_overlap_info = 11;
    PNCJunctionOverlapInfo pnc_junction_overlap_info = 12;
    RSUOverlapInfo rsu_overlap_info = 13;
  }
}

// Here, the "overlap" includes any pair of objects on the map
// (e.g. lanes, junctions, and crosswalks).
message Overlap {
  optional Id id = 1; //Overlap元素的全局唯一标识

  // Information about one overlap, include all overlapped objects.
  repeated ObjectOverlapInfo object = 2; //Overlap表示两两元素的关系，所以一个Overlap一般包含两个ObjectOverlapInfo元素

  repeated RegionOverlapInfo region_overlap = 3; //两个元素的region_overlap信息应该是一致的，为了避免重复存储，所以把这个信息统一放到Overlap结构中
}
```

#### modules/common_msgs/map_msgs/map_clear_area.proto

ClearArea主要是对道路禁止停车区的表达

```
message ClearArea {
  optional Id id = 1; //禁停区(ClearArea)元素的全局唯一ID
  repeated Id overlap_id = 2; //禁停区几何与其他元素的overlap关系ID
  optional Polygon polygon = 3; //禁止停车区的外边框
}
```

#### modules/common_msgs/map_msgs/map_speed_bump.proto

SpeedBump主要是对地图中减速带元素的描述

```
message SpeedBump {
  optional Id id = 1; //减速带(SpeedBump)的全局唯一ID
  repeated Id overlap_id = 2;
  repeated Curve position = 3; //减速带的停止线
}
```

#### modules/common_msgs/map_msgs/map_road.proto

```
// 道路边界
message BoundaryEdge {
  optional Curve curve = 1; //道路边界的几何，线状几何，由形状点构成
  enum Type {
    UNKNOWN = 0;
    NORMAL = 1; //左右边界之外的其它类型
    LEFT_BOUNDARY = 2; //道路左边界
    RIGHT_BOUNDARY = 3; //道路右边界
  };
  optional Type type = 2;
}

message BoundaryPolygon {
  repeated BoundaryEdge edge = 1; //BoundaryPolygon是BoundaryEdge的集合
}

// boundary with holes
message RoadBoundary {
  optional BoundaryPolygon outer_polygon = 1; //道路的物理外边界
  // if boundary without hole, hole is null
  repeated BoundaryPolygon hole = 2; //道路边界中间的孔洞
}

message RoadROIBoundary {
  optional Id id = 1;
  repeated RoadBoundary road_boundaries = 2;
}

// road section defines a road cross-section, At least one section must be
// defined in order to
// use a road, If multiple road sections are defined, they must be listed in
// order along the road
message RoadSection {
  optional Id id = 1;
  // lanes contained in this section
  repeated Id lane_id = 2; //该Section中包含的所有的Lane的ID
  // boundary of section
  optional RoadBoundary boundary = 3; //道路边界
}

// The road is a collection of traffic elements, such as lanes, road boundary
// etc.
// It provides general information about the road.
// 道路
message Road {
  optional Id id = 1; //道路(Road)元素的全局唯一标识
  repeated RoadSection section = 2; //Road中包含的所有的Section，一个Road可以包含一个或者多个Section

  // if lane road not in the junction, junction id is null.
  optional Id junction_id = 3; //如果Road在Junction中，junction_id为该Junction的ID；如果Road不在Junction中，该字段为空

  enum Type {
    UNKNOWN = 0;
    HIGHWAY = 1; //高速路
    CITY_ROAD = 2; //城市路
    PARK = 3; //内部路
  };
  optional Type type = 4; //道路等级
}
```

#### modules/common_msgs/map_msgs/map_parking_space.proto

ParkingSpace是对单个停车位的描述

```
message ParkingSpace {
  optional Id id = 1; //停车位(ParkingSpace)元素的全局唯一标识

  optional Polygon polygon = 2; //停车位(ParkingSpace)元素的边界

  repeated Id overlap_id = 3;

  optional double heading = 4; //停车位(ParkingSpace)的朝向
}

// ParkingLot is a place for parking cars.
message ParkingLot {
  optional Id id = 1;

  optional Polygon polygon = 2;

  repeated Id overlap_id = 3;
}
```

#### modules/common_msgs/map_msgs/map_pnc_junction.proto

PNCJunction是对人理解的路口的表达，相对于Junction，它对边界有更高的约束，比如要求边界沿着停止线等等

```
message Passage {
  optional Id id = 1;

  repeated Id signal_id = 2;
  repeated Id yield_id = 3;
  repeated Id stop_sign_id = 4;
  repeated Id lane_id = 5;

  enum Type {
    UNKNOWN = 0;
    ENTRANCE = 1;
    EXIT = 2;
  };
  optional Type type = 6;
};

message PassageGroup {
  optional Id id = 1;

  repeated Passage passage = 2;
};

message PNCJunction {
  optional Id id = 1; //PNC Junction元素的全局唯一标识

  optional Polygon polygon = 2; //PNC Junction的边界  

  repeated Id overlap_id = 3;

  repeated PassageGroup passage_group = 4;
}
```

#### modules/common_msgs/map_msgs/map_rsu.proto

```
message RSU {
  optional Id id = 1; //路边广播设备(RSU)的全局唯一ID
  optional Id junction_id = 2; //RSU所属的Junction ID
  repeated Id overlap_id = 3; //RSU要素相关的Overlap ID
}
```
