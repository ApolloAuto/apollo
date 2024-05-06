planning-traffic-rules-destination
==============

## 简介
`Destination` 是 planning 中 traffic rules 的一个插件，用于处理 planning 规划终点的决策判断。该插件由 TrafficDecider 进行加载，通过 ApplyRule 接口方法，根据全局路由（routing）的终点位置生成规划终点的停车决策墙；如果终点是 pull over 场景的终点，则只生成 pull over 停车决策墙。

### MakeDecisions
函数通过对输入pull_over终点位置、全局路由终点位置进行判断，来决定车辆行驶的终点停车决策。首先根据全局路由的终点位置在SL坐标系下投影计算S距离，如果判断路由终点S距离加车辆后轴中心到车辆前沿距离大于参考线的长度，则会输出警告终点不在参考线上；但是依然会生成基于路由终点的停车决策，根据路由终点位置减去目的地安全距离（配置）和虚拟墙距离（ gflag 配置），生成终点前的停车决策。如果终点是 pull over 场景的终点，则生成 pull over 停车决策墙。如果当前车辆的起点S距离大于路由终点的S距离，即参考线还距离终点较远，则不生成基于路由终点的停车决策墙。

## 目录结构
```shell
modules/planning/traffic_rules/destination/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── destination.cc
├── destination.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── destination.proto
└── README_cn.md
```

## 模块输入输出与配置

### planning-traffic-rules-destination 插件

#### 输入
| Channel名称 | 类型 | 描述 |
| --- | --- | --- |
| frame->local_view().end_lane_way_point | Frame | 全局路由路径终点 |
| frame->is_near_destination() | Frame | 是否接近终点检查 |
| reference_line_info->reference_line() | ReferenceLineInfo | 参考线 |
| reference_line.Length() | ReferenceLineInfo | 参考线长度 |
| reference_line_info->AdcSlBoundary() | ReferenceLineInfo | 主车（自车）边界 |

#### 输出
| Channel名称 | 类型 | 描述 |
| --- | --- | --- |
| reference_line_info->path_decision() | ReferenceLineInfo | 路径决策 |

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/traffic_rules/destination/conf/default_conf.pb.txt` | apollo::planning::DestinationConfig | Destination 插件的配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式

##### 配置加载 Destination 插件
在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 文件中，配置参数中 `name` 表示 traffic rule 的名称，这个由用户自定义，配置 `type` 是 Destination 的类名称，即 `Destination`。
```
task {
  name: "DESTINATION"
  type: "Destination"
}
```

##### 配置 Destination 参数
在`modules/planning/traffic_rules/destination/conf/default_conf.pb.txt`中，对 `Destination` 插件参数进行配置。

##### mainboard启动
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```