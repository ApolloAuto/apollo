planning-traffic-rules-reference-line-end
==============

## 简介
`ReferenceLineEnd` 是 planning 中 traffic rules 的一个插件，用于生成在 reference_line （参考线）末端的停车决策。该插件由 TrafficDecider 进行加载，通过 ApplyRule 接口方法，在满足自车距离参考线的阈值内，会基于参考线的末端前的一定距离（FLAGS_virtual_stop_wall_length），生成一个虚拟障碍物停止墙决策。

### MakeDecisions
函数通过判断自车到参考线末端的剩余距离满足阈值内（参数配置 min_reference_line_remain_length ），进行生成虚拟停车决策墙。根据当前参考线内的车道线ID构建虚拟障碍物，在虚拟障碍物前的一定阈值（配置参数 stop_distance ）内构建停车线，然后生成路径决策的停车决策。

## 目录结构
```shell
modules/planning/traffic_rules/reference_line_end/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── reference_line_end.cc
├── reference_line_end.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── reference_line_end.proto
└── README_cn.md
```

## 模块输入输出与配置

### planning-traffic-rules-reference-line-end 插件

#### 输入
| Channel名称 | 类型 | 描述 |
| --- | --- | --- |
| reference_line.Length() | ReferenceLineInfo | 参考线长度 |
| reference_line_info->Lanes() | ReferenceLineInfo | 参考线车道线信息 |
| reference_line_info->AdcSlBoundary() | ReferenceLineInfo | 主车（自车）边界 |

#### 输出
| Channel名称 | 类型 | 描述 |
| --- | --- | --- |
| reference_line_info->path_decision() | ReferenceLineInfo | 路径决策 |

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/traffic_rules/reference_line_end/conf/default_conf.pb.txt` | apollo::planning::ReferenceLineEndConfig | ReferenceLineEnd 插件的配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式
##### 配置加载 ReferenceLineEnd 插件
在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 文件中，配置参数中 `name` 表示 traffic rule 的名称，这个由用户自定义，配置 `type` 是 ReferenceLineEnd 的类名称，即 `ReferenceLineEnd`。
```
task {
  name: "REFERENCE_LINE_END"
  type: "ReferenceLineEnd"
}
```

##### 配置 ReferenceLineEnd 参数
在`modules/planning/traffic_rules/reference_line_end/conf/default_conf.pb.txt`中，对 `ReferenceLineEnd` 插件参数进行配置。

##### mainboard启动
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```