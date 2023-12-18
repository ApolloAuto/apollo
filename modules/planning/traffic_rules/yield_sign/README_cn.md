planning-traffic-rules-yield-sign
============

## 简介

`YieldSign`任务用于对于地图中的交通灯生成停车的交通决策
### 模块流程

产生交通灯决策主要在YieldSign::MakeDecisions函数中，产生交通灯决策的条件包括：
- 主车前方的参考线存在让行标记
- 让行标记id在injector_中没有标记为done
## 目录结构

```shell

modules/planning/traffic_rules/yield_sign/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── yield_sign.proto
├── README_cn.md
├── yield_sign.cc
└── yield_sign.h

```

## 模块

### YieldSign插件

apollo::planning::YieldSign

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/traffic_rules/yield_sign/conf/default_conf.pb.txt` | apollo::planning::YieldSignConfigConfig | YieldSignConfig的默认配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/yield_sign/proto/yield_sign.proto

| enabled | 是否使能     |
| ------------------------------- | ---------------------- |
| stop_distance  | 在停止线前停车距离         |

#### 使用方式

##### 配置加载 YieldSign 插件

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加`YieldSign`插件的配置，配置参数中`name` 表示rule的名称，这个由用户自定义，表达清楚是哪个rule即可，`type` 是rule的类名称，即`YieldSign`。
```
  rule {
      name: "YIELD_SIGN"
      type: "YieldSign"
  }
  ```