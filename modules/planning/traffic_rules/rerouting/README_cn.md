planning-traffic-rules-rerouting
============

## 简介

`Rerouting`任务用于换道没有成功时重路由
### 模块流程
产生重路由的决策主要在Rerouting::ChangeLaneFailRerouting()函数中，产生重路由的条件包括：
- 当前车道不是直行车道
- 车辆在当前参考线车道内
- 当前车道通路没有出口
- 上一次产生重路由不久
- 主车距离当前通路很近
## 目录结构

```shell

modules/planning/traffic_rules/rerouting
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── rerouting.proto
├── README_cn.md
├── rerouting.cc
└── rerouting.h

```

## 模块

### Rerouting插件

apollo::planning::Rerouting

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/traffic_rules/rerouting/conf/default_conf.pb.txt` | apollo::planning::ReroutingConfig | Rerouting的默认配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/rerouting/proto/rerouting.proto

| cooldown_time | 两次产生重路由间隔时间     |
| ------------------------------- | ---------------------- |
| prepare_rerouting_time  | 准备重路由时间         |

#### 使用方式

##### 配置加载 Rerouting 插件

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加`Rerouting`插件的配置，配置参数中`name` 表示rule的名称，这个由用户自定义，表达清楚是哪个rule即可，`type` 是rule的类名称，即`Rerouting`。

```
rule {
    name: "REROUTING"
    type: "Rerouting"
}
```
        