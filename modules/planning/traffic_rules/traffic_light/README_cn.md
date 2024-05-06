planning-traffic-rules-traffic-light
============

## 简介

`TrafficLight`任务用于对于地图中的交通灯生成停车的交通决策
### 模块流程
产生交通灯决策主要在TrafficLight::MakeDecisions函数中，产生交通灯决策的条件包括：
- 主车前方的参考线存在交通灯停止线
- 交通灯id在injector_中没有标记为done
- 交通灯的颜色不是绿色
- 从车辆当前位置到交通灯刹停的最小加速度小于设定的最大刹车max_stop_deceleration
## 目录结构

```shell

modules/planning/traffic_rules/traffic_light/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── traffic_light.proto
├── README_cn.md
├── traffic_light.cc
└── traffic_light.h

```

## 模块

### TrafficLight插件

apollo::planning::TrafficLight

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/traffic_rules/traffic_light/conf/default_conf.pb.txt` | apollo::planning::TrafficLightConfig | TrafficLight的默认配置文件 |

#### 模块参数说明

算法参数配置定义于modules/planning/tasks/traffic_light/proto/traffic_light.proto

| enabled | 是否使能     |
| ------------------------------- | ---------------------- |
| stop_distance  | 在停止线前停车距离         |
|max_stop_deceleration|最大刹车减速度|

#### 使用方式

  ##### 配置加载 TrafficLight 插件

  在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加`TrafficLight`插件的配置，配置参数中`name` 表示rule的名称，这个由用户自定义，表达清楚是哪个rule即可，`type` 是rule的类名称，即`TrafficLight`。

  ```
    rule {
        name: "TRAFFIC_LIGHT"
        type: "TrafficLight"
    }
    ```