planning-traffic_rules-backside-vhicle
============

## 简介

`BacksideVehicle`用于产生后方来车是否忽略的决策, 决策结果保存在 reference_line_info 的 path_decision 中

#### 模块流程
遍历所有障碍物, 根据其位置信息, 判断是否产生忽略的决策, 条件包括:
- 不能忽略自车前方或侧方的障碍物
- 忽略从自车正后方过来的障碍物

## 目录结构

```shell
modules/planning/traffic_rules/backside_vehicle/
├── BUILD                       // 构建规则文件
├── README_cn.md                // 说明文档
├── backside_vehicle.cc         // 源文件
├── backside_vehicle.h          // 头文件
├── conf                        // 参数配置文件夹
│   └── default_conf.pb.txt
├── cyberfile.xml               // 包管理配置文件
├── plugins.xml                 // 插件规则文件
└── proto                       // 配置定义文件夹
    ├── BUILD
    └── backside_vehicle.proto
```

## 模块

### BacksideVehicle插件

apollo::planning::BacksideVehicle

#### 配置文件 & 配置项

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/traffic_rules/backside_vehicle/conf/default_conf.pb.txt` | `apollo::planning::BacksideVehicleConfig` | traffic rule的默认配置文件 |

#### 使用方式

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加 `BacksideVehicle` 插件的配置

```
rule {
name: "BACKSIDE_VEHICLE"
type: "BacksideVehicle"
}
```
