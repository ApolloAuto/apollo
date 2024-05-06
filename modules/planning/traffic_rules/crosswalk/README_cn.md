planning-traffic_rules-crosswalk
============

## 简介

`Crosswalk`用于在人行横道处生成虚拟障碍物, 礼让行人优先通行

#### 模块流程
1. 遍历参考线上的人行横道信息, 若自车车头已越过某处的人行横道, 则忽略该人行横道
2. 遍历障碍物信息, 满足下列条件, 则需停车让行
    - 属于行人和非机动车类型
    - 在人行横道的区域范围内
    - 障碍物要横穿自车path, 或在自车前方的道路范围内
    - 自车停车所需的减速度小于所允许的最大减速度
3. 记录各人行横道所需停车让行的障碍物
4. 在需要停车让行的人行横道处, 生成虚拟障碍物, 用于停车

## 目录结构

```shell
modules/planning/traffic_rules/crosswalk/
├── BUILD                       // 构建规则文件
├── README_cn.md                // 说明文档
├── crosswalk.cc                // 源文件
├── crosswalk.h                 // 头文件
├── conf                        // 参数配置文件夹
│   └── default_conf.pb.txt
├── cyberfile.xml               // 包管理配置文件
├── plugins.xml                 // 插件规则文件
└── proto                       // 配置定义文件夹
    ├── BUILD
    └── crosswalk.proto
```

## 模块

### Crosswalk插件

apollo::planning::Crosswalk

#### 配置文件 & 配置项

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt` | `apollo::planning::CrosswalkConfig` |配置文件 |

| 配置项 | 说明 |
| ----- | ----- |
| stop_distance         | 生成虚拟障碍物的位置距离人行横道的距离 |
| max_stop_deceleration | 判断停车时允许的最大减速度 |
| min_pass_s_distance   | 判断自车是否越过人行横道的距离 |
| expand_s_distance     | 人行横道在s方向的扩展距离 |
| stop_strict_l_distance| 障碍物在l方向的距离小于此阈值时, 若在道路内则停车 |
| stop_loose_l_distance | 障碍物在l方向的距离大于此阈值时, 若横穿自车path则停车 |
| stop_timeout          | 障碍物静止时间超过此阈值, 则不在停车让行此障碍物 |

#### 使用方式

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加 `Crosswalk` 插件的配置

```
rule {
name: "CROSSWALK"
type: "Crosswalk"
}
```
