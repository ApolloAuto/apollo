planning-traffic_rules-keep-clear
============

## 简介

`KeepClear`用于在禁停区域生成虚拟静止障碍物, 避免自车进入禁停区域

#### 模块流程
- 打开禁停区域开关后, 遍历自车参考线上的禁停区域信息, 若自车已经进入禁停区域, 则继续行驶; 若自车还未进入禁停区域, 则在禁停区域开始位置处生成虚拟静止障碍物
- 打开路口禁停的开关后, 若自车未处于蠕行状态, 则将路口区域看作禁停区域

## 目录结构

```shell
modules/planning/traffic_rules/keep_clear/
├── BUILD                       // 构建规则文件
├── README_cn.md                // 说明文档
├── keep_clear.cc               // 源文件
├── keep_clear.h                // 头文件
├── conf                        // 参数配置文件夹
│   └── default_conf.pb.txt
├── cyberfile.xml               // 包管理配置文件
├── plugins.xml                 // 插件规则文件
└── proto                       // 配置定义文件夹
    ├── BUILD
    └── keep_clear.proto
```

## 模块

### KeepClear插件

apollo::planning::KeepClear

#### 配置文件 & 配置项

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/traffic_rules/keep_clear/conf/default_conf.pb.txt` | `apollo::planning::KeepClearConfig` |配置文件 |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |

| 配置项 | 说明 |
| ----- | ----- |
| enable_keep_clear_zone            | 使能禁停区域的开关 |
| enable_junction                   | 使能路口禁停的开关 |
| min_pass_s_distance               | 判断自车是否进入禁停区域的距离阈值 |
| align_with_traffic_sign_tolerance | 对齐路口与红绿灯/人行横道/停车标志的距离阈值 |

#### 使用方式

在 `modules/planning/planning_component/conf/traffic_rule_config.pb.txt` 增加 `KeepClear` 插件的配置

```
rule {
name: "KEEP_CLEAR"
type: "KeepClear"
}
```
