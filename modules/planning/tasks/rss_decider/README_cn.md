planning-task-rss-decider
==============

## 简介
`RSS Decider`RSS决策器。RSS，resposibility sensitive safety，用于判断planning给出的规划是否安全

### rss_config_default_dynamics
设定部分所需的rss默认参数

### rss_create_ego_object
创建主车object，并为主车object赋值

### rss_create_other_object
创建其他车辆/交通参与者的object，并为object赋值

### rss_dump_world_info
打印部分rss相关信息

### Process
该函数通过如下方式实现rss策略：
1) 初始化主车boundingbox的前后左右边界，初始化最近障碍物边界(都为0) 

2) 遍历所有障碍物列表中的障碍物，忽略虚拟障碍物和不在路上的障碍物，忽略后向障碍物，计算出沿s方向的最近前向障碍物。

3) 如果最近前向障碍物距离大于等于rss_max_front_obstacle_distance,则判定为rss安全，并设置rssinfo中的前、左、右最大最小距离。

4) 如果不满足上述安全情况，则开始进行如下逻辑

4.1 记录障碍物和主车部分相关数据，加入到rss_info中，方便后续输出log使用。
4.2 leading Object和followingObject信息加入worldModel中，用于后续判断。
4.3 分别调用 RssSituationExtraction,checkSituations,provideProperResponse,
transformProperResponse,calculateSafeLongitudinalDistanceSameDirection对结果进行判断，并返回结果，输出日志。



## 目录结构 
```shell
modules/planning/tasks/rss_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── rss_decider.cc
├── rss_decider.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── rss_decider.proto
└── README_cn.md

```

## 模块

### RssDeciderh插件
apollo::planning::RssDecider

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/rss_decider/conf/default_conf.pb.txt` | apollo::planning::RssDeciderConfig | RssDecider 的配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式
##### 配置加载 RssDecider Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`RssDecider`插件的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 RssDecider
```
task {
  name: "RSS_DECIDER"
  type: "RssDecider"
}
```
##### 配置 RssDecider 参数
在`modules/planning/tasks/rss_decider/conf/default_conf.pb.txt`中，对`RssDecider`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`RssDecider`插件的gflag参数进行配置
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```