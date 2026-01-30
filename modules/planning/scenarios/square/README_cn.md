# planning-scenario-square

## 介绍

`planning-scenario-square` 是用于执行广场区域脱困绕行驾驶，在车辆进入`junction`区域后触发该scenario。以广场区域边界作为道路边界约束，并支持车辆阻塞后向后倒车脱困。

- **pipeline**：配置两个`stage`：`SQUARE_LANE_FOLLOW_STAGE`和`EXTRICATE_STAGE`
  - `SQUARE_LANE_FOLLOW_STAGE`:与主路行车`LANE_FOLLOW_STAGE`相比，减少`LANE_CHANGE_PATH`和`LANE_FOLLOW_PATH`的task，增加了`SQUARE_PATH`的task。在该task中，以junction边界作为道路约束边界，扩大了驾驶空间。
  - `EXTRICATE_STAGE`：在`SQUARE_LANE_FOLLOW_STAGE`求解失败或没有前进Path时，采取倒车轨迹规划。增加`REVERSE_PATH`和`REVERSE_SPEED_DECIDER`进行倒车路径和速度规划，直到前进轨迹规划成功。
- **场景切入条件**：车辆进入`junction`区域。

## 目录结构

```shell

modules/planning/scenarios/square/
├── square
    ├── BUILD                // 构建规则文件
    ├── conf                 // 参数配置文件
    ├── context.h            // 场景上下文信息
    ├── cyberfile.xml        // 包管理配置文件
    ├── extricate_stage.cc          // Stage类源码，倒车脱困阶段
    ├── extricate_stage.h           // Stage类头文
    ├── proto                       // 消息定义
    ├── README_cn.md                // 说明文档
    ├── square_lane_follow_stage.cc     // Stage类源码，车道保持阶段
    ├── square_lane_follow_stage.h      // Stage类头文
    ├── square_scenario.cc              // 场景类源码
    └── square_scenario.h               // 场景类头文件
```

## 插件

### SquareScenario

apollo::planning::SquareScenario

#### 配置

| 文件路径                                                 | 类型/结构                            | <div style="width: 300pt">说明</div>  |
| -------------------------------------------------------- | ------------------------------------ | ------------------------------------- |
| `modules/planning/scenarios/square/conf/pipeline.pb.txt` | `apollo::planning::ScenarioPipeline` | 配置文件，场景中运行的Stage和Task列表 |

#### Flags

| 文件路径                                                                                          | <div style="width: 300pt">说明</div> |
| ------------------------------------------------------------------------------------------------- | ------------------------------------ |
| `modules/planning/scenarios/square/conf/extricate_stage/lane_borrow_path_generic.pb.txt`          | 脱困阶段，借道绕行task参数           |
| `modules/planning/scenarios/square/conf/extricate_stage/piecewise_jerk_speed.pb.txt`              | 脱困阶段，速度规划参数               |
| `modules/planning/scenarios/square/conf/square_lane_follow_stage/lane_borrow_path_generic.pb.txt` | 前进规划阶段，借道绕行task参数       |
| `modules/planning/scenarios/square/conf/square_lane_follow_stage/path_decider.pb.txt`             | 前进规划阶段，路径决策参数           |

#### 使用方式
##### 添加包依赖
如果使用包管理的方式运行apollo，在包管理的cyber.xml文件中添加对插件的依赖：
```shell
<depend repo_name="planning-scenario-square" type="binary">planning-scenario-square</depend>
```
##### 插件加载配置
所有场景的插件都是在planning包中加载运行的，如果需要支持square的场景时，在配置文件`modules/planning/planning_component/conf/public_road_planner_config.pb.txt`中添加以下配置：
```shell
  scenario {
    name: "SQUARE"
    type: "SquareScenario"
  }
```
