# Storytelling

## 简介
Storytelling是一个全局性的高级场景管理器，用于协调跨模块的行动。为了在城市道路上安全驾驶自动驾驶汽车，需要复杂的规划场景来确保安全驾驶。这些复杂的场景可能涉及不同的模块以确保适当的机动。为了避免对这种场景采用基于序列的方法，创建了一个新的独立场景管理器，“Storytelling”模块。该模块创建复杂场景的Story，这些Story会触发多个模块的行动。根据一些预定义的规则，该模块创建一个或多个Story，并发布到“/apollo/storytelling”通道。该模块的主要优点是微调驾驶体验，并将复杂的场景隔离成Story，这些Story可以被其他模块（如规划、控制等）订阅。

## 模块

### Storytelling

#### 输入

| Channel 名                               | 类型                                            | <div style="width: 300pt">描述</div>                                           |
| ---------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------ |
| `/apollo/planning`   | `apollo::planning::ADCTrajectory`   | planning输出的规划轨迹 |     

#### 输出

| Channel 名                               | 类型                                            | <div style="width: 300pt">描述</div>                                           |
| ---------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------ |
| `/apollo/storytelling`   | `apollo::storytelling::Stories`   | Storytelling发布的Story | 

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/storytelling/conf/storytelling_conf.pb.txt`                 | `apollo::storytelling::StorytellingConfig`              |Storytelling的配置文件       |

#### 使用方式

##### 使用 mainboard 启动

```shell
mainboard -d modules/storytelling/dag/storytelling.dag
```

##### 使用 cyber_launch 启动

```shell
cyber_launch start modules/storytelling/launch/storytelling.launch