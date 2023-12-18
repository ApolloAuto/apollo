planning-task-path-reference-decider
==============

## 简介
`PathReferenceDecider`  该task可以对推理生成的路径的合理性进行决策，用于后续求解。


### Process
依据reference_info进行如下决策

1) reference_line_info>1,即正在进行换道，则跳过决策过程，同时将learning_model_output使用率继续保持为当前值,并设置errmsg正在换道，返回statu ok

2) 超车过程中，跳过决策过程。将learning_model_output使用率继续保持为当前值，并设置errmsg正在超车,返回statu ok

3) 判断是否有regular boundary,如果没有则返回错误码status errcode，这个过程使用了GetRegularPathBound方法

4) 推理模型没有输出时，使用基于rule的普通方法进行path决策,并返回status ok  

5) 使用AdjustTrajectoryWhichStartsFromCurrentPos方法在path中确定plan的起始点。
   当推理模型输出的size过短（<=1）时，也不使用模型推理输出的path，并返回status ok

6) 将推理出的trajectory中的点加入到pathpoints中

7) 调用IsValidPathReference方法判断是否为有效path reference，如果无效，则输出err message ，并返回ok

8) 调用EvaluatePathReference评估推理出的path reference

9) 更新推理模型有效率，记录部分log


### ConvertTrajectoryToPath
将trajectory中的路径点加入path的路径点中

### GetRegularPathBound
寻找label为regular的pathbound，遍历所有传入的boundry，判断是否有regular bound，如果寻找到regular bound，则返回bound开始到当前bound的距离。

### IsValidPathReference
判断path reference是否有效，判断方式是调用IsPointWithinPathBounds方法，判断pathreference中的点是否超出path边界。

### IsADCBoxAlongPathReferenceWithinPathBounds
未使用方法

### PathBoundToLineSegments
未使用方法

### IsPointWithinPathBounds
用于判断一个点是不是在path边界内。

### EvaluatePathReference
对推理生成的pathreference进行评估，调用discrete_path_reference.Evaluate方法

### RecordDebugInfo
记录部分与pathname pathpoints有关的信息


## 目录结构 
```shell
modules/planning/tasks/path_reference_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── path_reference_decider_test.cc
├── path_reference_decider.cc
├── path_reference_decider.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── path_reference_decider.proto
└── README_cn.md

```

## 模块

### PathReferenceDecider插件
apollo::planning::PathReferenceDecider

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/path_reference_decider/conf/default_conf.pb.txt` | apollo::planning::PathReferenceDeciderConfig | PathReferenceDecider 的配置文件 |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |

#### 使用方式
##### 配置加载 PathReferenceDecider Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`PathReferenceDecider`插件的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 PathReferenceDecider。
```
task {
  name: "PATH_REFERENCE_DECODER"
  type: "PathReferenceDecider"
}

##### 配置 PathReferenceDecider 参数
在`modules/planning/tasks/path_reference_decider/conf/default_conf.pb.txt`中，对`PathReferenceDecider`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`PathReferenceDecider`插件的gflag参数进行配置
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```