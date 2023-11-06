# 预测

## 介绍
预测模块研究并预测感知模块检测到的所有障碍物的行为。预测接收障碍物数据以及包括位置、航向、速度、加速度在内的基本感知信息，然后生成具有这些障碍物的概率的预测轨迹。预测模块由四个子模块组成：  **容器**, **场景**, **评估器** 和 **预测器**. 


```
注意:
预测模块仅预测障碍物的行为，而不预测主车的行为。规划模块规划主车的轨迹。
```

### 容器

容器存储来自订阅频道的输入数据。当前支持的输入为： **_perception obstacles_**, **_vehicle localization_** 和 **_vehicle planning_**.

### 场景

场景子模块分析包括主车在内的场景。目前，我们有两种定义的场景：

- **Cruise** : 该场景包括车道保持和跟车
- **Junction** : 这个场景涉及交叉路口，路口可以有红绿灯和/或STOP标志。

我们还定义了三种类型的障碍优先级：

- **Ignore**: 这些障碍物不会影响主车的轨迹，可以安全地忽略（例如障碍物太远）
- **Caution**: 这些障碍物很有可能与主车有交互
- **Normal**: 不属于忽略或警告范围的障碍物默认置于正常情况


### 评估器

评估器分别预测任何给定障碍物的路径和速度。评估器通过使用存储在 _prediction/data/_ 中的给定模型来评估路径，并输出车道序列的概率

可用评估器列表包括：

* **Cost evaluator**: 用一组成本函数计算概率。

* **MLP evaluator**: 用MLP模型计算概率。

* **RNN evaluator**: 用RNN模型计算概率。

* **Cruise MLP + CNN-1d evaluator**: 使用MLP和CNN-1d混合模型计算巡航场景的概率。

* **Junction MLP evaluator**: 使用交叉口场景的MLP模型计算概率。

* **Junction Map evaluator**: 路口场景下，使用基于语义图的CNN模型来计算概率. 此评估器是为Caution级别的障碍而创建的。

* **Social Interaction evaluator**: 该模型用于行人的短期轨迹预测。它使用Social LSTM。此评估器是为Caution级别的障碍而创建的。

* **Semantic LSTM evaluator**: 该评估器被用于新的Caution Obstructure模型中，以生成使用CNN和LSTM计算的短期轨迹点。车辆和行人都使用相同的模型，但参数不同。

* **Vectornet LSTM evaluator**: 该评估器用于代替语义LSTM评估器，为Caution级别的障碍物生成短期轨迹点。 

* **Jointly prediction planning evaluator**: 该评估器用于新的交互式障碍物（车辆类型）模型中，以生成短期轨迹点，该轨迹点使用Vectornet和LSTM进行计算。通过考虑ADC的轨迹信息，可以在交互场景下更准确地预测障碍物的轨迹。

### 预测器

预测器生成障碍物的预测轨迹。目前，支持的预测器包括：

* **Empty**: 障碍物没有预测的轨迹。
* **Single lane**: 在公路导航模式下障碍物沿着单条车道移动。不在车道上的障碍物将被忽略。
* **Lane sequence**: 障碍物沿车道移动。
* **Move sequence**: 障碍物沿其运动模式沿车道移动。
* **Free movement**: 障碍物自由移动。
* **Regional movement**: 障碍物在可能的区域中移动。
* **Junction**: 障碍物向路口移动的可能性很高。
* **Interaction predictor**: 计算在所有评估器运行之后创建后验预测结果的可能性。此预测器是为Caution级别的障碍而创建的。
* **Extrapolation predictor**: 扩展语义LSTM评估器的结果以创建8秒的轨迹。

## 目录结构

```
├── prediction
    ├── common                  // common code        
    ├── conf                    // configuration folder   
    ├── container               // container sub-module
    │   ├── adc_trajectory   
    │   ├── obstacles 
    │   ├── pose   
    │   └── storytelling              
    ├── dag                     // module startup file     
    ├── data                    // module configuration parameters
    ├── evaluator               // evaluator sub-module
    │   ├── cyclist   
    │   ├── model_manager 
    │   ├── pedestrian
    │   ├── vehicle     
    │   └── warm_up            
    ├── images                  // demo images
    ├── launch                  // launch file
    ├── network                 // network code
    ├── pipeline                // VectorNet code
    ├── predictor               // predictor sub-module
    │   ├── empty   
    │   ├── extrapolation 
    │   ├── free_move
    │   ├── interaction   
    │   ├── junction 
    │   ├── lane_sequence  
    │   ├── move_sequence   
    │   ├── sequence    
    │   └── single_lane            
    ├── proto                   // configuration proto file
    ├── scenario                // scenario sub-module
    │   ├── analyzer   
    │   ├── feature_extractor 
    │   ├── interaction_filter
    │   ├── prioritization   
    │   ├── right_of_way   
    │   └── scenario_features            
    ├── submodules              // manage evaluator and predictor submodules
    ├── testdata                // test data
    ├── BUILD                   // compile file
    ├── cyberfile.xml           // package management file
    ├── prediction_component.cc //component entrance
    ├── prediction_component.h   
    └── prediction_component_test.cc
    
```

## 模块

### PredictionComponent

#### 输入

| 名称    | 类型                                             | 描述         |
| ------- | ------------------------------------------------ | ------------------- |
| `frame` | `apollo::perception::PerceptionObstacles` | 障碍物信息 |


#### 输出

| 名称    | 类型                                             | 描述                    |
| ------- | ------------------------------------------------ | ------------------------------ |
| `frame` | `apollo::prediction::PredictionObstacles` | 障碍物预测信息 |

#### 使用方式

1. 修改 `modules/prediction/dag/prediction.dag`

- config\_file_path: 配置文件路径
- flag\_file_path: flag文件路径
- reader channel: 输入通道的名称

2. 修改 `modules/prediction/conf/prediction_conf.pb.txt`

- topic\_conf: 不同主题的名称
  - xxx\_topic_name： the name of xxx topic
- evaluator\_model_conf: 不同评估器的配置文件
  - evaluator\_type: SEMANTIC\_LSTM_EVALUATOR
  - obstacle_type:  PEDESTRIAN, VEHICLE
  - backend: GPU, CPU 
  - priority: 模型优先级
  - type: SemanticLstmPedestrianGpuTorch, SemanticLstmVehicleGpuTorch ...

- obstacle\_conf: 不同类型障碍物的配置文件
  - obstacle\_type: VEHICLE, PEDESTRIAN, BICYCLE, UNKNOWN
  - obstacle\_status: ON\_LANE, OFF\_LANE, IN\_JUNCTION, MOVING
  - interactive\_tag: INTERACTION
  - priority\_type: CAUTION, NORMAL,IGNORE
  - evaluator\_type: VECTORNET\_EVALUATOR, CRUISE\_MLP_EVALUATOR ...
  - predictor\_type: EXTRAPOLATION\_PREDICTOR, MOVE_SEQUENCE\_PREDICTOR ...

3. 启动预测模块

```bash
cyber_launch start modules/prediction/launch/prediction.launch
```

## 参考引用

1. [Xu K, Xiao X, Miao J, Luo Q. "Data Driven Prediction Architecture for Autonomous Driving and its Application on Apollo Platform." *arXiv preprint arXiv:2006.06715.* ](https://arxiv.org/pdf/2006.06715.pdf)
