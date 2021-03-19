# DreamView用法介绍

DreamView是一个web应用程序，提供如下的功能：
1. 可视化显示当前自动驾驶车辆模块的输出信息，例如规划路径、车辆定位、车架信息等。
2. 为使用者提供人机交互接口以监测车辆硬件状态，对模块进行开关操作，启动自动驾驶车辆等。
3. 提供调试工具，例如PnC监视器可以高效的跟踪模块输出的问题

## 界面布局和特性

该应用程序的界面被划分为多个区域：标题、侧边栏、主视图和工具视图。

### 标题
标题包含4个下拉列表，可以像下述图片所示进行操作：
![](images/dreamview_usage_table/header.png) 

附注：导航模块是在Apollo 2.5版本引入的满足低成本测试的特性。在该模式下，Baidu或Google地图展现的是车辆的绝对位置，而主视图中展现的是车辆的相对位置。

### 侧边栏和工具视图
![](images/dreamview_usage_table/sidebar.png) 
侧边栏控制着显示在工具视图中的模块

### Tasks
在DreamView中使用者可以操作的tasks有：
![](images/dreamview_usage_table/tasks.png)

* **Quick Start**: 当前选择的模式支持的指令。通常情况下，

    **setup**: 开启所有模块

    **reset all**: 关闭所有模块

    **start auto**: 开始车辆的自动驾驶
* **Others**: 工具经常使用的开关和按钮
* **Module Delay**: 从模块中输出的两次事件的时间延迟
* **Console**: 从Apollo平台输出的监视器信息

### Module Controller
监视硬件状态和对模块进行开关操作
![](images/dreamview_usage_table/module_controller.png) 

### Layer Menu
显式控制各个元素是否显示的开关
![](images/dreamview_usage_table/layer_menu.png) 

### Route Editing
在向Routing模块发送寻路信息请求前可以编辑路径信息的可视化工具
![](images/dreamview_usage_table/route_editing.png)

### Data Recorder
将问题报告给rosbag中的drive event的界面
![](images/dreamview_usage_table/data_recorder.png)  

### Default Routing
预先定义的路径或者路径点，该路径点称为兴趣点（POI）。

![](images/dreamview_usage_table/default_routing.png) 

如果打开了路径编辑模式，路径点可被显式的在地图上添加。

如果关闭了路径编辑模式，点击一个期望的POI会向服务器发送一次寻路请求。如果只选择了一个点，则寻路请求的起点是自动驾驶车辆的当前点。否则寻路请求的起点是选择路径点中的第一个点。

查看Map目录下的[default_end_way_point.txt](../../modules/map/data/demo/default_end_way_point.txt)文件可以编译POI信息。例如，如果选择的地图模式为“Demo”，则在`modules/map/data/demo`目录下可以查看对应的 [default_end_way_point.txt](../../modules/map/data/demo/default_end_way_point.txt) 文件。

### 主视图
主视图在web页面中以动画的方式展示3D计算机图形

![](images/dreamview_usage_table/mainview.png) 

下表列举了主视图中各个元素：

| Visual Element                           | Depiction Explanation                    |
| ---------------------------------------- | ---------------------------------------- |
| ![](images/dreamview_usage_table/0clip_image002.png) | <ul><li>自动驾驶车辆    </li></ul>                  |
| ![](images/dreamview_usage_table/0clip_image004.png) | <ul><li>车轮转动的比率</li> <li>左右转向灯的状态</li></ul> |
| ![](images/dreamview_usage_table/0clip_image003.png) | <ul><li>交通信号灯状态</li></ul>          |
| ![](images/dreamview_usage_table/0clip_image005.png) |<ul><li>  驾驶状态（AUTO/DISENGAGED/MANUAL等） </li></ul>  |
| ![](images/dreamview_usage_table/0clip_image006.png) | <ul><li>行驶速度 km/h</li> <li>加速速率/刹车速率</li></ul> |
| ![](images/dreamview_usage_table/0clip_image026.png) | <ul><li> 红色粗线条表示建议的寻路路径</li></ul>  |
| ![](images/dreamview_usage_table/0clip_image038.png) |<ul><li>  轻微移动物体决策—橙色表示应该避开的区域 </li></ul> |
| ![](images/dreamview_usage_table/0clip_image062.png) |<ul><li>  绿色的粗曲线条带表示规划的轨迹 </li></ul> |

#### 障碍物

| Visual Element                           | Depiction Explanation                    |
| ---------------------------------------- | ---------------------------------------- |
| ![](images/dreamview_usage_table/0clip_image010.png) | <ul><li>车辆障碍物   </li></ul>                     |
| ![](images/dreamview_usage_table/0clip_image012.png) | <ul><li>行人障碍物    </li></ul>                 |
| ![](images/dreamview_usage_table/0clip_image014.png) | <ul><li>自行车障碍物      </li></ul>                |
| ![](images/dreamview_usage_table/0clip_image016.png) | <ul><li>未知障碍物 </li></ul>                        |
| ![](images/dreamview_usage_table/0clip_image018.png) | <ul><li>速度方向显示了移动物体的方向，长度随速度按照比率变化</li></ul>  |
| ![](images/dreamview_usage_table/0clip_image020.png) | <ul><li>白色箭头显示了障碍物的移动方向</li></ul>  |
| ![](images/dreamview_usage_table/0clip_image022.png) | 黄色文字表示: <ul><li>障碍物的跟踪ID</li><li>自动驾驶车辆和障碍物的距离及障碍物速度</li></ul> |
| ![](images/dreamview_usage_table/0clip_image024.png) | <ul><li>线条显示了障碍物的预测移动轨迹，线条标记为和障碍物同一个颜色</li></ul>  |

#### Planning决策
##### 决策栅栏区

决策栅栏区显示了Planning模块对车辆障碍物做出的决策。每种类型的决策会表示为不同的颜色和图标，如下图所示：

| Visual Element                           | Depiction Explanation                    |
| ---------------------------------------- | ---------------------------------------- |
| ![](images/dreamview_usage_table/0clip_image028.png) | <ul><li>**停止** 表示物体主要的停止原因</li></ul>  |
| ![](images/dreamview_usage_table/0clip_image030.png) | <ul><li>**停止** 表示物体的停止原因n</li></ul>  |
| ![2](images/dreamview_usage_table/0clip_image032.png) | <ul><li>**跟车** 物体</li></ul>                        |
| ![](images/dreamview_usage_table/0clip_image034.png) | <ul><li>**让行** 物体决策—点状的线条连接了各个物体</li></ul>  |
| ![](images/dreamview_usage_table/0clip_image036.png) | <ul><li>**超车** 物体决策—点状的线条连接了各个物体</li></ul>  |

线路变更是一个特殊的决策，因此不显示决策栅栏区，而是将路线变更的图标显示在车辆上。

| Visual Element                           | Depiction Explanation                    |
| ---------------------------------------- | ---------------------------------------- |
| ![](images/dreamview_usage_table/change-lane-left.png) | <ul><li>变更到**左**车道 </li></ul>|
| ![](images/dreamview_usage_table/change-lane-right.png) | <ul><li>变更到**右**车道 </li></ul>|

在优先通行的规则下，当在交叉路口的停车标志处做出让行决策时，被让行的物体在头顶会显示让行图标

| Visual Element                                       | Depiction Explanation          |
| ---------------------------------------------------- | ------------------------------ |
| ![](images/dreamview_usage_table/0clip_image037.png) | 停止标志处的让行物体 |

##### 停止原因

如果显示了停止决策栅栏区，则停止原因展示在停止图标的右侧。可能的停止原因和对应的图标为：

| Visual Element                           | Depiction Explanation                    |
| ---------------------------------------- | ---------------------------------------- |
| ![](images/dreamview_usage_table/0clip_image040.png) | <ul><li>**前方道路侧边区域** </li></ul>|
| ![](images/dreamview_usage_table/0clip_image042.png) | <ul><li>**前方人行道** </li></ul>|
| ![](images/dreamview_usage_table/0clip_image044.png) | <ul><li>**到达目的地** </li></ul>|
| ![](images/dreamview_usage_table/0clip_image046.png) | <ul><li>**紧急停车**  </li></ul>       |
| ![](images/dreamview_usage_table/0clip_image048.png) | <ul><li> **自动驾驶模式未准备好** </li></ul>|
| ![](images/dreamview_usage_table/0clip_image050.png) | <ul><li>**障碍物阻塞道路**</li></ul> |
| ![](images/dreamview_usage_table/0clip_image052.png) | <ul><li> **前方行人穿越** </li></ul> |
| ![](images/dreamview_usage_table/0clip_image054.png) | <ul><li>**黄/红信号灯** </li></ul>|
| ![](images/dreamview_usage_table/0clip_image056.png) | <ul><li> **前方有车辆** </li></ul> |
| ![](images/dreamview_usage_table/0clip_image058.png) | <ul><li> **前方停止标志** </li></ul>|
| ![](images/dreamview_usage_table/0clip_image060.png) | <ul><li>**前方让行标志** </li></ul> |

#### 视图
可以在主视图中展示多种从**Layer Menu**选择的视图模式：

| Visual Element                           | Point of View                            |
| ---------------------------------------- | ---------------------------------------- |
| ![](images/dreamview_usage_table/default_view.png) | <ul><li>**默认视图**      </li></ul> |       |
| ![](images/dreamview_usage_table/near_view.png) | <ul><li>**近距离视图**   </li></ul> |             |
| ![](images/dreamview_usage_table/overhead_view.png) | <ul><li>**俯瞰视图**    </li></ul> |        |
| ![](images/dreamview_usage_table/map_view.png) | **地图** <ul><li> 放大/缩小：滚动鼠标滚轮或使用两根手指滑动 </li><li> 移动：按下右键并拖拽或或使用三根手指滑动</li></ul> |
