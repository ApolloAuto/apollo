## 核心软件模块：

Apollo 自动驾驶应用层提供了感知、预测、规划、控制、人机交互等数十个核心模块。这些模块之间的交互关系如下图所示：

![Apollo_software_architecture.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/Apollo_software_architecture_fd24164.png)

注：橘色箭头线段代表数据流向，黑色箭头线段代表处理流程流向。

### Perception（感知）

感知模块识别自动驾驶车辆周围的世界。感知中有两个重要的子模块：障碍物检测和交通灯检测。

### Prediction（预测）

预测模块预测感知障碍物的未来运动轨迹。

### Routing（路由）

路由模块告诉自动驾驶车辆如何通过一系列车道或道路到达其目的地。

### Planning（规划）

规划模块规划自动驾驶车辆的时间和空间轨迹。

### Control（控制）

控制模块通过产生诸如油门，制动和转向的控制命令来执行规划模块产生的轨迹。

### CanBus

CanBus是将控制命令传递给车辆硬件的接口。它还将底盘信息传递给软件系统。

### HD-Map（高精地图）

该模块类似于库（Library）。它不是发布和订阅消息，而是经常用作查询引擎支持，以提供关于道路的特定结构化信息。

### Localization（定位）

定位模块利用来自 GPS，LiDAR 和 IMU 设备的信息来评估自动驾驶车辆的位置。

### HMI（人机交互）

人机交互模块，或者说 Apollo 中 DreamView 是一个用于查看车辆状态，测试其他模块以及实时控制车辆功能的模块。

### Monitor（监控）

<!--The surveillance system of all the modules in the vehicle including hardware.-->

监控车辆中所有软硬件模块的监控系统。

### Guardian

功能安全模块，用于干预 Monitor 模块检测到的失效情况并采取相应的动作。

## 丰富的自动驾驶算法

Apollo 提供了丰富的自动驾驶算法，涵盖感知、规划、控制以及定位等自动驾驶核心技术栈。

### 多模态多视角的感知算法

- 视觉信号灯识别
- 视觉车道线检测
- 视觉障碍物识别
- 激光点云障碍物识别
- 多传感器融合障碍物识别
- 车路协同感知融合

### 基于场景的规划算法

- Public Road 规划器
- Lattice 规划器
- Navi 规划器
- RTK 规划器
- 基于模型的规划器
- 基于模型的 Open Space 规划器

### 精准的控制算法

- PID 控制器
- LQR 控制器
- MPC 控制器
- MRAC 双循环控制器

### 多融合的定位算法

- RTK 定位
- 多传感器融合（MSF）定位
