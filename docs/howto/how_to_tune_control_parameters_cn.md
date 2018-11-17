# 如何调节控制参数

## 引言
控制模块的目标是基于计划轨迹和当前车辆状态生成控制命令给车辆。

## 背景

### 输入/输出

#### 输入
* 规划轨迹
* 当前的车辆状态
* HMI驱动模式更改请求
* 监控系统

#### 输出
输出控制命令管理`canbus`中的转向、节流和制动等功能。

### 控制器介绍
控制器包括管理转向指令的横向控制器和管理节气门和制动器命令的纵向控制器。

#### 横向控制器
横向控制器是基于LQR的最优控制器。 该控制器的动力学模型是一个简单的带有侧滑的自行车模型。它被分为两类，包括闭环和开环。

- 闭环提供具有4种状态的离散反馈LQR控制器： 
  - 横向误差
  - 横向误差率
  - 航向误差
  - 航向误差率
- 开环利用路径曲率信息消除恒定稳态航向误差。


#### 纵向控制器
纵向控制器配置为级联PID +校准表。它被分为两类，包括闭环和开环。

- 闭环是一个级联PID（站PID +速度PID），它将以下数据作为控制器输入：
  - 站误差
  - 速度误差
- 开环提供了一个校准表，将加速度映射到节气门/制动百分比。


## 控制器调谐

### 实用工具
类似于[诊断](https://github.com/ApolloAuto/apollo/tree/master/modules/tools/diagnostics) 和 [realtime_plot](https://github.com/ApolloAuto/apollo/tree/master/modules/tools/realtime_plot) 可用于控制器调优，并且可以在`apollo/modules/tools/`中找到.
### 横向控制器的整定
横向控制器设计用于最小调谐力。“所有”车辆的基础横向控制器调谐步骤如下：

1. 将`matrix_q` 中所有元素设置为零.

2. 增加`matrix_q`中的第三个元素，它定义了航向误差加权，以最小化航向误差。

3. 增加`matrix_q`的第一个元素，它定义横向误差加权以最小化横向误差。

#### 林肯MKZ调谐

对于Lincoln MKZ，有四个元素指的是状态加权矩阵Q的对角线元素：

- 横向误差加权
- 横向误差率加权
- 航向误差加权
- 航向差错率加权

通过在横向控制器调谐中列出的基本横向控制器调整步骤来调整加权参数。下面是一个例子。

```
lat_controller_conf {
  matrix_q: 0.05
  matrix_q: 0.0
  matrix_q: 1.0
  matrix_q: 0.0
}
```

#### 调谐附加车辆类型

当调整除林肯MKZ以外的车辆类型时，首先更新车辆相关的物理参数，如下面的示例所示。然后，按照上面列出的基本横向控制器调整步骤*横向控制器调谐*和定义矩阵Q参数。

下面是一个例子.
```
lat_controller_conf {
  cf: 155494.663
  cr: 155494.663
  wheelbase: 2.85
  mass_fl: 520
  mass_fr: 520
  mass_rl: 520
  mass_rr: 520
  eps: 0.01
  steer_transmission_ratio: 16
  steer_single_direction_max_degree: 470
}
```

### 纵控制器的调谐
纵向控制器由级联的PID控制器组成，该控制器包括一个站控制器和一个具有不同速度增益的高速/低速控制器。Apollo管理开环和闭环的调谐通过：

- 开环: 校准表生成。请参阅[how_to_update_vehicle_calibration.md](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_update_vehicle_calibration.md)的详细步骤
- 闭环: 基于高速控制器->低速控制器->站控制器的顺序。

#### 高/低速控制器的调谐

高速控制器代码主要用于跟踪高于某一速度值的期望速度。例如：

```
high_speed_pid_conf {
  integrator_enable: true
  integrator_saturation_level: 0.3
  kp: 1.0
  ki: 0.3
  kd: 0.0
}
```
1.  首先将`kp`, `ki`, 和 `kd` 的值设为0.
2.  然后开始增加`kp`的值，以减小阶跃响应对速度变化的上升时间。
3.  最后，增加`ki`以降低速度控制器稳态误差。

一旦获得较高速度的相对准确的速度跟踪性能，就可以开始从起点开始调整低速PID控制器以获得一个舒适的加速率。

 ```
 low_speed_pid_conf {
   integrator_enable: true
   integrator_saturation_level: 0.3
   kp: 0.5
   ki: 0.3
   kd: 0.0
 }
 ```
*注意:*  当设备切换到 *Drive*时，Apollo 通常将速度设置为滑行速度。

#### 站控制器调谐

Apollo 使用车辆的站控制器来跟踪车辆轨迹基准与车辆位置之间的站误差。  一个站控制器调谐示例如下所示。
```
station_pid_conf {
  integrator_enable: true
  integrator_saturation_level: 0.3
  kp: 0.3
  ki: 0.0
  kd: 0.0
}
```
## 参考文献
1. "Vehicle dynamics and control." Rajamani, Rajesh. Springer Science & Business Media, 2011.

2. "Optimal Trajectory generation for dynamic street scenarios in a Frenet
   Frame", M. Werling et., ICRA2010
