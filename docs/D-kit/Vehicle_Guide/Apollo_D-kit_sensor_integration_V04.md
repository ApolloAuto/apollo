# **Apollo传感器集成说明**

传感器上车集成前，请优先或同步完成工控机的Apollo系统安装，教程详见“循迹搭建——Apollo系统安装”。工控机安装Apollo系统时开采用自带的直流电源模块供电，显示器可使用电脑显示器。

## **快速介绍**

请您在传感器集成前仔细阅读本文档。本文档介绍了传感器供电线束安装的注意事项。

## **Apollo上装电气原理图**

Apollo上装电气原理图如下图所示：

![Apollo_electrical_schematic](./images/Apollo_electrical_schematic.jpg)


通过电气原理图可知，6108/8108工控机（IPC）由底盘独立提供24V电源，显示器、激光雷达、毫米波雷达、IMU和路由器则由底盘通过保险盒提供12V电源。

## **1. 上装传感器供电线束简介**

Apollo电气原理图中的供电部分集成在了一条供电线束中，如下图所示。

![apollo_power_wire_harness](./images/apollo_power_wire_harness.jpg)

保险丝盒取消了负极接线，负极回路集成在线束内部。

底盘后部电气面板如下图所示：

![Electrical_panel](./images/Motor_Enable_1.jpg)

上装供电插座的脚位定义如图所示:1、Pin1—12V+；2、Pin2—12V-；3、Pin3—24V+；4、Pin4—24V-。

![apollo_to_chassis](./images/apollo_to_chassis.jpg)

底盘can口通过db9延长线与工控机相连。上装供电线通过4pin航插与底盘供电插座相连。
**<font color=#FF0000> 提示：请在检查供电线束与传感器连接插头极性之后，再连接供电线束与底盘供电口的航插。</font>**

## **2. 传感器电源接口介绍**

### **1、工控机电源接口**

![8108_Power_1](./images/8108_Power_1.jpg)

![8108_Power_2](./images/8108_Power_2.jpg)

![8108_Power_3](./images/8108_Power_3.jpg)


### 2、M2电源接口

M2自带数据线缆为1分5线缆，一端为与工控机相连的航插接头，另一端为M2电源输入口，网口、激光雷达授时输出口，USB输出口，db9输出口（M2配置串口）。详细如下图所示：

![M2_1](./images/M2_1.jpg)

注意：M2线缆电源输入口为白色2pin插头，红色为12V正极，蓝色为12V负极；上装线束M2供电插头为蓝色2pin插头，红色为12V正极，黑色色为12V负极，对插后红色对红色，黑色对蓝色。连接插头时请确认线色或用万用表检查线束端供电插头电源极性。连接如下图所示：

![M2_2](./images/M2_2.jpg)

### 3、毫米波雷达电源接口

毫米波雷达线束，一端为毫米波雷达连接插头，另一端是电源插头2pin凤凰端子和CAN口。

![Radar_1](./images/Radar_1.jpg)

![Radar_2](./images/Radar_2.jpg)

**<font color=#FF0000 >注意：毫米波雷达线束电源插头：红色正+，蓝色负-；线束供电端为红色正+，黑色负-；再插接时，保证红对红，黑对蓝。</font>**

### 4、激光雷达电源接口

![Lidar_1](./images/Lidar_1.jpg)

![Lidar_2](./images/Lidar_2.jpg)

![Lidar_3](./images/Lidar_3.jpg)

![Lidar_4](./images/Lidar_4.jpg)


### 5、屏幕供电

车载屏幕供电线为单独一段线束：1端为DJ7021-1.5-11插件（公头），连接供电线束端DJ7021-1.5-21（母头）；另一端为DC5.5mm插头，接口如下图所示：

![power_line_of_display](./images/Display.jpg)

### 6、路由器供电

路由器线束断供电插头为DC5.5直流插头，如下图所示：

![apollo_power_wire_harness](./images/apollo_power_wire_harness.jpg)

## **3. 传感器固定介绍**

### 1、工控机固定

![IPC_integration_1](./images/IPC_integration_1.jpg)

### 2、M2及线束保险丝盒固定

![IMU_fusebox_integration](./images/IMU.jpg)

### 3、激光雷达固定

![lidar_integration_installation](./images/lidar_integration_installation.jpg)

#### 3.1激光雷达支架固定

![lidar_bracket](./images/lidar_bracket.jpg)

#### 3.2激光雷达固定

![lidar_integration_look](./images/lidar_integration_look.jpg)

![Lidar_integration](./images/Lidar_integration.jpg)

![lidar_integration_slot](./images/lidar_integration_slot.jpg)

### 4、摄像头固定

摄像头安装位置如下图

![camera_integration_installation](./images/camera_integration_installation.jpg)

![camera_integration_overview](./images/camera_integration_overview.jpg)

![camera_integration_look](./images/camera_integration_look.jpg)

![camera_integration_bolting](./images/camera_integration_bolting.jpg)

![camera_integration_line](./images/camera_integration_line.jpg)

![camera_integration_background](./images/camera_integration_background.jpg)

### 5、激光雷达与摄像头相对水平调整

![Lidar_integration_levelling](./images/Lidar_integration_levelling.jpg)

![camera_integration_levelling](./images/camera_integration_levelling.jpg)

### 6、毫米波雷达固定

![radar_look](./images/lidar_integration_radar_look.jpg)

![radar_integration](./images/radar_integration.jpg)

![radar_installation_position](./images/lidar_integration_radar_installation_position.jpg)

