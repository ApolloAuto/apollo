该用户手册旨在帮助用户完成激光雷达感知设备的安装、集成及数据验证。

## 前提条件

完成 [循迹搭建--车辆循迹演示](../应用实践/车辆集成教程/循迹搭建/车辆循迹演示.md)。

## 激光雷达安装与数据验证

![banner-pic.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo-D-KIT-CN/banner-pic_dac1b06.png)

### 1. 激光雷达的安装固定

激光雷达要牢靠固定安装在车顶部，建议激光雷达对地高度 1.5～1.8 米，水平放置，精度在 2 度以内。安装位置如下图：

![IMG_2450.JPG](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo-D-KIT-CN/IMG_2450_c6ef4d6.JPG)

### 2. 激光雷达的配置

修改工控机 IP 地址：

1. 激光雷达通过网线与工控机连接，将该工控机接口的 IP 修改为固定 IP地址 `192.168.1.100` ，子网掩码设置为 `255.255.255.0` 。

![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_2cc21e8.png)

2. 在浏览器中输入 `192.168.1.201` ，确认激光雷达的 Destination Lidar Port 为 1368，Destination Gps Port 为 10110。

![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo-D-KIT-CN/1_e36066c.png)

### 3. 激光雷达数据的验证

#### 1) 启动 Apollo Docker 环境

如果您使用的是包管理，请您按照以下步骤操作：

1. 打开命令行终端，切换路径到 Apollo：
   
   ```bash
   cd ~/apollo
   ```

2. 输入以下命令进入 Apollo Docker 环境：
   
   ```bash
   aem start #创建容器
   aem enter #进入容器
   buildtool build
   ```
   
   > 注意：首次进入容器用 `buildtool build` ；如果是只编译更新的模块，使用以下命令 `buildtool build -p 模块名` 。
   
   如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7e7a575.png)

如果您使用的是源码，请您按照以下步骤进入 docker 环境：

1. 打开命令行终端，切换路径到 Apollo：
   
   ```bash
   cd ~/apollo     #切换路径到apollo，注意cd与~/apollo之间有一个空格
   ```

2. 输入以下命令启动 Apollo Docker 环境：
   
   ```bash
   bash docker/scripts/dev_start.sh
   ```
   
   启动 docker 环境：
   
   ```bash
   bash docker/scripts/dev_into.sh
   ```
   
   如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：
   
   ```bash
   [user@in-runtime-docker:/apollo]$
   ```

#### 2）启动并打开 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

- 方式一：包管理方式
  
  通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：
  
  ```bash
  aem bootstrap start --plus
  ```
  
  > 注意：
  > 
  > - 如果您想要停止 Dreamview+，请输入 `aem bootstrap stop --plus` ，
  > - 如果您想要重启 Dreamview+，请输入 `aem bootstrap restart --plus` 。

- 方式二：源码方式
  
  通过源码方式进入 docker 环境，在 docker 环境中执行以下命令启动 Dreamview+：
  
  ```bash
  bash scripts/bootstrap.sh start_plus
  ```
  
  > 注意：
  > 
  > - 如果您想要停止 Dreamview+，请输入 `bash scripts/bootstrap.sh stop_plus` ，
  > - 如果您想要重启 Dreamview+，请输入 `bash scripts/bootstrap.sh restart_plus` 。

启动成功后，在浏览器输⼊ `localhost:8888` ⽹址打开 Dreamview+ 界面。

#### 3）启动所需模块。

1. 在浏览器中打开 `(http://localhost:8888)` 。
2. 选择 **Vehicle Test** 模式。
3. 根据车辆铭牌信息选择对应的车型。
4. 在 **Modules** 中启动 `GPS` 、 `Localization` 、 `Transform` 模块。

定位模块启动后，需要接收定位数据，需要等待约 1 分钟左右。

5）打开新的终端并进入 docker 环境。

6）在新终端中输入 `cyber_monitor` 命令查看 `tf`、`tf_static`、`/apollo/localization/pose` 数据，这三个数据在 `cyber_monitor` 中均显示为绿色代表定位模块启动成功。

![lidar_integration_localization_check](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/a36b9d903205c7dd0cfc35add9f201fe.png)

7）在 DreamView+ 中启动 `lidar` 模块。

#### 4）检查 lidar 数据是否正确。

使用 `cyber_monitor`，查看激光雷达数据是否正常输出。

- 使用上下方向键选择 channel，
- 使用右方向键查看 channel 详细数据，数据无异常则说明激光雷达适配成功。

单激光雷达用户，请检查如下 channel 是否正常输出：

| 序号  | channel                                           | 帧率   |
| --- | ------------------------------------------------- | ---- |
| 1   | `/apollo/sensor/hesai40p/PointCloud2`             | 10Hz |
| 2   | `/apollo/sensor/hesai40p/Scan`                    | 10Hz |
| 3   | `/apollo/sensor/hesai40p/compensator/PointCloud2` | 10Hz |

![lidar_integration_cyber_monitor](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/c5da579fc5cf086eaed731caf69b5a8c.png)

## 毫米波雷达安装与数据验证（根据需求配置）

> 注意：在目前的感知方案中，没有融合毫米波雷达数据，对于没有二次开发需求的用户可以不配置毫米波；如果您有二次开发需求，可按照本章节内容配置毫米波雷达。

- 毫米波雷达型号：continental AS 408-21

- continental AS 408-21简介：ARS408-21 是大陆 40X 毫米波雷达传感器系列中最新推出的高端产品，可以适用于不同的应用场景。ARS 408-21 很好的处理了测量性能与高安全性之间的矛盾，可实时检测目标的距离并根据当前车速判断是否存在碰撞风险可靠；具有自动故障检测功能，可识别传感器问题，并自动输出故障码鲁棒、轻量化设计；通过使用相对简单的雷达测量技术，以及在汽车行业的深度研发和批量生产基础，可以保证产品鲁棒和轻量化性能。
  
  ![lidar_integration_radar_look](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/a416478339c553cfebb0d72ffb2ca513.jpg)

### 1. 毫米波雷达接口及线序

continental AS 408-21传感器采用 12V 直流供电，使用 CAN 通信接口。使用时，通过如下图所示的连接线缆将传感器 CAN 通信接口与 Apollo 的 CAN1 口连接，电源接口接入12V直流电源(车辆上提供 12V 电源接线盒)，注意正负极。

![lidar_integration_radar_line](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/f4f043616917407f2956062bac5882e9.png)

传感器接口及定义如下图所示：其中，

- 端口 1 接 12V 直流电源；

- 端口 8 接 GND；

- 端口 4 接 CAN_L；

- 端口 7 接 CAN_H；

- 毫米波雷达 CAN 接口与工控机的毫米波雷达 CAN 接口连接。
  
  ![lidar_integration_radar_picture](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/f6f6debe85bdd0b23730b36dd5addd34.png)
  
  ![lidar_integration_radar_pin](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/3ca81d78493fac8f8d4a5c4150b75948.png)

### 2. 毫米波雷达的安装固定

毫米波雷达要牢靠固定在车身上，连接到毫米波雷达的接头要牢靠接插。离地面高 0.5 米，不能向下倾斜，向上仰 0-2 度以内，高度误差 ±0.2 米，俯仰角误差 0-2 度（向上仰小于 2 度，不能向下倾斜），翻滚角误差 ±2 度（radar 左右两侧的平齐程度），航向角误差 ±2 度（radar 是否正对前方）。

![IMG_2450.JPG](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo-D-KIT-CN/IMG_2450_c6ef4d6.JPG)

### 3. 毫米波雷达的配置及启动

#### 毫米波雷达的配置

由于只使用了前向毫米波，需要在配置文件 `modules/drivers/radar/contiradar/dag/conti_radar.dag` 中删除后向毫米波雷达（rear components）的相关配置。否则，由于后向毫米波雷达使用的 can 2，而我们的 socket can 没有 can 2，会导致 canbus 无法正常启动。修改后的 `conti_radar.dag` 文件内容如下：

```
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/radar/conti_radar/libconti_radar.so"
    components {
        class_name : "ContiRadarCanbusComponent"
        config {
            name: "conti_radar_front"
            config_file_path:  "/apollo/modules/drivers/radar/conti_radar/conf/radar_front_conf.pb.txt"
        }
    }
}
```

#### 毫米波雷达的启动

1. 正确启动 Apollo 及 DreamView。

2. 根据实际情况选择模式。

3. 根据车辆铭牌信息选择对应的车型，并打开 `radar` 模块开关。

4. 使用 `cyber_monitor` 工具，查看 `/apollo/sensor/radar/front` 数据。
   
   ![lidar_integration_radar_rostopic2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/8884274329bcab4fe8bf0b7e2b00c767.png)
   
   ![lidar_integration_radar_rostopic3](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/2e860856f27988f8b8cb1be424fefbf7.png)
   
   ![lidar_integration_radar_rostopic4](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/ee545f633e9f101eae3c56efc04e1152.png)
   
   主要参数的含义如下表所示：
   
   | 参数               | 含义       |
   | ---------------- | -------- |
   | `longitude_dist` | 距目标的纵向距离 |
   | `lateral_dist`   | 距目标的横向距离 |
   | `longitude_vel`  | 目标的纵向速度  |
   | `lateral_vel`    | 目标的横向速度  |

### 4. 毫米波雷达数据的验证

- 纵向距离（`longitude_dist`）/横向距离（`lateral_dist`）的验证：该传感器默认使用 `长距离` 模式，检测距离为 0.2~250m，在车辆正前方检测距离范围内，分别放置障碍物，查看 `/apollo/sensor/conti_radar` 话题中的 `longitute_dist`、`lateral_dist` 数据是否正常（单位：m）。下图中红色部分为长距离模式下传感器检测范围，误差在 +/-0.4m 内为正常。
  
  ![lidar_integration_radar_verify_picture](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/bf634189ae4cf2332c23c3174691decd.png)

- 纵向速度（`longitude_vel`）/横向速度（`lateral_vel`）的验证：使得目标物体以固定速度在检测范围内运动，检测数据是否在误差允许有范围内。

该传感器各项参数的测量范围及分辨率如下图所示：

![lidar_integration_radar_verify_excel](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/821609129511cefb79b3914a3b9fdf43.png)

## NEXT

现在，您已经完成激光雷达感知设备集成，接下来可以开始 [基于激光雷达的封闭园区自动驾驶搭建--感知设备标定](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知设备标定.md)。

## 常见问题

### 无法打开 Lidar 配置网页

一般情况下只要 Lidar IP、计算单元（HW3.0）与 Lidar 连接的网口 IP 处于相同号段就可以在浏览器中打开配置网页，如果确认本地IP已经正确修改还不能打开配置页面，请确认 Lidar 的 IP 地址是否被修改，可以下载 `wireshark` 来查看 Lidar IP。
