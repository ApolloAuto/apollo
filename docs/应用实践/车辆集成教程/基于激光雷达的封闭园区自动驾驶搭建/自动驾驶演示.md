该用户手册旨在帮助用户完成基于 Lidar 感知的封闭园区自动驾驶。

<!--
为了方便您快速实现基于 Lidar 的封闭园区自动驾驶演示，我们提供了 `Dev kit Closeloop` 模式：
- 建议您在进行基于 Lidar 的封闭园区自动驾驶演示时使用该模式。
- 建议使用 `Dev Kit Debug` 模式。

本文以 `Dev kit Closeloop` 模式进行说明，如果您选择使用 `Dev Kit Debug` 模式，操作方法相同。
-->

## 前提条件

1. 完成 [封闭园区自动驾驶搭建--规划适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/规划适配.md)。

2. 确保在道路平整、车少人少等相对安全的情况下实验。

3. 确保至少两人操作，一人操作计算单元（HW 3.0），一人操作遥控器，做好随时接管准备。

## 启动流程

### 步骤一：编译项目，启动 DreamView+

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

### 步骤二：启动所需模块

1. 在浏览器中打开 `http://localhost:8888` 。

2. 在 **Mode Settings/模式设置** > **Mode/模式** 中选择 **Vehicle Test** 模式。

3. 在 **Operations/操作** 中选择 **Waypoint_Follow/循迹** 操作。

4. （可选）选择车辆。

> 说明：如有对应的车辆配置，可选择对应配置，如无对应车辆配置，可用默认配置。

5. 在 **Environment Resources/环境资源** > **HD Map/高精地图** 中，选择自动驾驶需要的地图。

6. 在 **Modules/模块** 中启动 Canbus、Control、ExternalCmd、GPS、Lidar、Localization、Planning、Prediction、Transform 模块。

   定位模块启动后，需要接收定位数据，需要等待约 1 分钟左右。

7. 打开新的终端，并进入 docker 环境。

8. 在新的 docker 中输入 `cyber_monitor` 命令查看 `tf`、`tf_static`、`/apollo/localization/pose` 数据，这三个数据在 `cyber_monitor` 中均显示为绿色代表定位模块启动成功。

   ![lidar_integration_localization_check](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/a36b9d903205c7dd0cfc35add9f201fe.png)

### 步骤三：检查数据是否正确

使用 `cyber_monitor` ，查看激光雷达数据是否正常输出。

- 使用上下方向键选择 channel，

- 使用右方向键查看 channel 详细数据，数据无异常则说明激光雷达启动成功。

单激光雷达用户，请检查如下 channel 是否正常输出：

| 序号 | channel                                          | 帧率 |
| ---- | ------------------------------------------------ | ---- |
| 1    | `/apollo/sensor/lidar16/PointCloud2`             | 10Hz |
| 2    | `/apollo/sensor/lidar16/Scan`                    | 10Hz |
| 3    | `/apollo/sensor/lidar16/compensator/PointCloud2` | 10Hz |

![lidar_integration_cyber_monitor](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/c5da579fc5cf086eaed731caf69b5a8c.png)

### 步骤四：检查各模块 channel 是否正确

在 docker 中输入 `cyber_monitor` 命令并检查以下 channel。

- 使用 `上下方向键` 选择 channel，

- 使用 `右方向键` 查看 channel 详细信息。

单激光雷达用户，请检查如下 channel 是否正常输出：

| channel_name                                     | 检查项目                                               |
| ------------------------------------------------ | ------------------------------------------------------ |
| `/apollo/localization/pose`                      | 确保能正常输出数据                                     |
| `/apollo/sensor/gnss/best_pose`                  | 确保能正常输出数据、`sol_type:` 选项显示为`NARROW_INT` |
| `/apollo/sensor/lidar16/PointCloud2`             | 确保能正常输出数据                                     |
| `/apollo/sensor/lidar16/Scan`                    | 确保能正常输出数据                                     |
| `/apollo/sensor/lidar16/compensator/PointCloud2` | 确保能正常输出数据                                     |
| `/tf`                                            | 确保能正常输出数据                                     |
| `/tf_static`                                     | 确保能正常输出数据                                     |
| `/apollo/canbus/chassis`                         | 确保能正常输出数据                                     |
| `/apollo/canbus/chassis_detail`                  | 确保能正常输出数据                                     |
| `/apollo/perception/obstacles`                   | 确保能正常输出数据                                     |
| `/apollo/perception/obstacles`                   | 确保能正常输出数据                                     |

使用 `cyber_monitor` 查看 `/apollo/perception/obstacles` 是否正常输出，查看车前方 10 米处运动的人或者自行车，在 DreamView 上查看障碍物颜色以及位置速度信息（自行车青蓝色，行人黄色，车辆绿色），如下图所示：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d48fa7e.png)

`/apollo/perception/obstacles` 的数据如下图所示：

![lidar_demonstration_obstacles1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/02d441b2c1aed3836f1a84b88ccd898b.png)

![lidar_demonstration_obstacles2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/d13530a8266217bb7a407287590becc5.png)

确保在 DreamView+ 上能看到障碍物且 `/apollo/perception/obstacles` 有障碍物信息。

### 步骤五：启动 Planning、Prediction、Control 模块

在 Modules 启动 Planning、Prediction、Control 模块。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6d30cb0.png)

### 步骤六：验证 Planning、Prediction、Control 模块是否启动成功

1. 在车辆可视化面板中，点击路由编辑，选取起始点和途径点。

2. 点击 **保存编辑**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9b9860a.png)

3. 单击 **发送路由** 按钮。

   从 DreamView+ 中查看会出现一个蓝色的线以及一个红色的 stop 标志。

   在车前方存在人或者自行车（车上有人）时，在 **车辆可视化面板** 中查看 planning 轨迹线，正常情况下 planning 会重新规划轨迹，如下图所示：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f7bbf89.png)

   在 docker 环境中输入命令 `cyber_monitor` 并查看 planning channel 信息：

   ![lidar_demonstration_channel](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/37aad4eb3d789c3ff843fa437a9157a8.png)

   如果出现上图所示轨迹线和 topic 信息，表示规划模块适配和开环测试通过，否则继续查看 log 信息进行调试。

### 步骤七：启动自动驾驶

在附近没有人员和车的情况下，遥控器下放权限并点击 **自动驾驶** 使车进入自动驾驶状态，在车自动驾驶这个过程中做好随时用遥控器接管确保安全的准备。

## 常见问题

### 自动驾驶过程中车轮来回摆动，有画龙现象

这种情况一般是车辆动力学标定没有做好，建议重新进行动力学标定。

### 车辆换道、停止等效果不理想

在保证感知、定位稳定的前提下，可以尝试修改 planning 配置文件中的相关参数进行调整，详情参见 [封闭园区自动驾驶搭建--规划适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/规划适配.md)。
