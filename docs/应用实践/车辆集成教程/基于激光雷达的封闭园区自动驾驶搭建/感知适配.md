该用户手册旨在帮助用户完成激光雷达感知适配，实现 Lidar 感知。

## 前提条件

1.  完成 [循迹搭建--车辆循迹演示](../应用实践/车辆集成教程/循迹搭建/车辆循迹演示.md)。

2.  完成 [基于激光雷达的封闭园区自动驾驶搭建--感知设备集成](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知设备集成.md)。

3.  完成 [基于激光雷达的封闭园区自动驾驶搭建--感知设备标定](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知设备标定.md)。

## 步骤一：启动感知

把车辆开到户外，启动 Lidar 感知。

### 1. 编译项目，启动 Dreamview+

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

### 2. 启动所需模块

1. 在浏览器中打开`(http://localhost:8888)`。

2. 选择模式为 `Vehicle Test`。

3. 根据实际情况选择对应的车型。

4. 在 `Module Controller` 标签页启动 Canbus、GPS、Localization、Transform 模块。

   定位模块启动后，需要接收定位数据，需要等待约 1 分钟左右。

5. 打开新的终端进入 docker 环境，在新终端中输入 `cyber_monitor` 命令查看各个模块相关 channel 数据。

   这三个数据在 cyber_monitor 中均显示为绿色代表定位模块启动成功。

   ![lidar_integration_localization_check](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/a36b9d903205c7dd0cfc35add9f201fe.png)

6. 在 Dreamview 中启动 `lidar` 模块。

7. 检查 lidar 数据是否正确。

   使用 `cyber_monitor` ，查看激光雷达相关 channel 是否正常输出，

   - 使用上下方向键选择 channel，

   - 使用右方向键查看 channel 详细数据。

   单激光雷达用户，请检查如下 channel 是否正常输出：

   | 序号 | channel                                      | 帧率 |
   | ---- | -------------------------------------------- | ---- |
   | 1    | `/apollo/sensor/40p/PointCloud2`             | 10Hz |
   | 2    | `/apollo/sensor/40p/Scan`                    | 10Hz |
   | 3    | `/apollo/sensor/40p/compensator/PointCloud2` | 10Hz |

## 步骤二：验证感知效果

查看车前方 10 米处运动的人或者自行车（自行车上要有人），在 DreamView+ 上查看障碍物颜色以及位置速度信息，如下图所示：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a4e033d.png)

`/apollo/perception/obstacles`的数据如下图所示：

![lidar_adaptation_obstacles1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/02d441b2c1aed3836f1a84b88ccd898b.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a2100fc.png)

如果在 DreamView+ 上能看到障碍物并且 `/apollo/perception/obstacles` 有障碍物信息，则开环测试通过。

## NEXT

现在，您已经完成激光雷达感知适配，接下来可以开始 [封闭园区自动驾驶搭建--规划适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/规划适配.md)

<!--##  常见问题
###  感知的输出帧率达不到10帧
建议使用 `bash apollo.sh build_opt_gpu` 编译 Apollo 工程。

### 感知的障碍物位置与实际障碍物误差较大
- 确认 localization 信号状态，保证 `/apollo/sensor/gnss/best_pose` 中 `sol_type` 选项显示为 `NARROW_INT`。
- 保证各个传感器外参文件准确。
-->
