该用户手册旨在帮助用户完成基于 Camera 感知的封闭园区自动驾驶。

## 前提条件

1. 完成 [基于摄像头的封闭园区自动驾驶搭建--感知设备集成](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/感知设备集成.md)。

2. 完成 [基于摄像头的封闭园区自动驾驶搭建--感知适配](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/感知适配.md)。

3. 完成 [封闭园区自动驾驶搭建--虚拟车道线制作](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/虚拟车道线制作.md)。

4. 完成 [封闭园区自动驾驶搭建--规划适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/规划适配.md)。

5. 确保在道路平整、车少人少等相对安全的情况下实验。

6. 确保至少两人操作，一人操作车辆作为安全员，一人操作自动驾驶，安全员做好随时接管准备。

## 启动流程

### 1. 编译项目，启动 DreamView+

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

1. 在浏览器中打开 `http://localhost:8888` 。

2. 选择模式为 `Vehicle Test` 。

3. 根据实际情况选择对应的车型。

4. 在 module Controller 标签页启动 Canbus、Camera、GPS、Localization、Transform 模块。

### 3. 检查各模块 channel 是否正确

在 docker 中输入 `cyber_monitor` 命令并检查以下 channel。

- 使用 `上下方向键` 选择 channel，

- 使用 `右方向键` 查看 channel 详细信息。

| channel_name                            | 检查项目                                               |
| --------------------------------------- | ------------------------------------------------------ |
| `/apollo/localization/pose`             | 确保能正常输出数据                                     |
| `/apollo/sensor/gnss/best_pose`         | 确保能正常输出数据、`sol_type:` 选项显示为`NARROW_INT` |
| `/apollo/sensor/camera/front_6mm/image` | 确保能正常输出数据、帧率在15帧左右                     |
| `/tf`                                   | 确保能正常输出数据                                     |
| `/tf_static`                            | 确保能正常输出数据                                     |
| `/apollo/canbus/chassis`                | 确保能正常输出数据                                     |
| `/apollo/canbus/chassis_detail`         | 确保能正常输出数据                                     |

使用 `cyber_monitor` 查看 `/apollo/perception/obstacles` 是否正常输出，并在 DreamView+ 上查看障碍物信息：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_571cf77.png)

`/apollo/perception/obstacles` 的数据如下图所示：

![camera_demonstration_perception_obstacle1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/02d441b2c1aed3836f1a84b88ccd898b.png)

![camera_demonstration_perception_obstacle2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/d13530a8266217bb7a407287590becc5.png)

确保在 Dreamview+ 上能看到障碍物并且 `/apollo/perception/obstacles` 有障碍物信息。

### 6. 启动 Planning、Prediction、Control 模块

在 `Modules` 中启动 Planning、Prediction、Control 模块。

### 7. 验证 Planning、Prediction、Control 模块是否启动成功

1. 在车辆可视化面板中，点击路由编辑，选取起始点和途径点。

2. 点击 **保存编辑**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e405fb7.png)

3. 单击 **发送路由** 按钮。

   从 DreamView+ 中查看，会出现一个蓝色的线以及一个红色的 stop 标志。如下图所示：

   在车前方存在人或者自行车（车上有人）时，在 Tasks 标签页查看 Planning 轨迹线，如下图所示：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e4472a9.png)

   在 docker 环境中输入命令 `cyber_monitor` 并查看 Planning channel 信息：

   ![camera_demonstration_planning_data](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/37aad4eb3d789c3ff843fa437a9157a8.png)

   如果出现上图所示轨迹线和 topic 信息，表示规划模块适配和开环测试通过，否则继续查看 log 信息进行调试。

### 8. 启动自动驾驶

在附近没有人员和车的情况下，遥控器下放权限并点击 **自动驾驶** 使车进入自动驾驶状态，在车自动驾驶这个过程中做好随时用遥控器接管，确保安全的准备。

## 常见问题

### 自动驾驶过程中车轮来回摆动，有画龙现象

这种情况一般是车辆动力学标定没有做好，建议重新进行动力学标定。

### 车辆换道、停止等效果不理想

在保证感知、定位稳定的前提下，可以尝试修改 planning 配置文件中的相关参数进行调整，详情参见 [封闭园区自动驾驶搭建--规划适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/规划适配.md)。
