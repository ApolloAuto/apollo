该用户手册旨在帮助用户完成摄像头感知适配，实现 Camera 感知。

## 前提条件

1. 完成 [循迹搭建--车辆循迹演示](../应用实践/车辆集成教程/循迹搭建/车辆循迹演示.md)。

2. 完成 [基于摄像头的封闭园区自动驾驶搭建--感知设备集成](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/感知设备集成.md)。

3. 完成 [基于摄像头的封闭园区自动驾驶搭建--感知设备标定](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/感知设备标定.md)。

## 步骤一：配置文件的修改

| 序号  | 待修改文件                                     | 修改内容                          |
| --- | ----------------------------------------- | ----------------------------- |
| 1   | `modules/common/data/global_flagfile.txt` | 添加`--half_vehicle_width=0.43` |

## 步骤二：启动 Camera 模块

把车辆开到户外，手动控制车辆，看感知是否有数据。

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

1. 在浏览器中打开 `http://localhost:8888` 。

2. 根据实际情况选择模式。

3. 根据实际情况选择车型，并选择相应高精地图。

4. 在 Modules 启动 Canbus、Camera、GPS、Localization、Transform 模块。

### 3. 检查各模块 channel 是否正确

在 docker 中输入 `cyber_monitor` 命令并检查以下 channel：

- 使用 `上下方向键` 选择 channel，

- 使用 `右方向键` 查看 channel 详细信息。

关于 cyber_monitor 更详细使用，参见 [CyberRT_Developer_Tools](Apollo_Doc_CN_6_0/上机使用教程/实时通信框架CyberRT的使用/CyberRT介绍.md)。

| channel_name                             | 检查项目                                      |
| ---------------------------------------- | ----------------------------------------- |
| `/apollo/localization/pose`              | 确保能正常输出数据                                 |
| `/apollo/sensor/gnss/best_pose`          | 确保能正常输出数据、 `sol_type:` 选项显示为 `NARROW_INT` |
| `/apollo/sensor/camera/front_6mm/image`  | 确保能正常输出数据，帧率稳定在15HZ左右                     |
| `/apollo/sensor/camera/front_12mm/image` | 确保能正常输出数据，帧率稳定在15HZ左右                     |
| `/tf`                                    | 确保能正常输出数据                                 |
| `/tf_static`                             | 确保能正常输出数据                                 |
| `/apollo/canbus/chassis`                 | 确保能正常输出数据                                 |
| `/apollo/canbus/chassis_detail`          | 确保能正常输出数据                                 |

使用 `cyber_monitor` 查看 `/apollo/perception/obstacles` 是否正常输出，并在 DreamView+ 上查看障碍物信息：

### 5. 验证 Camera 感知效果

查看车前方运动的人或者自行车（自行车上要有人），在 DreamView+ 上查看障碍物颜色以及位置速度信息（自行车青蓝色、行人黄色、车辆绿色）。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e4472a9.png)

`/apollo/perception/obstacles` 的数据如下图所示：

![camera_adaption_dreamview_obstacle1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/8186b4dd5412a15940fe2bd22bb3da9b.png)

![camera_adaption_dreamview_obstacle2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/a29262dca0fa8f258c45e690fbecb6fd.png)

如果在 DreamView+ 上能看到障碍物并且 `/apollo/perception/obstacles` 有障碍物信息，则开环测试通过。

## NEXT

现在，您已经完成摄像头感知适配，接下来可以开始 [封闭园区自动驾驶搭建--规划适配](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/规划适配.md)

## 常见问题

### 感知的输出帧率达不到 10 帧

建议使用 `bash apollo.sh build_opt_gpu` 编译 Apollo 工程。

### 感知的障碍物位置与实际障碍物误差较大

- 确认 localization 信号状态，保证 `/apollo/sensor/gnss/best_pose` 中 `sol_type` 选项显示为 `NARROW_INT`。
- 保证各个传感器外参文件准确。
