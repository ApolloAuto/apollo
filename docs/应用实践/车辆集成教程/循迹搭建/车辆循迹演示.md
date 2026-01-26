本文档介绍在开发套件上实现循迹功能。循迹作为自动驾驶的最小闭环子集，在硬件上需要搭建自动驾驶最小系统的集成，在软件上实现相关模块的配置、标定、启动等工作，才能进行循迹的操作演示。在循迹过程中，既考验开发者在前述准备工作的掌握程度，又能够很直观的感受到自动驾驶的使用体验。

## 前提条件

1. [循迹搭建--车辆集成](../应用实践/车辆集成教程/循迹搭建/车辆集成.md)

2. [循迹搭建--Apollo 系统安装](../应用实践/车辆集成教程/循迹搭建/Apollo系统安装.md)

3. [循迹搭建--定位模块配置](../应用实践/车辆集成教程/循迹搭建/定位模块配置.md)

4. [循迹搭建--车辆动力学云标定](Apollo_Beta_Doc/应用实践/车辆集成教程/循迹搭建/车辆动力学云标定.md)

5. 确保在开阔场地、车少、人少等相对安全的情况下实验。

6. 确保至少两人操作，一人操作车辆作为安全员，一人操作自动驾驶，安全员做好随时接管准备。

## 步骤一：录制轨迹

### 1. 启动 Canbus 驱动

启动工控机后，在桌面上打开命令行终端（Terminal），进入 CAN 卡目录（默认在 home 路径下），启动 CAN 驱动：

```bash
bash ~/socketcan/start.sh
```

### 2. 启动 Apollo Docker 环境

- 如果您使用的是包管理，请您按照以下步骤操作：
  
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

> 注意：首次进入容器用`buildtool build`；如果是只编译更新的模块，使用以下命令`buildtool build -p 模块名`。

     如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：
    
     ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7e7a575.png)

- 如果您使用的是源码，请您按照以下步骤进入 docker 环境：
  
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

### 3. 启动并打开 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

1. 启动 Dreamview+
   
   - 方式一：包管理方式
     
          通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：
         
          ```bash
          aem bootstrap start --plus
          ```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`aem bootstrap stop --plus`，
> - 如果您想要重启 Dreamview+，请输入`aem bootstrap restart --plus`。

    - 方式二：源码方式
    
      通过源码方式进入 docker 环境，在 docker 环境中执行以下命令启动 Dreamview+：
    
      ```bash
      bash scripts/bootstrap.sh start_plus
      ```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`bash scripts/bootstrap.sh stop_plus`，
> - 如果您想要重启 Dreamview+，请输入`bash scripts/bootstrap.sh restart_plus`。

2. 启动成功后，在浏览器输⼊`localhost8888`⽹址打开 Dreamview+ 界面。

## 步骤二：录制轨迹

### 1. 选择循迹模式

1. 在 **Mode Settings/模式设置** > **Mode/模式** 中选择 **Vehicle Test** 模式。

2. 在 **Operations/操作** 中选择 **Waypoint_Follow/循迹** 操作。

### 2. 启动循迹驾驶所需模块

1. 在 **Modules/模块** 中，选择启动 **Canbus**、**Control**、**ExternalCmd**、**GPS**，**Localization** 和 **Transform** 按钮。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d7d75b6.png)

> 注意：ExternalCmd 是 planning 的一个子进程，循迹会用到这个子进程，路测模式只开 planning 模块就可以。

### 3. 查看连接状态

1. 查看 Component 状态。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_69f58c5.png)

> 说明：
> 
> - Component 中显示绿色 OK 为正常，可以进行下面的操作。
> - WARNING 黄色只是信号不好，模块已经启动，只要不是 ERROR 就不影响循迹。

开启 cyber_monitor，查看 channel 信息。

2. 查看 cyber_monitor 数据。
   
   进入 docker 环境，输入 `cyber_monitor`，启动的 cyber_monitor 工具中，可以看到很多 channel 信息，其中 /tf 和 /tf_static 对应 Transform 模块，/apollo/localization/pose 对应 Localization 模块，apollo/sensor/gnss/best_pose 对应 GPS 模块。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4650e42.png)
   
   使用上下方向键选择 channel /apollo/sensor/gnss/best_pose 按右方向键进入此 channel，查看 sol_type 项是否是 NARROW_INT，若为 NARROW_INT，则表示 GPS 信号良好，符合定位要求；若不为 NARROW_INT，则遥控移动下车辆，直到出现 NARROW_INT 为止。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f716d6d.png)
   
   检查所有开启模块均正常后可以进行下面的操作。

### 4. 开启轨迹录制

1. 将车辆遥控至录制轨迹起点，标记此时车辆车头方向和车辆位置，可以在车辆左前轮处画一个十字符号用于标记，示意图如下：
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3baaba6.png)

2. 在操作底栏点击 **Record/录制** 按钮，启动轨迹录制。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ee5a170.png)

### 5. 关闭轨迹录制

1. 使用遥控器遥控车辆行驶一段轨迹，这里建议走直线，车辆停止后，点击 **Stop/停止录制** 录制按钮，关闭循迹录制。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ee5a170.png)

2. 关闭轨迹录制后，在 **Record Name/记录名称** 中填写数据包名称，并单击 **Save/保存**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_dd3fd3b.png)
   
   > 注意：数据包命名规则限制：名称只能由字母、数字、下划线组成。

## 步骤三：启动循迹驾驶演示

1. 将车辆遥控至上述章节录制循迹数据时的起始位置。
   
   此时，在 Dreamview+ 中车辆前方会生成一条蓝色的轨迹线，这条轨迹线就是刚才循迹录制生成的轨迹线。检查生成的轨迹线是否清晰无毛刺，确认轨迹是否与实际路线的大致相符，如果相差很大，比如本来录制的是直行，而轨迹显示的是大幅度转弯，请重新检查定位，并重新录制新轨迹线。

> 注意：车辆的起点位置和车头朝向都尽量与循迹录制时的车辆起点位置和车头朝向保持一致。

2. 将遥控器切换到自动驾驶模式。

3. 在 Dreamview+ 中点击操作底栏的 **START/启动** 按钮，启动自动驾驶。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_b6bc07c.png)
   
   车辆就会启动循迹，沿着已有的轨迹进行自动驾驶，观察车辆运动轨迹，包括速度、方向等是否与之前录制的一致。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_aad3bd4.png)

> 注意：
> 
> - 车辆在循迹自动驾驶时，并没有使用到感知相关的传感器，遇到障碍物不会刹停或者避让，遇到紧急情况请及时遥控器接管。这也是寻找宽阔人少车少封闭场所的原因。
> - 车辆可能开始起步比较猛，请用遥控器随时接管，紧急情况下，快速按下急停按钮。

4. 车辆循迹完成后，及时遥控器接管。

<!--
## 步骤三：调试

### 1. 底盘连接是否正常

硬件连接：确保 CAN 硬件连接线 CAN0 和车辆线控底盘连接正常，进入 `apollo docker` 环境，输入以下命令：

```
cd /apollo/scripts
bash bootstrap.sh
bash canbus.sh  //启动canbus模块
cyber_monitor
```

在 cyber_monitor 界面中能看到如下的模块：

![图片](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/36ee5729e52d5d62af7aee6c8600c4fb.png)

用键盘上下箭头移动光标选择`chassis`或者`chassis_detail`，选中后按右箭头可以进入查看详细信息，这些信息即是车辆底盘信息，`chassis`消息如下：

![图片](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/17b3d0db4afb2f737beb4a2db71cd442.png)

`chassis detail`消息如下：

![图片](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/325c5674ba36df18bc00675674ffd093.png)

- 如果这些信息得到正确显示，说明 CANBUS 模块工作正常。

- 如果不能显示底盘信息，很大可能是 CANBUS 有问题，或者底盘有问题，没有向上反馈底盘数据。

保证 CANBUS 的连接正确性，才能确定计算单元(HW 3.0)计算单元可以控制车辆底盘，才能继续其它部分的调试。

### 2. canbus_teleop 的使用

CANBUS 能够连接底盘后，我们可以尝试控制底盘的运动。如果控制正常，则说明计算单元(HW 3.0)计算单元下发命令后得到正确执行。 进入 Ubuntu Docker 环境，输入以下命令：

```
cd /apollo/scripts
bash canbus_teleop.sh
```

弹出界面如下：

![图片](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/f4d91130d2965dcec5f0741637f5f034.png)

1. 根据提示按 `m0` 重置系统，按 `m1` 是开始控制底盘。
2. 按几次 `a` 或者 `d`，看看车轮是否转动。
3. 按 `g1`，挂前进档，按几次 `w`，看车辆是否前进，按几次 `s` 看车辆是否停下来。
请小心测试，不要轻易长时间连续按键，以防车辆突然高速动作发生事故。



调试过程中，常见的问题及解决办法总结如下：

### 网络断开或者网络太差无法进入 docker
报错如下：

![图片](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/bdd354adabbaaa94ae637a4e7b167176.png)

主要是因为网络没有连接上，或者网络太慢，请检查网路。

可以执行 `bash docker/scripts/dev_start.sh -n`，`-n` 参数表示不联网下载 `docker image` 更新，直接使用本地镜像。

### 无法控制汽车底盘和读取底盘信号

具体表现是车辆下发油门转向等信号，底盘不执行，也读不到底盘的相关信息，这时候的原因应该是CAN不通。解决方法如下：

1. 查看 CAN 线硬件是否连接合适。

2. 如果步骤 1 已经正确，但问题还存在，查看启动 canbus 前是否在 `docker` 外是否启动 `SocketCAN` 启动脚本 `start.sh`。

3. 如果步骤 2 已经正确，但问题还存在，查看 `apollo/data/log/canbus.INFO` 是否有报错信息。

4. 如果步骤 3 尝试了，仍然没有解决，用 CAN 调试工具，看底盘是否有信号上报，以及 apollo 执行的时候，是否有命令下发。

### Teleop 进入不了自动驾驶模式

打开 `canbus.sh` 后，执行 `canbus_teleop.sh` 始终无法进入自动驾驶模式，CAN 调试工具测量底盘信号是通的。

这种情况，有可能是 guardian 模块打开造成，guardian 模块是一种安全保护机制，开发者调试阶段，可以根据需要关掉。

打开 `/apollo/module/canbus/conf/canbus.conf`：
 修改 `--receive_guardian` 为 `--noreceive_guardian`。

### chassis_detail 不显示

1. 查看 `DreamView` 内否选择了 `Dev_Kit` 车型，如果不确认可以先选择其它车型后，重新选择`Dev_Kit`。

2. 查看 canbus 配置文件 `/apollo/module/canbus/conf/canbus.conf` 内是否打开 chassis_detail，如没有打开，把 `--noenable_chassis_detail_pub` 修改为 `enable_chassis_detail_pub`，重启 `canbus` 后生效。

3. 是否正确启动 CAN 驱动，查看启动 canbus 前是否在 `docker` 外是否启动 `SocketCAN` 启动脚本 `start.sh`。

4. 查看硬件 CAN 线接口有没有松动。

### apollo 系统第一次搭建完毕，测试发转角车辆实际转角不对

例如：下发转向角 10%，但是转动角度远超过转向最大角度的 10%。

可以在 cyber_monitor 中查看 /apollo/canbus/chassis，及时观察底盘信号，apollo 默认是 Lincoln 车型。如果使用者第一次搭建好 apollo，还没有选择车型，那么默认会按照 `mkz_example` 车型的最大转向角乘以百分比去执行。

解决方法如下：

1. 检查 modules/canbus/conf/canbus_conf.pb.txt 中的配置，brand 设置为正确的车型（例如`mkz`）。
2. 重启 canbus，再尝试。
3.（如果仍然未解决）请运行 bootstrap.sh 脚本，在 DreamView 中选择对应的车型，选择 `Dev Kit` 开发套件，然后再执行前述步骤检查转角是否正确。

### 循迹回放中，点击 Start Auto 开始后，车辆不动

查看车辆自动驾驶是否放权。


### 循迹刹车地点不准确，或者刹车停不下来

可能是标定没有做好，标定刹车部分没有做好，急刹、缓刹都要考虑。

### 循迹效果不佳

纵向效果不好，请考虑标定表和 PID 参数调整以及车辆本身的信号（例如，油门和刹车下发和上报是否一致），横向误差大，主要考虑 LQR 参数调整和车辆转向信号（下发和上报是否一致）。
-->
